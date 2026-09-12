// rtkrcv_node:接线 Task 1-6 的五个组件,监管 RTKLIB rtkrcv 子进程,把它的
// llh 解流转成 gnss_msgs/RtkFix 发布出去,并把 $SAT 状态行原样转发供轮 3
// 的 gnss_diag 消费(见 task-7-brief.md)。
//
// 接线顺序(brief §Step 4,顺序本身是要点——rtkrcv 首次连接本机端口如果连不上,
// 会白白吃掉一次退避周期什么都不干):
//   1. 读参数
//   2. LocalReserver 起两个本机服务端口(corr/obs),供 rtkrcv 当 tcpcli 连入
//   3. render_rtkrcv_conf() 写 conf 文件(用 LocalReserver 实际绑定到的端口,
//      不是请求的端口——见 write_conf() 前的 IMPORTANT 说明)
//   4. ProcessSupervisor 起 rtkrcv;后台起一个不阻塞构造函数的线程打印
//      "spawned pid" 日志
//   5. TcpStream 连 rtkrcv 的解算输出端口,按行切分喂 parse_llh_solution
//   6. 订阅 corrections/raw_obs,broadcast 进两个 LocalReserver
//   7. 轮询 run_dir 下最新的 rtkrcv_*.stat,tail 新增字节,原样发布
//
// 析构顺序:先停我们自己起的两个辅助线程(pid 日志、stat tail——它们只是
// 观察者,不参与 Task 1-6 组件之间的数据流,谁先停都不影响正确性),然后
// 严格按 brief 规定的镜像顺序停止 Task 1-6 的组件:先停 ProcessSupervisor
// (杀 rtkrcv,它不会再往 sol TcpStream 写数据),再停 TcpStream(它的线程
// 不会再回调 on_data),最后停两个 LocalReserver——全部发生在 node 和它的
// publisher 被销毁之前。这些组件都在各自的线程上回调,回调打到一个已经
// 析构的 publisher 上就是一次崩溃(与 rtcm_bridge_node.cpp 在
// rclcpp::shutdown() 之前 streams.clear() 是同一个道理)。

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <gnss_msgs/msg/raw_stream.hpp>
#include <gnss_msgs/msg/rtk_fix.hpp>

#include "gnss_bringup/local_reserver.hpp"
#include "gnss_bringup/process_supervisor.hpp"
#include "gnss_bringup/rtcm_bridge_params.hpp"   // is_valid_port_for_direction:与 rtcm_bridge_node 共用的端口校验
#include "gnss_bringup/rtk_fix_mapping.hpp"       // to_rtk_fix + LineSplitter + plan_stat_tail
#include "gnss_bringup/rtkrcv_conf.hpp"
#include "gnss_bringup/tcp_stream.hpp"
#include "gnss_core/rtkstat.hpp"                  // parse_llh_solution

namespace {

namespace fs = std::filesystem;

// review round 1 的 Minor(promoted):目录扫描全部走带 error_code 的重载,
// 不让 directory_iterator::operator++()(range-for 用的正是这个会抛异常的
// 版本)在 run_dir 运行期间被删除/掉线(比如挂载点掉了)时把异常捅到
// stat_tail_loop 这个独立线程上——那样会直接 std::terminate() 干掉整个
// 进程,而不是简单地"这一轮没扫到东西"。
std::vector<gnss_bringup::StatFileInfo> list_stat_candidates(const fs::path& run_dir) {
  std::vector<gnss_bringup::StatFileInfo> out;
  std::error_code ec;
  if (!fs::exists(run_dir, ec) || ec) return out;

  fs::directory_iterator it(run_dir, ec);
  if (ec) return out;
  const fs::directory_iterator end;

  while (it != end) {
    std::error_code fec;
    const fs::directory_entry entry = *it;
    const bool is_reg = entry.is_regular_file(fec);
    if (!fec && is_reg) {
      const auto& p = entry.path();
      if (p.extension() == ".stat") {
        std::error_code mec, sec;
        const auto mtime = entry.last_write_time(mec);
        const auto size = entry.file_size(sec);
        if (!mec && !sec) {
          gnss_bringup::StatFileInfo info;
          info.path = p.string();
          info.size = static_cast<uint64_t>(size);
          info.mtime = mtime.time_since_epoch().count();
          out.push_back(std::move(info));
        }
      }
    }
    it.increment(ec);
    if (ec) break;
  }
  return out;
}

}  // namespace

namespace gnss_bringup {

// 把 Task 1-6 的组件接成一个可运行的 rtkrcv 监管节点。
class RtkrcvSupervisorNode {
public:
  explicit RtkrcvSupervisorNode(rclcpp::Node* node) : node_(node) {
    // review round 2 的 Important:构造函数体内任何一步抛异常,这个对象都
    // 不会被认为构造完成——~RtkrcvSupervisorNode() 不会被调用,C++ 只会
    // 自动析构“已经构造完成的成员子对象”。LocalReserver/TcpStream/
    // ProcessSupervisor 都在自己的析构函数里正确 stop()/join() 了各自内部
    // 的线程,天然安全;但 pid_log_thread_/stat_thread_ 是裸 std::thread
    // 成员——一个仍然 joinable 的 std::thread 被析构会直接
    // std::terminate() 干掉整个进程,绕开 main() 里那个专门为了把配置
    // 错误报成一行 RCLCPP_ERROR 而写的 catch(复现:
    // -p corrections_topic:="bad topic!" 在这个修复之前会 SIGABRT、
    // core dump,并留下一个孤儿 rtkrcv 子进程)。
    //
    // 这里统一兜底:不变量是“构造函数还可能抛异常的任何一个时间点上,不
    // 存在一个 joinable 的线程成员活到这个对象的生命周期结束之外”——用
    // try/catch(...) 兜住,先把可能已经起来的两个线程停掉、join 干净,
    // 再原样把异常继续往外抛给 main() 的 catch,而不是依赖“把起线程的语句
    // 挪到最后一行”这种容易被后续改动悄悄破坏的顺序假设。
    try {
      read_params();
      start_local_reservers();
      write_conf();
      start_supervisor();
      start_pid_logger();
      connect_solution_stream();
      subscribe_uplink_streams();
      start_stat_tailer();
    } catch (...) {
      stop_pid_logger();
      stop_stat_tailer();
      throw;
    }
  }

  // 显式按上面文件头注释规定的顺序停止,而不是依赖成员声明顺序在析构时
  // 自动倒序析构——那样一来这里的顺序意图会散落在类定义的字段排列里,
  // 后来者改一下字段顺序就能悄悄改变停止顺序而不自知。
  ~RtkrcvSupervisorNode() {
    stop_pid_logger();
    stop_stat_tailer();
    if (supervisor_) supervisor_->stop();
    if (sol_stream_) sol_stream_->stop();
    obs_reserver_.stop();
    corr_reserver_.stop();
  }

  RtkrcvSupervisorNode(const RtkrcvSupervisorNode&) = delete;
  RtkrcvSupervisorNode& operator=(const RtkrcvSupervisorNode&) = delete;

private:
  // ---------- Step 1: 参数 ----------
  void read_params() {
    // obs_port/corr_port 是 LocalReserver 要 listen() 的本机端口——0 合法,
    // 表示让内核选(start_local_reservers() 之后立刻用 bound_port() 回填
    // conf_,见下面的说明)。
    conf_.obs_port = static_cast<int>(declare_port("obs_port", conf_.obs_port, /*allow_zero=*/true));
    conf_.corr_port =
        static_cast<int>(declare_port("corr_port", conf_.corr_port, /*allow_zero=*/true));
    // final-fix-wave 第 1 项:sol_port 不是 listen 端口——这个节点是客户端,
    // 拿它去 connect_solution_stream() 里 connect() rtkrcv 的 outstr1
    // (tcpsvr)。0 在这个方向上没有任何操作系统语义,会一路传到
    // TcpStream::run_client() 里的 connect(127.0.0.1:0),既不报错也永远
    // 连不上——worker 只会一次次退避重连,日志里只有一行不痛不痒的
    // "connect() failed",现场表现为"配置检查全部通过、节点看起来在跑,
    // 但 ~/rtk_fix 永远没有输出"。已实测复现:-p sol_port:=0。
    conf_.sol_port =
        static_cast<int>(declare_port("sol_port", conf_.sol_port, /*allow_zero=*/false));
    conf_.obs_format = node_->declare_parameter<std::string>("obs_format", conf_.obs_format);
    conf_.corr_format = node_->declare_parameter<std::string>("corr_format", conf_.corr_format);
    conf_.pos_mode = node_->declare_parameter<std::string>("pos_mode", conf_.pos_mode);
    conf_.navsys = static_cast<int>(node_->declare_parameter<int>("navsys", conf_.navsys));
    conf_.elmask = node_->declare_parameter<double>("elmask", conf_.elmask);
    conf_.ar_mode = node_->declare_parameter<std::string>("ar_mode", conf_.ar_mode);

    run_dir_ = node_->declare_parameter<std::string>("run_dir", "/tmp/rtkrcv_run");
    binary_ = node_->declare_parameter<std::string>("binary", "rtkrcv");
    extra_args_ = node_->declare_parameter<std::vector<std::string>>(
        "args", std::vector<std::string>{});

    restart_delay_s_ = declare_positive_seconds("restart_delay_s", 5.0);
    crash_loop_life_s_ = declare_positive_seconds("crash_loop_life_s", 30.0);
    max_restart_delay_s_ = declare_positive_seconds("max_restart_delay_s", 60.0);

    sol_initial_backoff_s_ = declare_positive_seconds("sol_initial_backoff_s", 1.0);
    sol_max_backoff_s_ = declare_positive_seconds("sol_max_backoff_s", 30.0);
    // review round 2 的"一个需要明说的决定":TcpStream 把 idle_timeout_s<=0
    // 当成一个(未在头文件里明文承诺、但 tcp_stream.cpp:247 确实实现了的)
    // 哨兵值——"彻底关闭空闲超时",poll() 会永久阻塞直到真正有数据或者被
    // stop() 打断。is_positive_finite_seconds 因此会拒绝
    // sol_idle_timeout_s=0。这是刻意的选择,不是校验逻辑复用时漏掉的
    // 意外:这个节点存在的意义就是"rtkrcv 连接卡死了要能被发现并自动
    // 恢复",允许运维通过 0 关掉这道空闲检测,等于允许在现场悄悄关掉这个
    // 节点最核心的自愈能力,而且这条哨兵语义没有在 TcpStreamConfig 的头
    // 文件注释里正式承诺过,不适合在这里对外暴露成"合法用法"。现在没有任何
    // 地方设置这个参数为 0,所以不存在需要放行的既有用例;以后如果确实需要
    // "禁用空闲超时"这个能力,应该作为一个独立的、显式命名的参数
    // (例如 disable_sol_idle_timeout)引入,而不是让 0 通过校验缺口悄悄
    // 变成一个隐藏用法。
    sol_idle_timeout_s_ = declare_positive_seconds("sol_idle_timeout_s", 30.0);

    corr_topic_ = node_->declare_parameter<std::string>("corrections_topic", "/gnss/rtcm_corrections");
    obs_topic_ = node_->declare_parameter<std::string>("raw_obs_topic", "/gnss/raw_obs");
    frame_id_ = node_->declare_parameter<std::string>("frame_id", "gnss");

    pos_opts_.leap_seconds = static_cast<int>(node_->declare_parameter<int>("leap_seconds", 18));
    // render_rtkrcv_conf() 固定写 out-timesys=gpst(见 rtkrcv_conf.cpp),
    // 与 parse_llh_solution 这边的默认时间系统必须一致,不做成参数以免
    // 两边失配。
    pos_opts_.default_time_system = gnss_core::PosTimeSystem::GPST;

    stat_poll_interval_s_ = declare_positive_seconds("stat_poll_interval_s", 0.2);

    if (run_dir_.empty()) {
      throw std::invalid_argument("run_dir 不能为空");
    }
  }

  int64_t declare_port(const std::string& name, int default_value, bool allow_zero) {
    // declare_parameter<int> 实际返回 int64_t(与 rtcm_bridge_node.cpp 同样的
    // 注意事项),这里统一按 int64_t 接住再校验范围。
    const int64_t v = node_->declare_parameter<int>(name, default_value);
    // is_valid_port_for_direction 的第二个参数字面意思是"listen 模式下 0
    // 合法",allow_zero 就是那个语义在这里的名字——sol_port 传 false 是因为
    // 它是 connect 方向,不是因为它本身会 listen。
    if (!gnss_bringup::is_valid_port_for_direction(v, allow_zero)) {
      throw std::invalid_argument(
          name + ": 非法端口号 " + std::to_string(v) + "(合法范围 0-65535" +
          (allow_zero ? "" : ";这个端口是连接方向,不接受 0") + ")");
    }
    return v;
  }

  // review round 1 的 Minor(promoted):轮询间隔/退避秒数如果是 0、负数或
  // 非有限值,会让对应的 wait_for/退避逻辑退化成热循环或钉死行为——与
  // is_valid_port 同一个道理,提前挡住、报得清楚。
  double declare_positive_seconds(const std::string& name, double default_value) {
    const double v = node_->declare_parameter<double>(name, default_value);
    if (!is_positive_finite_seconds(v)) {
      throw std::invalid_argument(name + ": 必须是正数秒(收到 " + std::to_string(v) + ")");
    }
    return v;
  }

  // ---------- Step 2: 先起本机服务,rtkrcv 才有地方连 ----------
  void start_local_reservers() {
    // final-fix-wave 第 2 项:LocalReserver 自己不做任何日志/回调
    // (它刻意不碰 ROS,便于单测),accept_loop 因不可恢复错误永久退出时
    // 原来完全没有任何对外信号——现场表现为"节点看起来在跑,
    // corrections/raw_obs 却再也传不到 rtkrcv"。这里补上 ERROR 级别的
    // 回调,这是现场唯一能看到的信号,不能比 TcpStream 那边的终态日志更弱。
    if (!corr_reserver_.start(conf_.corr_port, "127.0.0.1", [this](const std::string& detail) {
          RCLCPP_ERROR(node_->get_logger(),
                       "corr LocalReserver 的 accept 线程已永久退出,不会再接受任何新连接,"
                       "corrections 上行数据从此丢失(需要人工介入/重启节点): %s",
                       detail.c_str());
        })) {
      throw std::runtime_error("无法监听 corr_port=" + std::to_string(conf_.corr_port));
    }
    if (!obs_reserver_.start(conf_.obs_port, "127.0.0.1", [this](const std::string& detail) {
          RCLCPP_ERROR(node_->get_logger(),
                       "obs LocalReserver 的 accept 线程已永久退出,不会再接受任何新连接,"
                       "raw_obs 上行数据从此丢失(需要人工介入/重启节点): %s",
                       detail.c_str());
        })) {
      throw std::runtime_error("无法监听 obs_port=" + std::to_string(conf_.obs_port));
    }

    // review round 1 的 Important 1:conf 必须写 LocalReserver 实际绑定到的
    // 端口,不是请求的端口——corr_port/obs_port=0(让内核选)会让
    // conf 里写出 "127.0.0.1:0",rtkrcv 拿着这个地址去连,永远连不上任何
    // 东西,而且不会有任何错误日志。请求非 0 端口时 bound_port() 应该等于
    // 请求值,这里统一回填不额外分支,两种情况都覆盖。
    conf_.corr_port = corr_reserver_.bound_port();
    conf_.obs_port = obs_reserver_.bound_port();

    RCLCPP_INFO(node_->get_logger(), "本机服务已就绪: corr_port=%d obs_port=%d",
                conf_.corr_port, conf_.obs_port);
  }

  // ---------- Step 3: 渲染 + 落盘 conf ----------
  void write_conf() {
    std::error_code ec;
    fs::create_directories(run_dir_, ec);
    if (ec) {
      throw std::runtime_error("无法创建 run_dir=" + run_dir_ + ": " + ec.message());
    }

    std::string rendered;
    try {
      rendered = render_rtkrcv_conf(conf_);
    } catch (const std::invalid_argument& e) {
      // render_rtkrcv_conf 对非法输入(elmask 越界、字符串里带换行/=)抛
      // invalid_argument——这里必须接住,转成一条清楚的启动失败日志,而不是
      // 让异常继续往外捅、把已经起好的 LocalReserver 状态搞得不清不楚。
      throw std::runtime_error(std::string("rtkrcv.conf 参数非法: ") + e.what());
    }

    conf_path_ = (fs::path(run_dir_) / "rtkrcv.conf").string();
    std::ofstream ofs(conf_path_, std::ios::trunc);
    if (!ofs) {
      throw std::runtime_error("无法写入 " + conf_path_);
    }
    ofs << rendered;
    ofs.close();
    RCLCPP_INFO(node_->get_logger(), "已写入 %s", conf_path_.c_str());
  }

  // ---------- Step 4: 起 rtkrcv ----------
  void start_supervisor() {
    ProcessSupervisorConfig scfg;
    scfg.binary = binary_;
    scfg.cwd = run_dir_;
    scfg.restart_delay_s = restart_delay_s_;
    scfg.crash_loop_life_s = crash_loop_life_s_;
    scfg.max_restart_delay_s = max_restart_delay_s_;
    scfg.args = {"-s", "-nc", "-r", "2", "-o", conf_path_};
    for (const auto& a : extra_args_) scfg.args.push_back(a);

    supervisor_ = std::make_unique<ProcessSupervisor>(scfg);
    supervisor_->start();
    RCLCPP_INFO(node_->get_logger(), "已启动 rtkrcv 监管: binary=%s cwd=%s",
                binary_.c_str(), run_dir_.c_str());
  }

  // review round 1 的 Minor(promoted):原来这是构造函数里的一段阻塞轮询
  // (最多 3s),后果是——1) 节点在 spin() 之前就可能吃满 3s,这段时间
  // Ctrl-C 没有 executor 在跑,进不了 rclcpp 的信号处理路径;2) 订阅
  // corrections/raw_obs 的时机被推迟,窗口期内上行发布的差分字节会被
  // 直接丢弃。改成一个独立的、有界的后台线程,不阻塞构造函数继续往下走,
  // 只是打一行诊断日志,找不到 pid 也不影响接线。
  void start_pid_logger() {
    pid_log_running_.store(true);
    pid_log_thread_ = std::thread([this] { pid_log_loop(); });
  }

  void stop_pid_logger() {
    {
      std::lock_guard<std::mutex> lk(pid_log_mutex_);
      pid_log_running_.store(false);
    }
    pid_log_cv_.notify_all();
    if (pid_log_thread_.joinable()) pid_log_thread_.join();
  }

  void pid_log_loop() {
    // review round 2 的"也要修"项:与 stat_tail_loop 同样的道理——这是一个
    // 独立线程,任何逃逸的异常都会 std::terminate() 干掉整个进程,而不是
    // 简单地放弃这一次打日志。stat_tail_loop 这一轮已经补了 try/catch,
    // 这里补齐,让两个辅助线程在“绝不能让异常逃出去”这件事上保持对称。
    try {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
      for (;;) {
        const int pid = supervisor_->last_child_pid();
        if (pid > 0) {
          RCLCPP_INFO(node_->get_logger(), "rtkrcv spawned pid %d", pid);
          return;
        }
        std::unique_lock<std::mutex> lk(pid_log_mutex_);
        if (!pid_log_running_.load()) return;
        if (std::chrono::steady_clock::now() >= deadline) break;
        pid_log_cv_.wait_for(lk, std::chrono::milliseconds(20),
                              [this] { return !pid_log_running_.load(); });
        if (!pid_log_running_.load()) return;
      }
      RCLCPP_WARN(node_->get_logger(),
                  "3s 内未观测到 rtkrcv 子进程 pid,继续接线(supervisor 会持续重试)");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "pid 日志线程出错(已放弃打印 pid): %s", e.what());
    } catch (...) {
      // 非 std::exception 派生的异常同样不能让它逃出这个线程。
    }
  }

  // ---------- Step 5: 连解算输出,发布 RtkFix ----------
  void connect_solution_stream() {
    rtk_fix_pub_ = node_->create_publisher<gnss_msgs::msg::RtkFix>(
        "~/rtk_fix", rclcpp::QoS(100).reliable());

    TcpStreamConfig cfg;
    cfg.host = "127.0.0.1";
    cfg.port = conf_.sol_port;
    cfg.listen = false;   // 我们是客户端,连 rtkrcv 的 outstr1(tcpsvr)
    cfg.initial_backoff_s = sol_initial_backoff_s_;
    cfg.max_backoff_s = sol_max_backoff_s_;
    cfg.idle_timeout_s = sol_idle_timeout_s_;

    sol_stream_ = std::make_unique<TcpStream>(
        cfg,
        [this](const uint8_t* d, size_t n) { on_solution_bytes(d, n); },
        [this](bool connected, const std::string& detail, bool terminal) {
          // final-fix-wave 第 2 项:terminal=true 说明这条 TcpStream 的
          // worker 线程已经永久退出(sol_port 这条是客户端流,目前唯一会
          // 触发的终态路径是 eventfd() 分配失败),不会再有任何重连尝试。
          // 这个节点不会自动重启它,也没有健康检查话题,现场只能靠这行
          // 日志发现——必须是 ERROR。
          if (terminal) {
            RCLCPP_ERROR(node_->get_logger(),
                         "sol stream: %s %s(worker 线程已永久退出,不会再有任何"
                         "重连尝试,需要人工介入/重启节点)",
                         connected ? "connected" : "disconnected", detail.c_str());
          } else {
            RCLCPP_INFO(node_->get_logger(), "sol stream: %s %s",
                        connected ? "connected" : "disconnected", detail.c_str());
          }
          if (!connected) {
            // review round 1 的 Important 4:rtkrcv 被杀/连接断开时,
            // sol_splitter_ 里可能还留着一段没等到 '\n' 的半行。如果不清掉,
            // 重连后新流的第一条完整行会被拼在这段陈旧残留后面——拼出来的
            // 东西如果凑巧还有 >=10 个字段,parse_llh_solution 会"解析
            // 成功",发布一条新旧字段混杂的假解,新鲜的 header.stamp 配上
            // 一段过期/错位的数据,下游没有任何办法分辨。这个回调和
            // on_data 跑在同一个 TcpStream 线程上,不存在和 on_solution_bytes
            // 并发的竞态。
            sol_splitter_.reset();
          }
        });
    sol_stream_->start();
  }

  // TcpStream 给的是任意切分的字节块,不是行——用 LineSplitter 攒缓冲、
  // 按 '\n' 切分,半行留到下一次回调(brief 点名的坑:一行被切在两个块
  // 中间,如果直接当成两条破损的行解析,就是静默丢数据)。
  void on_solution_bytes(const uint8_t* d, size_t n) {
    const size_t prev_overflow = sol_splitter_.overflow_count();
    for (const auto& line : sol_splitter_.feed(d, n)) {
      gnss_core::PosRecord rec;
      if (!gnss_core::parse_llh_solution(line, rec, pos_opts_)) continue;

      auto msg = to_rtk_fix(rec);
      msg.header.stamp = node_->now();     // 接收时刻;msg.gnss_time 是解算历元,
                                            // 两者之差是轮 3 的 stamp_skew 诊断输入
      msg.header.frame_id = frame_id_;
      rtk_fix_pub_->publish(msg);
    }
    if (sol_splitter_.overflow_count() != prev_overflow) {
      // 限流:超限本身在真正配错(binary format / 接错端口)的场景下会
      // 持续发生,不节流会刷屏。
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "sol 流单行超过上限被丢弃(累计 %zu 次)——"
                            "检查 outstr1-format / sol_port 是否接对了流",
                            sol_splitter_.overflow_count());
    }
  }

  // ---------- Step 6: 差分/原始观测上行,喂给本机服务再转给 rtkrcv ----------
  void subscribe_uplink_streams() {
    corr_sub_ = node_->create_subscription<gnss_msgs::msg::RawStream>(
        corr_topic_, rclcpp::QoS(100).reliable(),
        [this](const gnss_msgs::msg::RawStream::SharedPtr msg) {
          corr_reserver_.broadcast(msg->data.data(), msg->data.size());
        });
    obs_sub_ = node_->create_subscription<gnss_msgs::msg::RawStream>(
        obs_topic_, rclcpp::QoS(100).reliable(),
        [this](const gnss_msgs::msg::RawStream::SharedPtr msg) {
          obs_reserver_.broadcast(msg->data.data(), msg->data.size());
        });
    RCLCPP_INFO(node_->get_logger(), "订阅上行: %s -> corr, %s -> obs",
                corr_topic_.c_str(), obs_topic_.c_str());
  }

  // ---------- Step 7: 原样转发 $SAT 状态行 ----------
  void start_stat_tailer() {
    stat_pub_ = node_->create_publisher<gnss_msgs::msg::RawStream>(
        "~/stat", rclcpp::QoS(100).reliable());
    stat_running_.store(true);
    stat_thread_ = std::thread([this] { stat_tail_loop(); });
  }

  void stop_stat_tailer() {
    {
      std::lock_guard<std::mutex> lk(stat_mutex_);
      stat_running_.store(false);
    }
    stat_cv_.notify_all();
    if (stat_thread_.joinable()) stat_thread_.join();
  }

  void stat_tail_loop() {
    bool first_poll = true;

    while (stat_running_.load()) {
      // review round 1 的 Minor(promoted):整个轮询体包一层 try/catch——
      // 这是一个没有人 join 失败路径的独立线程,任何逃逸的异常
      // (fs::filesystem_error、rclcpp 发布失败……)都会变成
      // std::terminate() 干掉整个进程,而不是这一轮跳过、下一轮重试。
      try {
        run_one_stat_poll(first_poll);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "stat tail 轮询出错(已跳过这一轮): %s", e.what());
      }
      first_poll = false;

      std::unique_lock<std::mutex> lk(stat_mutex_);
      stat_cv_.wait_for(lk, std::chrono::duration<double>(stat_poll_interval_s_),
                         [this] { return !stat_running_.load(); });
    }
  }

  void run_one_stat_poll(bool first_poll) {
    const auto candidates = list_stat_candidates(run_dir_);
    const auto decision =
        plan_stat_tail(candidates, stat_current_file_, stat_current_offset_, first_poll);
    if (!decision.has_target) return;

    stat_current_file_ = decision.file;
    stat_current_offset_ = decision.read_from;
    if (decision.read_to <= decision.read_from) return;   // 没有新内容

    stat_current_offset_ =
        decision.read_from + tail_read_and_publish(decision.file, decision.read_from, decision.read_to);
  }

  // review round 1 的 Important 3(第二条):按块读取/发布,不是不管文件
  // 涨到多大都一次性 read() 成一个 vector 再塞进一条 RawStream——长时间跑
  // 下来 .stat 文件可能到几百 MB,首次追上historical内容(或者 tail 线程
  // 卡顿一阵之后一次性追平)时,不加节制的话就是一次巨大分配 + 一条巨大的
  // DDS 消息。返回实际读到并发布出去的字节数,供调用方推进 offset——即使
  // 中途遇到短读,offset 也只会推进到真正发出去的那一段,不会把没读到的
  // 部分当成"已经处理过"而漏掉。
  uint64_t tail_read_and_publish(const std::string& file, uint64_t from, uint64_t to) {
    constexpr size_t kChunkBytes = 64 * 1024;

    std::ifstream ifs(file, std::ios::binary);
    if (!ifs) return 0;
    ifs.seekg(static_cast<std::streamoff>(from));
    if (!ifs) return 0;

    uint64_t total = 0;
    std::vector<uint8_t> buf(kChunkBytes);
    while (from + total < to) {
      const size_t want = static_cast<size_t>(std::min<uint64_t>(kChunkBytes, to - from - total));
      ifs.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(want));
      const auto got = static_cast<uint64_t>(ifs.gcount());
      if (got == 0) break;   // 文件比预期的 to 更短(被截断),下一轮 plan_stat_tail 会处理

      gnss_msgs::msg::RawStream msg;
      msg.header.stamp = node_->now();
      msg.header.frame_id = "rtkrcv_stat";
      msg.data.assign(buf.begin(), buf.begin() + static_cast<long>(got));
      stat_pub_->publish(msg);

      total += got;
      if (got < want) break;   // 短读:大概率已经到文件末尾
    }
    return total;
  }

  rclcpp::Node* node_;

  RtkrcvConfParams conf_;
  std::string run_dir_;
  std::string conf_path_;
  std::string binary_;
  std::vector<std::string> extra_args_;
  double restart_delay_s_ = 5.0, crash_loop_life_s_ = 30.0, max_restart_delay_s_ = 60.0;
  double sol_initial_backoff_s_ = 1.0, sol_max_backoff_s_ = 30.0, sol_idle_timeout_s_ = 30.0;
  std::string corr_topic_, obs_topic_, frame_id_;
  gnss_core::PosReadOptions pos_opts_;
  double stat_poll_interval_s_ = 0.2;

  // 声明顺序仅影响“构造中途抛异常”这一种边缘路径下的自动析构顺序;正常
  // 路径下的停止顺序由上面手写的析构函数体决定,不依赖这里的排列。
  LocalReserver corr_reserver_;
  LocalReserver obs_reserver_;
  std::unique_ptr<TcpStream> sol_stream_;
  std::unique_ptr<ProcessSupervisor> supervisor_;

  LineSplitter sol_splitter_;
  rclcpp::Publisher<gnss_msgs::msg::RtkFix>::SharedPtr rtk_fix_pub_;
  rclcpp::Publisher<gnss_msgs::msg::RawStream>::SharedPtr stat_pub_;
  rclcpp::Subscription<gnss_msgs::msg::RawStream>::SharedPtr corr_sub_, obs_sub_;

  std::thread pid_log_thread_;
  std::atomic<bool> pid_log_running_{false};
  std::mutex pid_log_mutex_;
  std::condition_variable pid_log_cv_;

  std::thread stat_thread_;
  std::atomic<bool> stat_running_{false};
  std::mutex stat_mutex_;
  std::condition_variable stat_cv_;
  std::string stat_current_file_;
  uint64_t stat_current_offset_ = 0;
};

}  // namespace gnss_bringup

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rtkrcv_node");
  std::unique_ptr<gnss_bringup::RtkrcvSupervisorNode> sup;

  try {
    sup = std::make_unique<gnss_bringup::RtkrcvSupervisorNode>(node.get());
  } catch (const std::exception& e) {
    // 配置错误(端口越界、conf 参数非法、run_dir 建不出来……)必须落成一行
    // RCLCPP_ERROR + 非零退出,不能让异常捅到 main() 外面变成
    // std::terminate()——现场操作人员需要看到的是清楚的报错,不是 core dump。
    RCLCPP_ERROR(node->get_logger(), "启动失败,配置有误: %s", e.what());
    sup.reset();
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);

  // 显式按类析构函数注释里那套顺序停止,必须发生在 node 本身析构之前——
  // sup 析构时 node 仍然完整存活,回调里 node_->now() / publisher::publish()
  // 都还是安全的。
  sup.reset();
  rclcpp::shutdown();
  return 0;
}
