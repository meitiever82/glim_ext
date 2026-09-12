// rtkrcv_node:接线 Task 1-6 的五个组件,监管 RTKLIB rtkrcv 子进程,把它的
// llh 解流转成 gnss_msgs/RtkFix 发布出去,并把 $SAT 状态行原样转发供轮 3
// 的 gnss_diag 消费(见 task-7-brief.md)。
//
// 接线顺序(brief §Step 4,顺序本身是要点——rtkrcv 首次连接本机端口如果连不上,
// 会白白吃掉一次退避周期什么都不干):
//   1. 读参数
//   2. LocalReserver 起两个本机服务端口(corr/obs),供 rtkrcv 当 tcpcli 连入
//   3. render_rtkrcv_conf() 写 conf 文件
//   4. ProcessSupervisor 起 rtkrcv
//   5. TcpStream 连 rtkrcv 的解算输出端口,按行切分喂 parse_llh_solution
//   6. 订阅 corrections/raw_obs,broadcast 进两个 LocalReserver
//   7. 轮询 run_dir 下最新的 rtkrcv_*.stat,tail 新增字节,原样发布
//
// 析构顺序是这个顺序的镜像,而且是硬约束:先停 ProcessSupervisor(杀
// rtkrcv,它不会再往 sol TcpStream 写数据),再停 TcpStream(它的线程不会
// 再回调 on_data),最后停两个 LocalReserver——全部发生在 node 和它的
// publisher 被销毁之前。这四个组件都在各自的线程上回调,回调打到一个已经
// 析构的 publisher 上就是一次崩溃(与 rtcm_bridge_node.cpp 在
// rclcpp::shutdown() 之前 streams.clear() 是同一个道理)。

#include <atomic>
#include <chrono>
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
#include "gnss_bringup/rtcm_bridge_params.hpp"   // is_valid_port:与 rtcm_bridge_node 共用的端口校验
#include "gnss_bringup/rtk_fix_mapping.hpp"       // to_rtk_fix + LineSplitter
#include "gnss_bringup/rtkrcv_conf.hpp"
#include "gnss_bringup/tcp_stream.hpp"
#include "gnss_core/rtkstat.hpp"                  // parse_llh_solution

namespace {

namespace fs = std::filesystem;

// run_dir 下最新的 rtkrcv_*.stat:按文件名排序取最大即可——RTKLIB 用
// "<前缀>_yyyymmddhhmmss.stat" 命名,时间戳在文件名里,字典序等价于时间序。
fs::path find_latest_stat_file(const fs::path& run_dir) {
  fs::path latest;
  std::error_code ec;
  if (!fs::exists(run_dir, ec)) return latest;
  for (const auto& entry : fs::directory_iterator(run_dir, ec)) {
    if (ec) break;
    if (!entry.is_regular_file()) continue;
    const auto& p = entry.path();
    if (p.extension() != ".stat") continue;
    if (latest.empty() || p.filename().string() > latest.filename().string()) {
      latest = p;
    }
  }
  return latest;
}

}  // namespace

namespace gnss_bringup {

// 把 Task 1-6 的组件接成一个可运行的 rtkrcv 监管节点。
class RtkrcvSupervisorNode {
public:
  explicit RtkrcvSupervisorNode(rclcpp::Node* node) : node_(node) {
    read_params();
    start_local_reservers();
    write_conf();
    start_supervisor();
    log_initial_pid();
    connect_solution_stream();
    subscribe_uplink_streams();
    start_stat_tailer();
  }

  // 显式按 brief 规定的镜像顺序停止,而不是依赖成员声明顺序在析构时自动
  // 倒序析构——那样一来这里的顺序意图会散落在类定义的字段排列里,后来者
  // 改一下字段顺序就能悄悄改变停止顺序而不自知。
  ~RtkrcvSupervisorNode() {
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
    conf_.obs_port = static_cast<int>(declare_port("obs_port", conf_.obs_port));
    conf_.corr_port = static_cast<int>(declare_port("corr_port", conf_.corr_port));
    conf_.sol_port = static_cast<int>(declare_port("sol_port", conf_.sol_port));
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

    restart_delay_s_ = node_->declare_parameter<double>("restart_delay_s", 5.0);
    crash_loop_life_s_ = node_->declare_parameter<double>("crash_loop_life_s", 30.0);
    max_restart_delay_s_ = node_->declare_parameter<double>("max_restart_delay_s", 60.0);

    sol_initial_backoff_s_ = node_->declare_parameter<double>("sol_initial_backoff_s", 1.0);
    sol_max_backoff_s_ = node_->declare_parameter<double>("sol_max_backoff_s", 30.0);
    sol_idle_timeout_s_ = node_->declare_parameter<double>("sol_idle_timeout_s", 30.0);

    corr_topic_ = node_->declare_parameter<std::string>("corrections_topic", "/gnss/rtcm_corrections");
    obs_topic_ = node_->declare_parameter<std::string>("raw_obs_topic", "/gnss/raw_obs");
    frame_id_ = node_->declare_parameter<std::string>("frame_id", "gnss");

    pos_opts_.leap_seconds = static_cast<int>(node_->declare_parameter<int>("leap_seconds", 18));
    // render_rtkrcv_conf() 固定写 out-timesys=gpst(见 rtkrcv_conf.cpp),
    // 与 parse_llh_solution 这边的默认时间系统必须一致,不做成参数以免
    // 两边失配。
    pos_opts_.default_time_system = gnss_core::PosTimeSystem::GPST;

    stat_poll_interval_s_ = node_->declare_parameter<double>("stat_poll_interval_s", 0.2);

    if (run_dir_.empty()) {
      throw std::invalid_argument("run_dir 不能为空");
    }
  }

  int64_t declare_port(const std::string& name, int default_value) {
    // declare_parameter<int> 实际返回 int64_t(与 rtcm_bridge_node.cpp 同样的
    // 注意事项),这里统一按 int64_t 接住再校验范围。
    const int64_t v = node_->declare_parameter<int>(name, default_value);
    if (!gnss_bringup::is_valid_port(v)) {
      throw std::invalid_argument(name + ": 非法端口号 " + std::to_string(v) +
                                   "(合法范围 0-65535)");
    }
    return v;
  }

  // ---------- Step 2: 先起本机服务,rtkrcv 才有地方连 ----------
  void start_local_reservers() {
    if (!corr_reserver_.start(conf_.corr_port)) {
      throw std::runtime_error("无法监听 corr_port=" + std::to_string(conf_.corr_port));
    }
    if (!obs_reserver_.start(conf_.obs_port)) {
      throw std::runtime_error("无法监听 obs_port=" + std::to_string(conf_.obs_port));
    }
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

  // 有界轮询,只为了在日志里打一行"spawned pid",不是业务逻辑依赖项——
  // 找不到也不影响后续接线(supervisor 自己会一直重试)。
  void log_initial_pid() {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      const int pid = supervisor_->last_child_pid();
      if (pid > 0) {
        RCLCPP_INFO(node_->get_logger(), "rtkrcv spawned pid %d", pid);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    RCLCPP_WARN(node_->get_logger(),
                "3s 内未观测到 rtkrcv 子进程 pid,继续接线(supervisor 会持续重试)");
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
        [this](bool connected, const std::string& detail) {
          RCLCPP_INFO(node_->get_logger(), "sol stream: %s %s",
                      connected ? "connected" : "disconnected", detail.c_str());
        });
    sol_stream_->start();
  }

  // TcpStream 给的是任意切分的字节块,不是行——用 LineSplitter 攒缓冲、
  // 按 '\n' 切分,半行留到下一次回调(brief 点名的坑:一行被切在两个块
  // 中间,如果直接当成两条破损的行解析,就是静默丢数据)。
  void on_solution_bytes(const uint8_t* d, size_t n) {
    for (const auto& line : sol_splitter_.feed(d, n)) {
      gnss_core::PosRecord rec;
      if (!gnss_core::parse_llh_solution(line, rec, pos_opts_)) continue;

      auto msg = to_rtk_fix(rec);
      msg.header.stamp = node_->now();     // 接收时刻;msg.gnss_time 是解算历元,
                                            // 两者之差是轮 3 的 stamp_skew 诊断输入
      msg.header.frame_id = frame_id_;
      rtk_fix_pub_->publish(msg);
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
    fs::path current_file;
    std::uintmax_t offset = 0;

    while (stat_running_.load()) {
      const fs::path latest = find_latest_stat_file(run_dir_);
      if (!latest.empty()) {
        if (latest != current_file) {
          // rtkrcv (再)起来后新开的 .stat 文件:从头开始 tail。
          current_file = latest;
          offset = 0;
        }
        tail_new_bytes(current_file, offset);
      }

      std::unique_lock<std::mutex> lk(stat_mutex_);
      stat_cv_.wait_for(lk, std::chrono::duration<double>(stat_poll_interval_s_),
                         [this] { return !stat_running_.load(); });
    }
  }

  void tail_new_bytes(const fs::path& file, std::uintmax_t& offset) {
    std::error_code ec;
    const auto size = fs::file_size(file, ec);
    if (ec || size <= offset) return;   // 没有新增内容(或文件被截断/轮转,offset 已在切换文件时归零)

    std::ifstream ifs(file, std::ios::binary);
    if (!ifs) return;
    ifs.seekg(static_cast<std::streamoff>(offset));

    std::vector<uint8_t> buf(static_cast<size_t>(size - offset));
    ifs.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
    const auto got = static_cast<std::uintmax_t>(ifs.gcount());
    if (got == 0) return;
    buf.resize(got);
    offset += got;

    gnss_msgs::msg::RawStream msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "rtkrcv_stat";
    msg.data = std::move(buf);
    stat_pub_->publish(msg);
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

  std::thread stat_thread_;
  std::atomic<bool> stat_running_{false};
  std::mutex stat_mutex_;
  std::condition_variable stat_cv_;
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

  // 显式按 supervisor -> sol_stream -> reservers 的镜像顺序停止(见类析构
  // 函数注释),必须发生在 node 本身析构之前——sup 析构时 node 仍然完整
  // 存活,回调里 node_->now() / publisher::publish() 都还是安全的。
  sup.reset();
  rclcpp::shutdown();
  return 0;
}
