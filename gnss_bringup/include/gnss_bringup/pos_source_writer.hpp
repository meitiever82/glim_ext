#pragma once
#include <cstddef>
#include <string>
#include "gnss_bringup/pos_rotation.hpp"
#include "gnss_bringup/pos_writer_params.hpp"
#include "gnss_core/pos_io.hpp"

namespace gnss_bringup {

// round 2 review 的 Important 4:pos_writer_node.cpp 里原来的 WrittenSource
// 是一个只在 ROS 节点里、靠冒烟测试才能碰到的类——两个变异(把轮转依据从
// r.stamp 换成"现在"、把失败锁存改坏)都能让 142 个既有测试全绿通过,说明
// 这条package"纯逻辑必须独立可测"的规矩,在这个节点自己的接线逻辑上从
// Task 5 提交起就没有被真正执行。这是这个缺口第三次在评审里被点名。
//
// 把"一路 .pos 输出"的全部状态与决策——PosDecimator + PosWriter + 当前
// 路径 + 失败锁存 + 沉默检测——搬到这里,做成一个不依赖任何 ROS 类型的类:
// 构造/处理都是纯粹的状态机,单测不需要起节点、不需要发布者/订阅者,可以
// 直接断言"轮转依据的是记录自己的 stamp,不是调用时刻"这类此前无法在没有
// 真实 rclcpp::Node 的情况下验证的属性。
//
// 节点(pos_writer_node.cpp)只做两件事:把 RtkFix 转成 PosRecord 喂给
// handle(),把返回的 PosSourceEvent 翻译成 RCLCPP_* 日志。

// 处理一条记录之后发生的事情——节点据此决定打什么级别的日志,这个类本身
// 不知道 RCLCPP_* 宏的存在。
enum class PosSourceEvent {
  kWritten,           // 正常写入一条记录
  kDroppedRateLimit,  // 未到 1 Hz 抽稀边界,正常丢弃(不是错误,不用报)
  kDroppedBadStamp,   // 时间戳不合理(见 stamp_sanity),按 need_log 节流报警
  kDroppedBadPath,    // pos_path_for 返回空串(防御性分支,理论上不会走到)
  kOpenFailed,        // 本次 open() 失败
  kWriteFailed,       // 本次 write() 失败
};

struct PosSourceResult {
  PosSourceEvent event = PosSourceEvent::kDroppedRateLimit;
  // 这一次要不要打日志——kWritten/kDroppedRateLimit 恒为 false;
  // kDroppedBadStamp/kDroppedBadPath 由调用方自行节流(它们本来就是可能
  // 高频出现的场景,不在这里做锁存);kOpenFailed/kWriteFailed 已经在类
  // 内部做过"同一路径只报一次 + 全局最短间隔"的锁存,need_log 为 true 时
  // 才应该真的打印。
  bool need_log = false;
  std::string path;                            // 相关路径(open/write 失败时有意义)
  double stamp = 0.0;                          // 相关记录的 stamp,供日志打印
  StampSanity stamp_sanity = StampSanity::kOk;  // kDroppedBadStamp 时的具体原因
  // 因时间戳不合理而被丢弃的记录累计数——用于 kDroppedBadStamp 事件里
  // 报一个"到目前为止一共丢了多少条"的计数,而不是让操作人员完全无法知道
  // 丢了多少数据(round 2 review 的 Minor)。
  std::size_t bad_stamp_drop_count = 0;
};

// 一路 = 一个 PosDecimator + 一个 PosWriter + 当前路径 + 失败锁存 + 沉默
// 检测。接口只用 gnss_core 的类型和内建类型,不出现任何 ROS/rclcpp 类型,
// 因此可以脱离 ROS 独立构造、独立单测。
class PosSourceWriter {
public:
  PosSourceWriter(std::string root, std::string name, double period_s,
                  gnss_core::PosTimeSystem time_system, int leap_seconds, double wall_now_s)
      : root_(std::move(root)),
        name_(std::move(name)),
        decimator_(period_s),
        writer_(time_system, leap_seconds),
        last_activity_wall_s_(wall_now_s) {}

  PosSourceWriter(const PosSourceWriter&) = delete;
  PosSourceWriter& operator=(const PosSourceWriter&) = delete;

  const std::string& name() const { return name_; }
  const std::string& current_path() const { return current_path_; }

  // wall_now_s:调用方传入的"现在"的墙钟秒数(建议用一个不受 use_sim_time
  // 影响的稳定时钟,比如 std::chrono::steady_clock——这个类本身不持有任何
  // 时钟,是为了在单测里可以完全控制时间前进)。只用于:
  //   1) 失败锁存的"全局最短报告间隔"节流(见 report_failure());
  //   2) 沉默检测(is_silent())的参照时刻。
  // 绝不用于轮转决策——轮转只看 r.stamp,这正是 Important 4 mutation 1
  // (把轮转依据换成"现在")要挡住的那类回归。
  PosSourceResult handle(const gnss_core::PosRecord& r, double wall_now_s) {
    const StampSanity sanity = classify_utc_stamp(r.stamp);
    if (sanity != StampSanity::kOk) {
      ++bad_stamp_drop_count_;
      PosSourceResult res;
      res.event = PosSourceEvent::kDroppedBadStamp;
      res.need_log = true;  // 节流交给调用方(它有 ROS 的 THROTTLE 时钟)
      res.path = current_path_;
      res.stamp = r.stamp;
      res.stamp_sanity = sanity;
      res.bad_stamp_drop_count = bad_stamp_drop_count_;
      return res;
    }

    if (!decimator_.accept(r)) {
      return make_result(PosSourceEvent::kDroppedRateLimit, false, r.stamp);
    }

    // 轮转只依据记录自己的 stamp,不依据 wall_now_s——一个重放的 bag 必须
    // 落进它的数据本身所属的目录,而不是"现在"所属的目录。
    const std::string next_path = pos_path_for(root_, name_, r.stamp);
    if (next_path.empty()) {
      return make_result(PosSourceEvent::kDroppedBadPath, true, r.stamp);
    }

    if (should_rotate(current_path_, next_path)) {
      writer_.close();
      current_path_ = next_path;
      error_logged_for_path_ = false;  // 新路径:上一路径的"只报一次"锁存不该延续到这里
    }

    if (!writer_.is_open() && !writer_.open(current_path_)) {
      const bool log_it = note_failure(wall_now_s);
      return make_result(PosSourceEvent::kOpenFailed, log_it, r.stamp);
    }

    if (!writer_.write(r)) {
      // write() 失败(典型是磁盘写满导致 flush 失败)之后,底层 ofstream
      // 进入失败态但 is_open() 仍然是 true——如果不在这里主动 close(),
      // 下一条记录会因为 current_path_ 没变而跳过 open() 分支,永远对着
      // 一个坏掉的流反复调用 write() 却什么都写不出去。主动关闭后,下一条
      // 记录会重新走 open() 路径,一旦磁盘空间恢复就能自动续上。
      writer_.close();
      const bool log_it = note_failure(wall_now_s);
      return make_result(PosSourceEvent::kWriteFailed, log_it, r.stamp);
    }

    error_logged_for_path_ = false;  // 成功写入:清掉锁存,以后再失败会重新报一次
    last_activity_wall_s_ = wall_now_s;
    return make_result(PosSourceEvent::kWritten, false, r.stamp);
  }

  // 距离上一次成功写入(或者构造时刻,如果从未成功写过)已经过去了多久
  // 超过 timeout_s,就认为这一路"沉默"了——round 2 review 的 Important 3:
  // 一个订阅了错误话题/QoS 不匹配/驱动没启动的源,不会有任何回调触发,
  // 唯一能发现它的办法是一个不依赖消息到达的独立时钟检查。
  bool is_silent(double wall_now_s, double timeout_s) const {
    return (wall_now_s - last_activity_wall_s_) > timeout_s;
  }

  // round 2 review 的另一个 Important:节点侧原来在 check_silence() 里用
  // RCLCPP_WARN_THROTTLE(node_->get_logger(), steady_clock_, 5000, ...) 给
  // 沉默告警节流——那个宏的节流状态是绑定在"调用它的源码行"上的一个进程内
  // 共享 static,不是绑定在对象上的。pos_writer_node.cpp 里每一路
  // WrittenSource::check_silence() 执行的是同一行代码,于是 N 路共享同一个
  // 5 秒节流窗口:reviewer 用三路全部指向死话题、silence_timeout_s=2 复现,
  // 运行 20 秒 → can 报 1 次、gpchc 报 3 次、rtkrcv 全程 0 次——第三路整场
  // 不可见,恰恰是沉默检测想防住的那类故障("一个话题名写错/QoS 不匹配/
  // 驱动没启动的源,唯一能发现它的办法是这个告警")。
  //
  // 把"现在是否应该打印沉默告警"做成每个实例自己的纯决策:先判定是否沉默
  // (复用 is_silent()),再看距离这个实例自己上一次真正打印是否已经过了
  // kMinReportIntervalS(与失败锁存用的是同一个 5 秒窗口、同一个
  // wall_now_s 时钟——不用 node 时钟,理由见 note_failure() 的注释:
  // use_sim_time=true 且没有 /clock 时节点时钟永远停在 0)。
  bool should_warn_silence(double wall_now_s, double timeout_s) {
    if (!is_silent(wall_now_s, timeout_s)) return false;
    const bool cooled_down = !has_logged_silence_ever_ ||
        (wall_now_s - last_silence_log_wall_s_) >= kMinReportIntervalS;
    if (cooled_down) {
      has_logged_silence_ever_ = true;
      last_silence_log_wall_s_ = wall_now_s;
    }
    return cooled_down;
  }

  // 必须在 node(以及它持有的 clock/logger)析构之前调用。
  void close() { writer_.close(); }

private:
  PosSourceResult make_result(PosSourceEvent event, bool need_log, double stamp) const {
    PosSourceResult res;
    res.event = event;
    res.need_log = need_log;
    res.path = current_path_;
    res.stamp = stamp;
    return res;
  }

  // PosWriter::open()/write() 失败(磁盘满、只读挂载、权限不足……)必须是
  // ERROR 级别——现场唯一能看到的信号——但两条独立的规则都要满足:
  //   (a) 同一路径连续 N 次失败,只报一次(is_open()==false 时每条记录都会
  //       重试 open(),不能重试一次刷屏一次);
  //   (b) 路径变化(轮转)会重新武装(a)里的锁存——这是常规的"新的一天,
  //       问题若仍在,值得再报一次"。但如果路径变化发生得很快(比如离线
  //       快速回放跨越了好几个真实日期边界,只读挂载问题始终没解决),
  //       "每次路径变化都报一次"本身就会在几秒真实时间内连续报出好几条
  //       ERROR——round 2 review 的 promoted Minor,实测 3 秒内 6 条。
  //       用一个跟路径无关的、全局的最短报告间隔(wall_now_s 度量的真实
  //       时间)把 (b) 的重新武装也节流住:只要上一次真正打印的间隔不够
  //       kMinReportIntervalS,即使路径刚刚变化、锁存刚刚重置,也不打印。
  // 返回 true 表示这次调用方应该真的打印。
  bool note_failure(double wall_now_s) {
    const bool first_for_this_path = !error_logged_for_path_;
    error_logged_for_path_ = true;
    const bool cooled_down =
        !has_logged_failure_ever_ || (wall_now_s - last_failure_log_wall_s_) >= kMinReportIntervalS;
    const bool should_log = first_for_this_path && cooled_down;
    if (should_log) {
      has_logged_failure_ever_ = true;
      last_failure_log_wall_s_ = wall_now_s;
    }
    return should_log;
  }

  // 与本文件其它地方(节点侧 RCLCPP_WARN_THROTTLE)用的 5 秒节流窗口保持
  // 一致,不是另挑的数字。
  static constexpr double kMinReportIntervalS = 5.0;

  std::string root_;
  std::string name_;
  gnss_core::PosDecimator decimator_;
  gnss_core::PosWriter writer_;
  std::string current_path_;

  bool error_logged_for_path_ = false;
  bool has_logged_failure_ever_ = false;
  double last_failure_log_wall_s_ = 0.0;

  double last_activity_wall_s_ = 0.0;
  std::size_t bad_stamp_drop_count_ = 0;

  // should_warn_silence() 的每实例节流状态——修复的核心就是这两个字段
  // 不再是共享的 static。
  bool has_logged_silence_ever_ = false;
  double last_silence_log_wall_s_ = 0.0;
};

}  // namespace gnss_bringup
