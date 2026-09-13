// pos_writer_node:接线 Task 2-4 的三个纯组件,把 N 路 gnss_msgs/RtkFix 各自
// 按 1 Hz 抽稀写成 RTKLIB .pos 摘要,按 UTC 天轮转目录(spec §5.3)。
//
// round 2 review 之后,"一路 .pos 输出"的全部状态与决策——PosDecimator +
// PosWriter + 当前路径 + 失败锁存 + 沉默检测——已经搬到
// gnss_bringup::PosSourceWriter(pos_source_writer.hpp)里,不依赖任何 ROS
// 类型,可以脱离节点独立单测(round 2 review 的 Important 4:两个能骗过
// 全部既有测试的变异——把轮转依据换成"现在"、把失败锁存改坏——现在都会被
// pos_source_writer.hpp 自己的单测直接抓到)。这个文件只做两件事:
//   1) 把 RtkFix 转成 PosRecord 喂给 PosSourceWriter::handle();
//   2) 把返回的 PosSourceEvent 翻译成 RCLCPP_* 日志,并跑一个独立于消息
//      到达的沉默检测定时器(Important 3)。
//
// 与 rtcm_bridge_node.cpp 相同的扁平前缀参数风格:root/sources/period_s/
// time_system/leap_seconds/silence_timeout_s 是全局参数,每路的
// "<name>.topic" 是唯一的每路参数。

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <gnss_msgs/msg/rtk_fix.hpp>

#include "gnss_bringup/pos_rotation.hpp"        // detail::is_valid_pos_source(启动时校验 sources)
#include "gnss_bringup/pos_source_writer.hpp"   // PosSourceWriter / PosSourceEvent / PosSourceResult
#include "gnss_bringup/pos_writer_params.hpp"   // parse_pos_time_system / is_sane_leap_seconds / is_sane_period_seconds / describe_stamp_sanity
#include "gnss_bringup/rtcm_bridge_params.hpp"  // find_duplicate_stream_name / is_positive_finite_backoff_seconds
#include "gnss_bringup/rtk_fix_mapping.hpp"     // to_pos_record
#include "gnss_core/pos_io.hpp"                 // PosRecord / PosTimeSystem

namespace {

// round 2 review 的 promoted Minor:RCLCPP_*_THROTTLE 原来用
// node_->get_clock()(可能是 sim time)做节流依据——use_sim_time=true 但
// 没有 /clock 发布者时,这个时钟永远停在 0,节流窗口再也不会"过期",第一次
// 之后的所有同类日志都会被吞掉,而 bag 回放正是这个节点最主要的使用场景。
// 节流只是"别刷屏"这个工程目的,应该用一个不受 sim time 影响、单调前进的
// 时钟,与业务时间(record 的 stamp,决定轮转)完全无关。
double steady_now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

namespace gnss_bringup {

// 一路 = 一个订阅 + 一个 PosSourceWriter(纯逻辑核心)+ 一个沉默检测定时器。
class WrittenSource {
public:
  WrittenSource(rclcpp::Node* node, std::string name, const std::string& root, double period_s,
                gnss_core::PosTimeSystem time_system, int leap_seconds, std::string topic,
                double silence_timeout_s)
      : node_(node),
        name_(std::move(name)),
        topic_(std::move(topic)),
        silence_timeout_s_(silence_timeout_s),
        steady_clock_(RCL_STEADY_TIME),
        core_(root, name_, period_s, time_system, leap_seconds, steady_now_s()) {
    sub_ = node_->create_subscription<gnss_msgs::msg::RtkFix>(
        topic_, rclcpp::QoS(100).reliable(),
        [this](const gnss_msgs::msg::RtkFix::SharedPtr msg) { on_msg(*msg); });

    // Important 3:独立于消息到达的沉默检测——一个订阅错话题/QoS 不匹配/
    // 驱动没启动的源,永远不会触发 on_msg,唯一能发现它的办法是一个
    // 按真实时间(wall clock,不是 sim time)跑的独立定时器。检查间隔固定
    // 取 1 秒:远小于任何合理的 silence_timeout_s,又不会造成明显开销。
    silence_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0),
                                               [this] { check_silence(); });

    RCLCPP_INFO(node_->get_logger(),
                "%s: %s -> %s/<YYYYMMDD>/%s.pos(QoS=reliable,"
                "silence_timeout_s=%.1f——%.1f 秒没有任何成功写入会报警;"
                "注意如果对端发布者是 best_effort,reliable 订阅根本收不到它,"
                "现象和\"驱动没启动\"一样是持续沉默)",
                name_.c_str(), topic_.c_str(), root.c_str(), name_.c_str(), silence_timeout_s_,
                silence_timeout_s_);
  }

  WrittenSource(const WrittenSource&) = delete;
  WrittenSource& operator=(const WrittenSource&) = delete;

  // 必须在 node(以及它持有的 clock/logger)析构之前调用——main() 里
  // rclcpp::shutdown() 之前显式 close() 所有路,与 rtcm_bridge_node.cpp 在
  // shutdown 之前 streams.clear() 是同一个道理。
  void close() { core_.close(); }

private:
  void on_msg(const gnss_msgs::msg::RtkFix& msg) {
    // PosSourceWriter/pos_io.hpp/rtk_fix_mapping.hpp 的注释都明确承诺不
    // 抛异常,这里的 try/catch 纯属防御性的——一路的意外不能通过异常捅穿
    // executor,连累其它路的订阅回调(brief 明确要求"其一失败不得影响
    // 其它路")。
    try {
      handle(msg);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "%s: 处理记录时出现意外异常(已跳过这一条): %s",
                   name_.c_str(), e.what());
    }
  }

  void handle(const gnss_msgs::msg::RtkFix& msg) {
    const gnss_core::PosRecord r = to_pos_record(msg);
    const PosSourceResult res = core_.handle(r, steady_now_s());

    switch (res.event) {
      case PosSourceEvent::kWritten:
      case PosSourceEvent::kDroppedRateLimit:
        return;  // 正常路径,不是错误,不打日志

      case PosSourceEvent::kDroppedBadStamp:
        // round 2 review 的 Minor:丢弃原因按实际分类描述(缺失 vs 字段有值
        // 但离谱),并附上累计丢弃计数——此前完全没有任何办法知道丢了多少。
        RCLCPP_WARN_THROTTLE(node_->get_logger(), steady_clock_, 5000,
                              "%s: 记录时间戳不合理(stamp=%.3f,%s),已跳过这条记录,"
                              "不写入、不建目录(累计已丢弃 %zu 条)",
                              name_.c_str(), res.stamp, describe_stamp_sanity(res.stamp_sanity),
                              res.bad_stamp_drop_count);
        return;

      case PosSourceEvent::kDroppedBadPath:
        // 闸门通过之后 pos_path_for 理论上不会再拒绝——但绝不能悄悄丢弃
        // 却不说一声,万一以后两边的判定标准出现分歧,这里要能被现场发现。
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), steady_clock_, 5000,
                               "%s: 无法为 stamp=%.3f 计算 .pos 路径,已丢弃这条记录",
                               name_.c_str(), res.stamp);
        return;

      case PosSourceEvent::kOpenFailed:
        if (res.need_log) {
          RCLCPP_ERROR(node_->get_logger(),
                       "%s: 打开 %s 失败(磁盘满/只读挂载/权限不足?)。在这个问题解决前,"
                       "这一路 .pos 会持续静默重试,不再为每条记录重复报错",
                       name_.c_str(), res.path.c_str());
        }
        return;

      case PosSourceEvent::kWriteFailed:
        if (res.need_log) {
          RCLCPP_ERROR(node_->get_logger(),
                       "%s: 写入 %s 失败(磁盘满?),已关闭该文件,下一条记录会重试打开;"
                       "在这个问题解决前不再为每条记录重复报错",
                       name_.c_str(), res.path.c_str());
        }
        return;
    }
  }

  void check_silence() {
    if (!core_.is_silent(steady_now_s(), silence_timeout_s_)) return;
    RCLCPP_WARN_THROTTLE(node_->get_logger(), steady_clock_, 5000,
                          "%s(topic=%s): 已经超过 %.1f 秒没有写出任何一条记录——"
                          "检查话题名是否配对、驱动是否在跑;订阅固定是 reliable QoS,"
                          "如果对端发布者是 best_effort,同样会表现为持续沉默",
                          name_.c_str(), topic_.c_str(), silence_timeout_s_);
  }

  rclcpp::Node* node_;
  std::string name_;
  std::string topic_;
  double silence_timeout_s_;
  // 专用于节流(WARN/ERROR_THROTTLE)与沉默检测的稳定时钟——刻意不用
  // node_->get_clock():后者在 use_sim_time=true 且没有 /clock 发布者时会
  // 永远停在 0,见文件顶部 steady_now_s() 的注释。
  rclcpp::Clock steady_clock_;
  PosSourceWriter core_;
  rclcpp::Subscription<gnss_msgs::msg::RtkFix>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr silence_timer_;
};

}  // namespace gnss_bringup

int main(int argc, char** argv) {
  std::shared_ptr<rclcpp::Node> node;
  std::vector<std::unique_ptr<gnss_bringup::WrittenSource>> sources;

  // round 2 review 的 Important 1:rclcpp::init()/节点构造本身也可能抛异常
  // (比如 --ros-args 解析失败,或者某些参数覆盖在类型推断上有歧义——
  // 已实测复现:sources 参数被设成 "[]" 空数组字面量时,rclcpp 在初始化
  // 阶段就可能抛出而不是等到 declare_parameter<vector<string>> 那一行)。
  // 原来只有 declare_parameter 那一段包在 try 里,rclcpp::init()/节点构造
  // 留在外面——这类异常会一路捅到 main() 之外变成 std::terminate()/
  // SIGABRT,而"sources: []"(今天没有 GNSS 数据要写)是一个完全合理、
  // 应该被干净拒绝而不是让节点崩溃的配置。现在把两者也纳入同一个 try,
  // node 用一个可能为空的 shared_ptr 承接,catch 里按 node 是否已经构造好
  // 决定用 RCLCPP_ERROR 还是退回 stderr。
  try {
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("pos_writer");

    const auto root = node->declare_parameter<std::string>("root", "");
    if (root.empty()) {
      throw std::invalid_argument("root 不能为空");
    }

    const auto names = node->declare_parameter<std::vector<std::string>>(
        "sources", std::vector<std::string>{});
    if (names.empty()) {
      throw std::invalid_argument("sources 不能为空——至少需要一路");
    }

    // 重名会在下面 declare_parameter("<name>.topic", ...) 时抛
    // ParameterAlreadyDeclaredException,那条报错不会说是哪个名字重复,
    // 这里提前查出来报清楚(与 rtcm_bridge_node.cpp 同一个道理)。
    if (const auto dup = gnss_bringup::find_duplicate_stream_name(names)) {
      throw std::invalid_argument("sources 参数里有重复的名字: " + *dup);
    }
    for (const auto& n : names) {
      // 与 pos_path_for 自身的校验用同一条判据(detail::is_valid_pos_source):
      // source 要能安全地拼进 "<root>/YYYYMMDD/<source>.pos" 这一段文件名——
      // 不能为空,也不能带路径分隔符(否则是路径穿越)。在启动时就挡住,
      // 而不是等第一条消息到达、pos_path_for 返回空串才发现。
      if (!gnss_bringup::detail::is_valid_pos_source(n)) {
        throw std::invalid_argument("sources 里的名字不能安全地用作文件名: \"" + n + "\"");
      }
    }

    const double period_s = node->declare_parameter<double>("period_s", 1.0);
    // round 2 review 的 Minor(promoted):正数且有限还不够——period_s=1e-12
    // 会让 PosDecimator::accept() 内部 floor(stamp/period_s) 产生一个远超
    // long long 表示范围的值,static_cast<long long> 是未定义行为。
    // is_sane_period_seconds 额外要求一个远离溢出边界的下限(1 ms)。
    if (!gnss_bringup::is_sane_period_seconds(period_s)) {
      throw std::invalid_argument(
          "period_s 不合理(收到 " + std::to_string(period_s) + ";必须是 >= " +
          std::to_string(gnss_bringup::kMinSanePeriodSeconds) +
          " 的有限正数——太小会在 PosDecimator 内部整数转换时溢出)");
    }

    const auto time_system_str = node->declare_parameter<std::string>("time_system", "GPST");
    const auto time_system = gnss_bringup::parse_pos_time_system(time_system_str);
    if (!time_system) {
      throw std::invalid_argument(
          "time_system 只接受 \"GPST\" 或 \"UTC\"(收到 \"" + time_system_str + "\")");
    }

    // declare_parameter<int> 实际返回 int64_t(rtcm_bridge_node.cpp/
    // rtkrcv_node.cpp 里同样的注意事项),这里按 int64_t 接住再校验/转换。
    const int64_t leap_seconds_raw = node->declare_parameter<int>("leap_seconds", 18);
    if (leap_seconds_raw < std::numeric_limits<int>::min() ||
        leap_seconds_raw > std::numeric_limits<int>::max() ||
        !gnss_bringup::is_sane_leap_seconds(static_cast<int>(leap_seconds_raw))) {
      throw std::invalid_argument("leap_seconds 不合理(收到 " +
                                   std::to_string(leap_seconds_raw) + ")");
    }
    const int leap_seconds = static_cast<int>(leap_seconds_raw);

    // Important 3:沉默检测超时——一个订阅了错误话题/QoS 不匹配(本节点
    // 固定用 reliable,一个 best_effort-only 的发布者不会被收到)/驱动没
    // 启动的源,不会有任何回调触发,必须有独立于消息到达的检查。默认
    // 10 秒——比 period_s 的常见取值(1 秒)宽松一个数量级,不会对正常的
        // 1 Hz 输出抖动误报。
    const double silence_timeout_s = node->declare_parameter<double>("silence_timeout_s", 10.0);
    if (!gnss_bringup::is_positive_finite_backoff_seconds(silence_timeout_s)) {
      throw std::invalid_argument("silence_timeout_s 必须是正数秒(收到 " +
                                   std::to_string(silence_timeout_s) + ")");
    }

    sources.reserve(names.size());
    for (const auto& n : names) {
      const auto topic = node->declare_parameter<std::string>(n + ".topic", "/gnss/" + n);
      sources.push_back(std::make_unique<gnss_bringup::WrittenSource>(
          node.get(), n, root, period_s, *time_system, leap_seconds, topic, silence_timeout_s));
    }
  } catch (const std::exception& e) {
    // 覆盖:上面主动抛出的 invalid_argument,以及 rclcpp::init()/
    // declare_parameter 对非法 CLI 参数/参数名/重复声明/类型不匹配抛出的
    // 各类异常,统一在这里收口。node 这时可能还没构造成功(比如
    // rclcpp::init() 本身失败),因此不能无条件假设 node->get_logger() 可用。
    if (node) {
      RCLCPP_ERROR(node->get_logger(), "启动失败,配置有误: %s", e.what());
    } else {
      std::fprintf(stderr, "pos_writer: 启动失败,配置有误: %s\n", e.what());
    }
    sources.clear();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  rclcpp::spin(node);

  // 先关闭所有写入器,再让 node(以及它的 clock/logger)析构——与
  // rtcm_bridge_node.cpp 在 rclcpp::shutdown() 之前 streams.clear() 是同一个
  // 道理:PosWriter 析构也会 close(),但那时机是"main 返回之后的自动析构",
  // 依赖它意味着关闭顺序悄悄绑定在成员/局部变量的声明顺序上,不如在这里
  // 显式做一遍来得清楚。
  for (auto& s : sources) s->close();
  sources.clear();
  rclcpp::shutdown();
  return 0;
}
