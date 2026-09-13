// pos_writer_node:接线 Task 2-4 的三个纯组件,把 N 路 gnss_msgs/RtkFix 各自
// 按 1 Hz 抽稀写成 RTKLIB .pos 摘要,按 UTC 天轮转目录(spec §5.3)。
//
// 每路一个 WrittenSource,持有自己的 PosDecimator + PosWriter + 当前路径,
// 互不影响——一路的磁盘写失败不该连累其它路。收到一条 RtkFix:
//   to_pos_record → 时间戳合理性闸门(见下) → PosDecimator::accept 不过则丢弃
//   → pos_path_for(root, name, r.stamp) → 与当前路径不同则 close() 旧的、
//   open() 新的 → write(r)。
//
// 与 rtcm_bridge_node.cpp 相同的扁平前缀参数风格:root/sources/period_s/
// time_system/leap_seconds 是全局参数,每路的 "<name>.topic" 是唯一的每路
// 参数。

#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <gnss_msgs/msg/rtk_fix.hpp>

#include "gnss_bringup/pos_rotation.hpp"        // pos_path_for / should_rotate / detail::is_valid_pos_source
#include "gnss_bringup/pos_writer_params.hpp"    // parse_pos_time_system / is_sane_leap_seconds / is_sane_utc_stamp
#include "gnss_bringup/rtcm_bridge_params.hpp"   // find_duplicate_stream_name / is_positive_finite_backoff_seconds
#include "gnss_bringup/rtk_fix_mapping.hpp"      // to_pos_record
#include "gnss_core/pos_io.hpp"                  // PosDecimator / PosWriter / PosRecord

namespace gnss_bringup {

// 一路 = 一个订阅 + 一个 PosDecimator + 一个 PosWriter + 当前打开的路径。
class WrittenSource {
public:
  WrittenSource(rclcpp::Node* node, std::string name, std::string root, double period_s,
                gnss_core::PosTimeSystem time_system, int leap_seconds, const std::string& topic)
      : node_(node),
        name_(std::move(name)),
        root_(std::move(root)),
        decimator_(period_s),
        writer_(time_system, leap_seconds) {
    sub_ = node_->create_subscription<gnss_msgs::msg::RtkFix>(
        topic, rclcpp::QoS(100).reliable(),
        [this](const gnss_msgs::msg::RtkFix::SharedPtr msg) { on_msg(*msg); });
    RCLCPP_INFO(node_->get_logger(), "%s: %s -> %s/<YYYYMMDD>/%s.pos", name_.c_str(),
                topic.c_str(), root_.c_str(), name_.c_str());
  }

  WrittenSource(const WrittenSource&) = delete;
  WrittenSource& operator=(const WrittenSource&) = delete;

  // 必须在 node(以及它持有的 clock/logger)析构之前调用——main() 里
  // rclcpp::shutdown() 之前显式 close() 所有路,与 rtcm_bridge_node.cpp 在
  // shutdown 之前 streams.clear() 是同一个道理:写入器不能带着悬空引用活到
  // 进程退出的自动析构里。
  void close() { writer_.close(); }

private:
  void on_msg(const gnss_msgs::msg::RtkFix& msg) {
    // Task 1-6 的组件都不抛异常(pos_io.hpp / rtk_fix_mapping.hpp /
    // pos_rotation.hpp 的注释都明确承诺了这一点),这里的 try/catch 纯属
    // 防御性的——一路的意外不能通过异常捅穿 executor,连累其它路的订阅
    // 回调(brief 明确要求"其一失败不得影响其它路")。
    try {
      handle(msg);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "%s: 处理记录时出现意外异常(已跳过这一条): %s",
                   name_.c_str(), e.what());
    }
  }

  void handle(const gnss_msgs::msg::RtkFix& msg) {
    const gnss_core::PosRecord r = to_pos_record(msg);

    // 板卡两个时间字段(gnss_time/header.stamp)都缺失或异常时,
    // to_pos_record 会原样把 0(或者别的离谱值)传下来——这是上一个任务
    // 明确、有意的决定(见 rtk_fix_mapping.hpp 的注释),不在本任务修改
    // 范围内。但真放任这种记录流到 pos_path_for,会在 <root>/19700101/
    // 下悄悄建出一个目录并往里写数据——这是一个没人会想起来查的地方,
    // 等同于数据静默丢失。本节点在这里加一道闸门:不合理的时间戳直接跳过
    // 并报警(节流,不刷屏),既不写文件也不建目录。这道闸门还顺带挡住了
    // NaN/±inf——PosDecimator::accept 内部 floor(stamp/period) 转 long long
    // 对非有限数是未定义行为,必须在调用它之前挡住。
    if (!is_sane_utc_stamp(r.stamp)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "%s: 记录时间戳不合理(gnss_time 与 header.stamp 都缺失或异常,"
                            "stamp=%.3f),已跳过这条记录,不写入、不建 1970 类目录",
                            name_.c_str(), r.stamp);
      return;
    }

    if (!decimator_.accept(r)) return;  // 未落在 1 Hz 抽稀桶的第一条,丢弃

    const std::string next_path = pos_path_for(root_, name_, r.stamp);
    if (next_path.empty()) {
      // 闸门通过之后 pos_path_for 理论上不会再拒绝(它的拒绝条件是空
      // root/source 在启动时已经校验过,以及非有限/越界 stamp 已经被上面
      // 挡住)——但绝不能悄悄丢弃却不说一声,万一以后两边的判定标准出现
      // 分歧,这里要能被现场发现。
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "%s: 无法为 stamp=%.3f 计算 .pos 路径,已丢弃这条记录",
                             name_.c_str(), r.stamp);
      return;
    }

    if (should_rotate(current_path_, next_path)) {
      writer_.close();
      current_path_ = next_path;
      error_logged_ = false;  // 新路径:上一路径的"只报一次"锁存不该延续到这里
    }

    if (!writer_.is_open() && !writer_.open(current_path_)) {
      report_failure_once("打开");
      return;
    }

    if (!writer_.write(r)) {
      // write() 失败(典型是磁盘写满导致 flush 失败)之后,底层 ofstream
      // 进入失败态但 is_open() 仍然是 true——如果不在这里主动 close(),
      // 下一条记录会因为 current_path_ 没变而跳过 open() 分支,永远对着
      // 一个坏掉的流反复调用 write() 却什么都写不出去。主动关闭后,下一条
      // 记录会重新走 open() 路径,一旦磁盘空间恢复就能自动续上。
      writer_.close();
      report_failure_once("写入");
      return;
    }

    error_logged_ = false;  // 成功写入:清掉锁存,以后再失败会重新报一次
  }

  // PosWriter::open()/write() 失败(磁盘满、只读挂载、权限不足……)必须是
  // ERROR 级别——现场唯一能看到的信号——但 1 Hz 下如果每条记录都报一次就是
  // 刷屏。同一个路径的失败只报一次,路径变化(brief 里的“重置”)或者中途
  // 恢复成功都会清掉这个锁存,下一次失败会重新报一次。
  void report_failure_once(const char* verb) {
    if (error_logged_) return;
    RCLCPP_ERROR(node_->get_logger(),
                 "%s: %s %s 失败(磁盘满/只读挂载/权限不足?)。在这个问题解决前,"
                 "这一路 .pos 会持续静默重试,不再为每条记录重复报错",
                 name_.c_str(), verb, current_path_.c_str());
    error_logged_ = true;
  }

  rclcpp::Node* node_;
  std::string name_;
  std::string root_;
  gnss_core::PosDecimator decimator_;
  gnss_core::PosWriter writer_;
  std::string current_path_;
  bool error_logged_ = false;
  rclcpp::Subscription<gnss_msgs::msg::RtkFix>::SharedPtr sub_;
};

}  // namespace gnss_bringup

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pos_writer");
  std::vector<std::unique_ptr<gnss_bringup::WrittenSource>> sources;

  // 配置错误(root 为空、sources 重名/非法/类型不对、period_s/leap_seconds/
  // time_system 不合法……)必须走"打日志 + 非零退出",不能让异常捅到 main()
  // 外面变成 std::terminate()/SIGABRT——与 rtcm_bridge_node.cpp/
  // rtkrcv_node.cpp 同一个道理。
  try {
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
    if (!gnss_bringup::is_positive_finite_backoff_seconds(period_s)) {
      throw std::invalid_argument("period_s 必须是正数秒(收到 " + std::to_string(period_s) + ")");
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

    sources.reserve(names.size());
    for (const auto& n : names) {
      const auto topic = node->declare_parameter<std::string>(n + ".topic", "/gnss/" + n);
      sources.push_back(std::make_unique<gnss_bringup::WrittenSource>(
          node.get(), n, root, period_s, *time_system, leap_seconds, topic));
    }
  } catch (const std::exception& e) {
    // 覆盖:上面主动抛出的 invalid_argument,以及 declare_parameter 对非法
    // 参数名/重复声明/类型不匹配抛出的各类 rclcpp 异常,统一在这里收口。
    RCLCPP_ERROR(node->get_logger(), "启动失败,配置有误: %s", e.what());
    sources.clear();
    rclcpp::shutdown();
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
