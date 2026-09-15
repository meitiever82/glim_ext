// gnss_cleanup_node:按保留天数与磁盘水位清理录包与 .pos/诊断目录(spec §3 A6/D4、§5.2,轮 3b)。
// 启动时先跑一轮,之后每 interval_s 一次。定时器与"今天"都用墙钟:删数据按真实日期,
// 不跟随回放时的 sim time。判定逻辑在 gnss_core::retention 与 cleanup_params.hpp。
#include <chrono>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "gnss_bringup/cleanup_params.hpp"

namespace {

void run_pass(rclcpp::Node& node, const gnss_bringup::CleanupParams& p) {
  const double now_s =
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  const int today = gnss_core::utc_yyyymmdd(now_s);
  size_t deleted = 0;
  for (const auto& r : gnss_bringup::run_cleanup_pass(p, today)) {
    if (r.missing) {
      RCLCPP_INFO(node.get_logger(), "%s 根目录不存在,跳过: %s", r.kind.c_str(), r.root.c_str());
      continue;
    }
    if (!r.report.error.empty()) {
      RCLCPP_WARN(node.get_logger(), "清理 %s 根目录没有完成: %s", r.kind.c_str(), r.report.error.c_str());
    }
    for (const auto& name : r.report.deleted) {
      RCLCPP_INFO(node.get_logger(), "已删除 %s/%s", r.root.c_str(), name.c_str());
    }
    deleted += r.report.deleted.size();
    if (!r.used_pct_after) {
      RCLCPP_WARN(node.get_logger(),
                  "%s 所在磁盘用量查不到,本轮只按保留天数(%d 天)清理,水位 %.1f%% 不起作用",
                  r.root.c_str(), p.retention_days, p.watermark_pct);
    } else if (gnss_bringup::over_watermark(r, p.watermark_pct)) {
      RCLCPP_WARN(node.get_logger(),
                  "%s 所在磁盘清理后仍占用 %.1f%%(水位 %.1f%%)——今天的数据与最新一项不删,需要人工处理",
                  r.root.c_str(), *r.used_pct_after, p.watermark_pct);
    }
  }
  RCLCPP_INFO(node.get_logger(), "清理完成一轮:删除 %zu 项", deleted);
}

}  // namespace

int main(int argc, char** argv) {
  std::shared_ptr<rclcpp::Node> node;
  gnss_bringup::CleanupParams p;
  rclcpp::TimerBase::SharedPtr timer;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  bool rclcpp_initialized = false;
  try {
    rclcpp::init(argc, argv);
    rclcpp_initialized = true;
    // executor 紧跟 init 建好,不用 rclcpp::spin(node):后者在里面临时构造 executor,
    // 下面第一轮清理期间收到停止信号时,executor 的 guard condition 会因 context 失效
    // 而抛,那一行在 try 外面就是 SIGABRT(测试实测打中过)。
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    node = std::make_shared<rclcpp::Node>("gnss_cleanup");
    p.bag_root = node->declare_parameter<std::string>("bag_root", "");
    p.pos_root = node->declare_parameter<std::string>("pos_root", "");
    const int64_t days = node->declare_parameter<int>("retention_days", p.retention_days);
    if (days < INT_MIN || days > INT_MAX) throw std::invalid_argument("retention_days 超出范围");
    p.retention_days = static_cast<int>(days);
    p.watermark_pct = node->declare_parameter<double>("watermark_pct", p.watermark_pct);
    p.interval_s = node->declare_parameter<double>("interval_s", p.interval_s);
    gnss_bringup::validate_cleanup_params(p);
    // 定时器也在 try 里建:启动途中收到停止信号时 create_wall_timer 会因 context 失效而抛,
    // 放在 try 外面就是 std::terminate()/SIGABRT。它建好之后节点才算起来,"已启动"
    // 与第一轮清理都排在它后面。
    timer = node->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(p.interval_s * 1000.0)),
                                    [&node, &p] { run_pass(*node, p); });
    executor->add_node(node);
  } catch (const std::exception& e) {
    // 启动途中收到 SIGINT/SIGTERM:rclcpp 的信号处理器先让 context 失效,节点构造
    // (参数服务)或 create_wall_timer 随即抛 "context is invalid"。这是正常的停机请求,
    // 不是配置错误,退出 0。rclcpp::init 本身抛出时 rclcpp::ok() 同样为假,先确认 init 成功过。
    if (rclcpp_initialized && !rclcpp::ok()) {
      if (node) {
        RCLCPP_INFO(node->get_logger(), "启动过程中收到停止信号,放弃启动: %s", e.what());
      } else {
        std::fprintf(stderr, "gnss_cleanup: 启动过程中收到停止信号,放弃启动: %s\n", e.what());
      }
      return 0;
    }
    if (node) {
      RCLCPP_ERROR(node->get_logger(), "启动失败,配置有误: %s", e.what());
    } else {
      std::fprintf(stderr, "gnss_cleanup: 启动失败,配置有误: %s\n", e.what());
    }
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "gnss_cleanup 已启动: bag_root=%s pos_root=%s 保留 %d 天 水位 %.1f%% 间隔 %.0f s",
              p.bag_root.empty() ? "-" : p.bag_root.c_str(), p.pos_root.empty() ? "-" : p.pos_root.c_str(),
              p.retention_days, p.watermark_pct, p.interval_s);
  run_pass(*node, p);
  executor->spin();   // context 已经失效时立刻返回
  executor->remove_node(node);
  executor.reset();
  rclcpp::shutdown();
  return 0;
}
