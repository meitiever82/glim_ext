#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <gnss_msgs/msg/raw_stream.hpp>
#include <gnss_msgs/msg/rtk_fix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "diag_test_fixtures.hpp"     // gnss_core/test:make_1005_frame
#include "node_process_harness.hpp"
using namespace gnss_bringup_test;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {
std::string domain_id() { return std::to_string(90 + ::getpid() % 10); }

std::vector<std::pair<std::string, std::string>> isolated_env(const std::string& dir) {
  return {{"ROS_DOMAIN_ID", domain_id()}, {"ROS_LOG_DIR", dir + "/roslog"}};
}

// 本测试进程自己也要起 rclcpp 时调用,必须在 NodeProcess 构造之后、rclcpp::init 之前
void join_isolated_domain(const std::string& dir) {
  ::setenv("ROS_DOMAIN_ID", domain_id().c_str(), 1);
  ::setenv("ROS_LOG_DIR", (dir + "/roslog").c_str(), 1);
}

std::vector<std::string> diag_args(const std::string& root, double grace_s, const std::vector<std::string>& extra = {}) {
  std::vector<std::string> a{"--ros-args", "-p", "root:=" + root, "-p", "startup_grace_s:=" + std::to_string(grace_s)};
  for (const auto& e : extra) {
    a.push_back("-p");
    a.push_back(e);
  }
  return a;
}

// 所有日期目录下同名文件的内容拼起来(测试跨 UTC 零点也不漏)
std::string all_day_files(const std::string& root, const std::string& filename) {
  std::string out;
  std::error_code ec;
  for (fs::directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec)) {
    const auto p = it->path() / filename;
    if (fs::is_regular_file(p)) out += read_file(p.string());
  }
  return out;
}

// TempDirGuard 搬进了 node_process_harness.hpp(task-5:多个节点级测试文件
// 复用同一份 RAII 清理逻辑),这里不再本地定义。

// review round 1:同样的道理,两个自己起 rclcpp 的用例原来在函数体末尾显式
// rclcpp::shutdown(),ASSERT_* 提前 return 时跳过,下一个用例开头的 rclcpp::init()
// 就会抛 "context is already initialized"——桩阶段的 RED 就复现过这个级联,
// 那时是刻意的(桩本来就不该让 5 个用例都真正各自失败),但在 GREEN 之后的真实
// 场景里,这个级联会让下一个用例的失败原因文不对题(明明是它自己没问题,却被
// 上一个用例的失败连累报错)。用析构函数兜底。
class RclcppGuard {
 public:
  RclcppGuard() = default;
  ~RclcppGuard() {
    if (rclcpp::ok()) rclcpp::shutdown();
  }
  RclcppGuard(const RclcppGuard&) = delete;
  RclcppGuard& operator=(const RclcppGuard&) = delete;
};
}  // namespace

TEST(GnssDiagNodeProcess, RefusesToStartOnInvalidConfig) {
  const auto dir = make_temp_dir("diag_node_bad_");
  ASSERT_FALSE(dir.empty());
  TempDirGuard dir_guard(dir);
  const auto expect_refused = [&](const std::vector<std::string>& args, const std::string& needle) {
    NodeProcess node(GNSS_DIAG_NODE_PATH, args, dir + "/node.log", isolated_env(dir));
    EXPECT_EQ(node.wait_exit(20.0), 1) << node.log();
    EXPECT_NE(node.log().find("启动失败,配置有误"), std::string::npos) << node.log();
    EXPECT_NE(node.log().find(needle), std::string::npos) << node.log();
  };
  expect_refused({"--ros-args"}, "root");
  expect_refused(diag_args(dir + "/r", 0.0, {"control_points.names:=['K1']"}), "control_points");
  expect_refused(diag_args(dir + "/r", 0.0, {"diagnosis.divergence_sigma_max_m:=0.01"}), "divergence_sigma_max_m");
  expect_refused(diag_args(dir + "/r", 0.0, {"clock_jump_tolerance_s:=0.0"}), "clock_jump_tolerance_s");
}

TEST(GnssDiagNodeProcess, NoInputOpensNoDataAndShutdownClosesIt) {
  const auto dir = make_temp_dir("diag_node_nodata_");
  ASSERT_FALSE(dir.empty());
  TempDirGuard dir_guard(dir);
  const std::string root = dir + "/diag";
  NodeProcess node(GNSS_DIAG_NODE_PATH, diag_args(root, 0.0), dir + "/node.log", isolated_env(dir));
  ASSERT_TRUE(wait_until([&] { return all_day_files(root, "events.log").find(" OPEN warning no_data ") != std::string::npos; }, 20.0))
      << node.log();
  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
  const auto ev = all_day_files(root, "events.log");
  EXPECT_NE(ev.find(" CLOSE warning no_data "), std::string::npos) << ev;
  EXPECT_NE(ev.find("reason=shutdown"), std::string::npos) << "停机必须先写关闭行再关文件\n" << ev;
  EXPECT_EQ(count_occurrences(ev, "% gnss_core events.log"), 1u) << ev;
}

TEST(GnssDiagNodeProcess, StartupGraceDelaysTheFirstJudgement) {
  const auto dir = make_temp_dir("diag_node_grace_");
  ASSERT_FALSE(dir.empty());
  TempDirGuard dir_guard(dir);
  const std::string root = dir + "/diag";
  NodeProcess node(GNSS_DIAG_NODE_PATH, diag_args(root, 5.0), dir + "/node.log", isolated_env(dir));
  ASSERT_TRUE(wait_until([&] { return node.log().find("gnss_diag 已启动") != std::string::npos; }, 20.0)) << node.log();
  std::this_thread::sleep_for(3s);
  EXPECT_EQ(all_day_files(root, "events.log").find(" OPEN "), std::string::npos) << "宽限期内不判定";
  EXPECT_TRUE(wait_until([&] { return all_day_files(root, "events.log").find(" OPEN warning no_data ") != std::string::npos; }, 15.0))
      << node.log();
  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
}

TEST(GnssDiagNodeProcess, WiresInputsToDiagnosticsBaseHistoryAndTheResetService) {
  const auto dir = make_temp_dir("diag_node_wire_");
  ASSERT_FALSE(dir.empty());
  TempDirGuard dir_guard(dir);
  const std::string root = dir + "/diag";
  NodeProcess node(GNSS_DIAG_NODE_PATH, diag_args(root, 0.0, {"diagnosis.base_warmup_s:=1.0"}), dir + "/node.log",
                   isolated_env(dir));
  join_isolated_domain(dir);
  rclcpp::init(0, nullptr);
  RclcppGuard rclcpp_guard;
  {
    auto n = std::make_shared<rclcpp::Node>("diag_node_test_driver");
    const auto qos = rclcpp::QoS(100).reliable();
    auto corr = n->create_publisher<gnss_msgs::msg::RawStream>("/gnss/rtcm_corrections", qos);
    auto sol = n->create_publisher<gnss_msgs::msg::RtkFix>("/rtkrcv_node/rtk_fix", qos);
    auto dev = n->create_publisher<gnss_msgs::msg::RtkFix>("/gnss_cgi610/rtk_fix", qos);
    diagnostic_msgs::msg::DiagnosticStatus last;
    bool fixed_seen = false;
    auto sub = n->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/gnss/diagnostics", 10, [&](const diagnostic_msgs::msg::DiagnosticArray& a) {
          for (const auto& s : a.status) {
            if (s.name != "gnss_diag") continue;
            last = s;
            for (const auto& kv : s.values) {
              if (kv.key == "status_code" && kv.value == "rtk_fixed") fixed_seen = true;
            }
          }
        });
    auto reset = n->create_client<std_srvs::srv::Trigger>("/gnss_diag/reset_base_baseline");
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(n);

    const auto frame = gnss_core::test_fixtures::make_1005_frame(1, -2148744.1, 4426641.2, 4044655.9);
    const auto feed_once = [&] {
      gnss_msgs::msg::RawStream raw;
      raw.data = frame;
      corr->publish(raw);
      gnss_msgs::msg::RtkFix f;
      f.quality = gnss_msgs::msg::RtkFix::QUALITY_FIXED;
      f.latitude = 44.5;
      f.longitude = 90.28;
      f.sigma_enu = {0.012, 0.011, 0.03};
      f.diff_age = 0.8f;
      f.sats_used = 20;
      f.ratio = 25.0f;
      sol->publish(f);
      dev->publish(f);
    };
    const auto spin_feeding_until = [&](const std::function<bool()>& pred, double timeout_s) {
      const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
      auto next_feed = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() < end) {
        if (std::chrono::steady_clock::now() >= next_feed) {
          feed_once();
          next_feed += 200ms;
        }
        ex.spin_some(20ms);
        if (pred()) return true;
      }
      return pred();
    };

    EXPECT_TRUE(spin_feeding_until([&] { return fixed_seen; }, 30.0)) << "最后状态: " << last.message << "\n" << node.log();
    EXPECT_EQ(last.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_TRUE(spin_feeding_until([&] { return fs::exists(root + "/base_baseline"); }, 15.0)) << node.log();
    EXPECT_NE(all_day_files(root, "base.pos").find("-2148744.1000"), std::string::npos) << all_day_files(root, "base.pos");

    ASSERT_TRUE(reset->wait_for_service(10s));
    auto fut = reset->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(ex.spin_until_future_complete(fut, 10s), rclcpp::FutureReturnCode::SUCCESS);
    const auto resp = fut.get();
    EXPECT_TRUE(resp->success) << resp->message;
  }
  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
}

TEST(GnssDiagNodeProcess, BackwardClockJumpClosesOpenEventsAndRebuildsTheEngine) {
  const auto dir = make_temp_dir("diag_node_jump_");
  ASSERT_FALSE(dir.empty());
  TempDirGuard dir_guard(dir);
  const std::string root = dir + "/diag";
  NodeProcess node(GNSS_DIAG_NODE_PATH, diag_args(root, 0.0, {"use_sim_time:=true"}), dir + "/node.log",
                   isolated_env(dir));
  join_isolated_domain(dir);
  rclcpp::init(0, nullptr);
  RclcppGuard rclcpp_guard;
  {
    auto n = std::make_shared<rclcpp::Node>("diag_clock_driver");
    auto clock = n->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    double sim = 1789000000.0;   // 2026-09-10 UTC
    const auto run_clock_until = [&](const std::function<bool()>& pred, double timeout_s) {
      const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
      while (std::chrono::steady_clock::now() < end) {
        rosgraph_msgs::msg::Clock c;
        c.clock.sec = static_cast<int32_t>(std::floor(sim));
        c.clock.nanosec = static_cast<uint32_t>((sim - std::floor(sim)) * 1e9);
        clock->publish(c);
        sim += 0.05;
        std::this_thread::sleep_for(50ms);
        if (pred()) return true;
      }
      return pred();
    };
    ASSERT_TRUE(run_clock_until([&] { return all_day_files(root, "events.log").find(" OPEN warning no_data ") != std::string::npos; }, 30.0))
        << node.log();
    sim -= 100.0;
    EXPECT_TRUE(run_clock_until([&] { return node.log().find("时钟回跳") != std::string::npos; }, 15.0)) << node.log();
    // review round 1(flake 源):engine_time() 里 RCLCPP_WARN("时钟回跳") 在
    // write_transitions()/events_->append() 之前——看到日志的那一刻,关闭行不一定
    // 已经落盘。不能看到日志就立刻断言文件内容,改成 run_clock_until 顺带继续喂
    // /clock、轮询等到写完(而不是用不发布 /clock 的 wait_until——万一子进程当时
    // 还需要更多拍才能把这次 tick 处理完,继续推进 sim time 更稳妥)。
    EXPECT_TRUE(run_clock_until(
        [&] { return all_day_files(root, "events.log").find("reason=shutdown") != std::string::npos; }, 5.0))
        << all_day_files(root, "events.log");
    EXPECT_TRUE(run_clock_until([&] { return count_occurrences(all_day_files(root, "events.log"), " OPEN warning no_data ") >= 2; }, 15.0))
        << "重建后的引擎重新判定\n" << all_day_files(root, "events.log");
  }
  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
}
