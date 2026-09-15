#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <gnss_msgs/msg/raw_stream.hpp>
#include <rclcpp/rclcpp.hpp>

#include "node_process_harness.hpp"
using namespace gnss_bringup_test;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {
std::string domain_id() { return std::to_string(20 + ::getpid() % 10); }

struct Restart {
  double delay_s = 0.1, crash_loop_life_s = 5.0, max_delay_s = 0.4;
};

std::vector<std::string> node_args(const std::string& dir, int sol_port, const std::string& mode, Restart r) {
  return {"--ros-args",
          "-p", std::string("binary:=") + FAKE_RTKRCV_PATH,
          "-p", "run_dir:=" + dir + "/run",
          "-p", "sol_port:=" + std::to_string(sol_port),
          "-p", "corr_port:=" + std::to_string(pick_free_port()),
          "-p", "obs_port:=" + std::to_string(pick_free_port()),
          "-p", "restart_delay_s:=" + std::to_string(r.delay_s),
          "-p", "crash_loop_life_s:=" + std::to_string(r.crash_loop_life_s),
          "-p", "max_restart_delay_s:=" + std::to_string(r.max_delay_s),
          "-p", "sol_initial_backoff_s:=0.2",
          "-p", "sol_max_backoff_s:=0.5",
          "-p", "health_no_solution_warn_s:=2.0",
          "-p", "args:=['" + mode + "']"};
}

// 起节点子进程 + 本进程 rclcpp,收集 /rtkrcv_node/diagnostics
class HealthFixture {
public:
  HealthFixture(const std::string& dir, int sol_port, const std::string& mode, Restart r)
      : node_(RTKRCV_NODE_PATH, node_args(dir, sol_port, mode, r), dir + "/node.log",
              {{"ROS_DOMAIN_ID", domain_id()}, {"ROS_LOG_DIR", dir + "/roslog"}}) {
    ::setenv("ROS_DOMAIN_ID", domain_id().c_str(), 1);
    ::setenv("ROS_LOG_DIR", (dir + "/roslog").c_str(), 1);
    rclcpp::init(0, nullptr);
    n_ = std::make_shared<rclcpp::Node>("rtkrcv_health_test_driver");
    sub_ = n_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/rtkrcv_node/diagnostics", 10, [this](const diagnostic_msgs::msg::DiagnosticArray& a) {
          for (const auto& s : a.status) {
            if (s.name == "rtkrcv_node") last_ = s;
          }
        });
    corr_ = n_->create_publisher<gnss_msgs::msg::RawStream>("/gnss/rtcm_corrections", rclcpp::QoS(100).reliable());
    // executor 要在 rclcpp::init 之后构造(它的 guard condition 绑定默认 context)
    ex_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    ex_->add_node(n_);
  }
  ~HealthFixture() {
    ex_->remove_node(n_);
    ex_.reset();
    sub_.reset();
    corr_.reset();
    n_.reset();
    rclcpp::shutdown();
  }

  // 边 spin 边检查;feed_uplink 为真时每 200 ms 发一块差分字节
  bool spin_until(const std::function<bool(const diagnostic_msgs::msg::DiagnosticStatus&)>& pred, double timeout_s,
                  bool feed_uplink = false) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    auto next_feed = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < end) {
      if (feed_uplink && std::chrono::steady_clock::now() >= next_feed) {
        gnss_msgs::msg::RawStream m;
        m.data = {0xD3, 0x00, 0x00};
        corr_->publish(m);
        next_feed += 200ms;
      }
      ex_->spin_some(20ms);
      if (pred(last_)) return true;
    }
    return false;
  }

  NodeProcess& node() { return node_; }
  const diagnostic_msgs::msg::DiagnosticStatus& last() const { return last_; }

private:
  NodeProcess node_;
  std::shared_ptr<rclcpp::Node> n_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  rclcpp::Publisher<gnss_msgs::msg::RawStream>::SharedPtr corr_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> ex_;
  diagnostic_msgs::msg::DiagnosticStatus last_;
};

// 在 sol_port 上当 rtkrcv 的 outstr1(tcpsvr):接受连接后每 200 ms 写一行 llh 解
class FakeSolutionServer {
public:
  explicit FakeSolutionServer(int port) {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    const int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    ok_ = ::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0 && ::listen(fd_, 4) == 0;
    thread_ = std::thread([this] { run(); });
  }
  ~FakeSolutionServer() {
    running_ = false;
    thread_.join();
    ::close(fd_);
  }
  bool ok() const { return ok_; }

private:
  void run() {
    int client = -1;
    while (running_) {
      if (client < 0) {
        pollfd p{fd_, POLLIN, 0};
        if (ok_ && ::poll(&p, 1, 100) > 0) client = ::accept(fd_, nullptr, nullptr);
        continue;
      }
      static const char kLine[] =
          "2026/09/12 10:23:45.000   44.501234560   90.287654320   617.1230   1  20   0.0110   0.0120   0.0330"
          "   0.0000   0.0000   0.0000   0.80   25.0\n";
      if (::send(client, kLine, sizeof(kLine) - 1, MSG_NOSIGNAL) < 0) {
        ::close(client);
        client = -1;
      }
      std::this_thread::sleep_for(200ms);
    }
    if (client >= 0) ::close(client);
  }
  int fd_ = -1;
  bool ok_ = false;
  std::atomic<bool> running_{true};
  std::thread thread_;
};

bool has(const std::string& s, const char* needle) { return s.find(needle) != std::string::npos; }
}  // namespace

TEST(RtkrcvNodeHealthProcess, DeadChildIsReportedAsError) {
  const auto dir = make_temp_dir("rtkrcv_health_dead_");
  ASSERT_FALSE(dir.empty());
  // 控制者裁定:紧跟在目录非空断言之后、先于 fixture/NodeProcess 声明——
  // 析构顺序 LIFO,fixture(内含 NodeProcess)先析构杀掉子进程,dir_guard
  // 再删目录;ASSERT_* 提前 return 的失败路径也一样被覆盖到。
  TempDirGuard dir_guard(dir);
  fs::create_directories(dir + "/run");
  {
    HealthFixture f(dir, pick_free_port(), "die", Restart{30.0, 1.0, 60.0});
    EXPECT_TRUE(f.spin_until([](const auto& s) { return s.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR; }, 20.0))
        << "最后状态: " << f.last().message << "\n" << f.node().log();
    EXPECT_TRUE(has(f.last().message, "未在运行")) << f.last().message;
    f.node().interrupt();
    EXPECT_EQ(f.node().wait_exit(20.0), 0) << f.node().log();
  }
}

TEST(RtkrcvNodeHealthProcess, SilentSolverIsWarnAndTellsWhetherUplinkArrives) {
  const auto dir = make_temp_dir("rtkrcv_health_silent_");
  ASSERT_FALSE(dir.empty());
  TempDirGuard dir_guard(dir);
  fs::create_directories(dir + "/run");
  {
    HealthFixture f(dir, pick_free_port(), "live", Restart{});
    EXPECT_TRUE(f.spin_until([](const auto& s) { return has(s.message, "上行无数据"); }, 20.0))
        << "最后状态: " << f.last().message << "\n" << f.node().log();
    EXPECT_EQ(f.last().level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    EXPECT_TRUE(f.spin_until([](const auto& s) { return has(s.message, "base_pos_type"); }, 15.0, true))
        << "最后状态: " << f.last().message;
    f.node().interrupt();
    EXPECT_EQ(f.node().wait_exit(20.0), 0) << f.node().log();
  }
}

TEST(RtkrcvNodeHealthProcess, SolutionLinesMakeItOk) {
  const auto dir = make_temp_dir("rtkrcv_health_ok_");
  ASSERT_FALSE(dir.empty());
  TempDirGuard dir_guard(dir);
  fs::create_directories(dir + "/run");
  const int sol_port = pick_free_port();
  {
    HealthFixture f(dir, sol_port, "live", Restart{});
    // 节点启动时会探测 sol_port 是否被占用(孤儿检测),假解算服务必须在它启动之后再监听
    ASSERT_TRUE(wait_until([&] { return has(f.node().log(), "已启动 rtkrcv 监管"); }, 20.0)) << f.node().log();
    FakeSolutionServer server(sol_port);
    ASSERT_TRUE(server.ok());
    EXPECT_TRUE(f.spin_until([](const auto& s) { return has(s.message, "解算输出正常"); }, 20.0))
        << "最后状态: " << f.last().message << "\n" << f.node().log();
    EXPECT_EQ(f.last().level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    f.node().interrupt();
    EXPECT_EQ(f.node().wait_exit(20.0), 0) << f.node().log();
  }
}
