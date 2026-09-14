#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "gnss_bringup/executable_lookup.hpp"
#include "gnss_msgs/msg/raw_stream.hpp"
#include "gnss_msgs/msg/rtk_fix.hpp"
#include "node_process_harness.hpp"

using namespace gnss_bringup_test;
using gnss_msgs::msg::RawStream;
using gnss_msgs::msg::RtkFix;

namespace {
// 按 RTCM3 帧切分,每遇到一条 1004(每个历元最后一条)切一段
std::vector<std::vector<uint8_t>> split_epochs_after_1004(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  const std::vector<uint8_t> d((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  std::vector<std::vector<uint8_t>> out;
  std::vector<uint8_t> cur;
  std::size_t i = 0;
  while (i + 6 <= d.size()) {
    if (d[i] != 0xD3) { ++i; continue; }
    const std::size_t len = (static_cast<std::size_t>(d[i + 1] & 0x03) << 8) | d[i + 2];
    if (i + 6 + len > d.size()) break;
    const int type = (d[i + 3] << 4) | (d[i + 4] >> 4);
    cur.insert(cur.end(), d.begin() + static_cast<long>(i), d.begin() + static_cast<long>(i + 6 + len));
    i += 6 + len;
    if (type == 1004) {
      out.push_back(std::move(cur));
      cur.clear();
    }
  }
  return out;
}

std::string find_faketime_lib() {
  for (const char* p : {"/usr/lib/x86_64-linux-gnu/faketime/libfaketimeMT.so.1",
                        "/usr/lib/aarch64-linux-gnu/faketime/libfaketimeMT.so.1"}) {
    if (::access(p, R_OK) == 0) return p;
  }
  return {};
}

std::string run_capture(const std::string& cmd) {
  std::string out;
  if (FILE* p = ::popen(cmd.c_str(), "r")) {
    char buf[256];
    while (std::fgets(buf, sizeof(buf), p)) out += buf;
    ::pclose(p);
  }
  return out;
}
}  // namespace

TEST(RtkrcvRealBinary, ReplayedTwoStationRtcmYieldsRtkFixesThroughTheNode) {
  const std::string rtkrcv = gnss_bringup::resolve_executable("rtkrcv", std::getenv("PATH"));
  if (rtkrcv.empty()) GTEST_SKIP() << "PATH 里没有 rtkrcv(RTKLIB-EX 2.5.1 未安装),跳过真实二进制回归";
  const std::string faketime = find_faketime_lib();
  if (faketime.empty()) GTEST_SKIP() << "未安装 libfaketime(sudo apt install faketime),跳过";
  const std::string version = run_capture(rtkrcv + " --version 2>&1");
  ASSERT_NE(version.find("EX 2.5"), std::string::npos)
      << "PATH 里的 rtkrcv 不是 RTKLIB-EX 2.5.x,rtkrcv_node 在这台机器上跑不起来:\n" << version;

  const auto rover = split_epochs_after_1004(std::string(GNSS_TEST_DATA_DIR) + "/rtcm_20050402_0759_rover.rtcm3");
  const auto base = split_epochs_after_1004(std::string(GNSS_TEST_DATA_DIR) + "/rtcm_20050402_3040_base.rtcm3");
  ASSERT_EQ(rover.size(), 120u);
  ASSERT_EQ(base.size(), 120u);

  const std::string dir = make_temp_dir("rtkrcv_real_");
  ASSERT_FALSE(dir.empty());
  std::filesystem::create_directories(dir + "/run");
  const std::string domain = std::to_string(40 + ::getpid() % 50);
  ::setenv("ROS_DOMAIN_ID", domain.c_str(), 1);            // 本测试进程自己的 rclcpp 也在这个域
  ::setenv("ROS_LOG_DIR", (dir + "/roslog").c_str(), 1);

  NodeProcess node(RTKRCV_NODE_PATH,
                   {"--ros-args",
                    "-p", std::string("binary:=") + RTKRCV_FAKETIME_WRAPPER,
                    "-p", "run_dir:=" + dir + "/run",
                    "-p", "sol_port:=" + std::to_string(pick_free_port()),
                    "-p", "corr_port:=" + std::to_string(pick_free_port()),
                    "-p", "obs_port:=" + std::to_string(pick_free_port()),
                    "-p", "sol_idle_timeout_s:=5.0"},
                   dir + "/node.log",
                   {{"ROS_DOMAIN_ID", domain}, {"ROS_LOG_DIR", dir + "/roslog"}, {"GNSS_TEST_FAKETIME_LIB", faketime}});

  rclcpp::init(0, nullptr);
  auto n = std::make_shared<rclcpp::Node>("rtkrcv_real_binary_test");
  std::mutex mu;
  std::vector<RtkFix> fixes;
  std::atomic<std::size_t> stat_bytes{0};
  const auto qos = rclcpp::QoS(1000).reliable();
  auto fix_sub = n->create_subscription<RtkFix>("/rtkrcv_node/rtk_fix", qos, [&](RtkFix::SharedPtr m) {
    std::lock_guard<std::mutex> lk(mu);
    fixes.push_back(*m);
  });
  auto stat_sub = n->create_subscription<RawStream>("/rtkrcv_node/stat", qos,
                                                    [&](RawStream::SharedPtr m) { stat_bytes += m->data.size(); });
  auto pub_obs = n->create_publisher<RawStream>("/gnss/raw_obs", qos);
  auto pub_corr = n->create_publisher<RawStream>("/gnss/rtcm_corrections", qos);
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(n);
  const auto spin_for = [&](double s) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(s);
    while (std::chrono::steady_clock::now() < end) ex.spin_some(std::chrono::milliseconds(20));
  };

  const bool ready = wait_until(
      [&] {
        ex.spin_some(std::chrono::milliseconds(20));
        return pub_obs->get_subscription_count() > 0 && pub_corr->get_subscription_count() > 0 &&
               node.log().find("sol stream: connected") != std::string::npos;
      },
      30.0);
  if (!ready) {
    rclcpp::shutdown();
    FAIL() << "节点没有就绪\n" << node.log();
  }

  for (std::size_t k = 0; k < rover.size(); ++k) {
    RawStream mb;
    mb.data = base[k];
    pub_corr->publish(mb);
    RawStream mr;
    mr.data = rover[k];
    pub_obs->publish(mr);
    spin_for(0.1);
  }
  spin_for(5.0);
  node.interrupt();
  const int rc = node.wait_exit(20.0);
  spin_for(0.5);
  rclcpp::shutdown();

  EXPECT_EQ(rc, 0) << node.log();
  EXPECT_GT(stat_bytes.load(), 0u) << "rtkrcv 的 $SAT/.stat 输出必须被转发";
  std::lock_guard<std::mutex> lk(mu);
  EXPECT_GE(fixes.size(), 80u)
      << "120 个历元回放下来应有约 100 条解;conf 缺 ant2-postype 时这里是 0\n" << node.log();
  for (const auto& f : fixes) {
    EXPECT_TRUE(f.quality == RtkFix::QUALITY_FLOAT || f.quality == RtkFix::QUALITY_FIXED)
        << "quality=" << static_cast<int>(f.quality);
    EXPECT_NEAR(f.latitude, 35.16087, 1e-3);
    EXPECT_NEAR(f.longitude, 139.61384, 1e-3);
    EXPECT_NEAR(f.altitude, 70.0, 10.0);
    // 2005-04-02 00:00:00–00:59:30 GPST,节点按 leap_seconds=18 换成 UTC unix 秒
    EXPECT_GT(f.gnss_time, 1112399900.0);
    EXPECT_LT(f.gnss_time, 1112403700.0);
  }
  std::filesystem::remove_all(dir);
}
