// ros_stream_adapter 的节点级测试:把适配节点当子进程起,本进程用 rclcpp
// 扮演上游(北云驱动)发布 RtcmData / Oem7RawMsg,验证字节被原样搬进
// RawStream 发到输出话题,且空消息被丢弃。
//
// 与 test_gnss_diag_node_process.cpp 同一套隔离手法:独立 ROS_DOMAIN_ID
// (本机 domain 0 上有陈旧的 FastDDS SHM 段,跨进程发现不可靠——已实测
// 默认域里 ros2 topic info 都看不到适配节点的话题),rclcpp 的 init/shutdown
// 用 RAII 兜底,ASSERT 提前 return 也不会影响下一个用例。
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <gnss_msgs/msg/raw_stream.hpp>
#include <novatel_oem7_msgs/msg/oem7_raw_msg.hpp>
#include <robominer_msgs/msg/rtcm_data.hpp>

#include "node_process_harness.hpp"

#ifndef ROS_STREAM_ADAPTER_PATH
#error "ROS_STREAM_ADAPTER_PATH must be defined by CMake"
#endif

using gnss_bringup_test::NodeProcess;
using gnss_bringup_test::TempDirGuard;
using gnss_bringup_test::make_temp_dir;
using gnss_bringup_test::wait_until;

namespace {

std::string domain_id() {
  return std::to_string((::getpid() % 100) + 20);
}

std::vector<std::pair<std::string, std::string>> isolated_env(const std::string& dir) {
  return {{"ROS_DOMAIN_ID", domain_id()}, {"ROS_LOG_DIR", dir + "/roslog"}};
}

// 本测试进程自己也要起 rclcpp 时调用,必须在 NodeProcess 构造之后、rclcpp::init 之前
void join_isolated_domain(const std::string& dir) {
  ::setenv("ROS_DOMAIN_ID", domain_id().c_str(), 1);
  ::setenv("ROS_LOG_DIR", (dir + "/roslog").c_str(), 1);
}

class RclcppGuard {
 public:
  RclcppGuard() = default;
  ~RclcppGuard() {
    if (rclcpp::ok()) rclcpp::shutdown();
  }
  RclcppGuard(const RclcppGuard&) = delete;
  RclcppGuard& operator=(const RclcppGuard&) = delete;
};

// spin 线程 RAII:任何断言失败提前 return 时也能让线程先退出,
// 避免 joinable std::thread 析构导致 std::terminate 把失败原因盖掉。
class SpinThread {
 public:
  explicit SpinThread(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    thread_ = std::thread([this]() {
      while (spinning_) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  }
  ~SpinThread() {
    spinning_ = false;
    if (thread_.joinable()) thread_.join();
  }
  SpinThread(const SpinThread&) = delete;
  SpinThread& operator=(const SpinThread&) = delete;

 private:
  rclcpp::Node::SharedPtr node_;
  std::atomic<bool> spinning_{true};
  std::thread thread_;
};

// 收集某个 RawStream 话题上收到的全部消息(data 字段),供断言。
class RawStreamCollector {
public:
  RawStreamCollector(rclcpp::Node* node, const std::string& topic) {
    sub_ = node->create_subscription<gnss_msgs::msg::RawStream>(
        topic, rclcpp::QoS(100).reliable(),
        [this](const gnss_msgs::msg::RawStream::SharedPtr m) {
          received_.push_back(m->data);
        });
  }
  std::vector<std::vector<uint8_t>> received_;
  rclcpp::Subscription<gnss_msgs::msg::RawStream>::SharedPtr sub_;
};

TEST(RosStreamAdapterProcess, ForwardsBytesAndSkipsEmpty) {
  const std::string dir = make_temp_dir("ros_stream_adapter_test");
  ASSERT_FALSE(dir.empty());
  TempDirGuard guard(dir);

  // NodeProcess 必须在 rclcpp::init 之前构造(见 node_process_harness.hpp 里
  // PR_SET_PDEATHSIG 的注释)。
  NodeProcess proc(ROS_STREAM_ADAPTER_PATH,
                   {"--ros-args",
                    "-p", "rtcm_corrections.in_topic:=/test/rtcm_in",
                    "-p", "rtcm_corrections.out_topic:=/test/corr_out",
                    "-p", "raw_obs.in_topic:=/test/obs_in",
                    "-p", "raw_obs.out_topic:=/test/obs_out"},
                   dir + "/adapter.log", isolated_env(dir));
  join_isolated_domain(dir);
  rclcpp::init(0, nullptr);
  RclcppGuard rclcpp_guard;

  auto node = std::make_shared<rclcpp::Node>("adapter_test");
  auto rtcm_pub = node->create_publisher<robominer_msgs::msg::RtcmData>(
      "/test/rtcm_in", rclcpp::QoS(10).reliable());
  auto obs_pub = node->create_publisher<novatel_oem7_msgs::msg::Oem7RawMsg>(
      "/test/obs_in", rclcpp::QoS(10).reliable());
  RawStreamCollector corr(node.get(), "/test/corr_out");
  RawStreamCollector obs(node.get(), "/test/obs_out");
  SpinThread spinner(node);

  // 等适配节点把订阅建好(双向都匹配上),再发数据。
  ASSERT_TRUE(wait_until(
      [&]() {
        return rtcm_pub->get_subscription_count() > 0 &&
               obs_pub->get_subscription_count() > 0;
      },
      20.0)) << "适配节点 20 s 内没有订阅输入话题,日志:\n" << proc.log();

  robominer_msgs::msg::RtcmData rtcm;
  rtcm.rtcm_data = {0xD3, 0x00, 0x15, 0x3E, 0xE0};
  rtcm_pub->publish(rtcm);
  novatel_oem7_msgs::msg::Oem7RawMsg oem;
  oem.message_data = {0xAA, 0x44, 0x12, 0x1C, 0x8C, 0x00};
  obs_pub->publish(oem);

  ASSERT_TRUE(wait_until(
      [&]() { return !corr.received_.empty() && !obs.received_.empty(); },
      20.0)) << "20 s 内没有收到适配后的 RawStream,日志:\n" << proc.log();

  EXPECT_EQ(corr.received_.front(), std::vector<uint8_t>({0xD3, 0x00, 0x15, 0x3E, 0xE0}));
  EXPECT_EQ(obs.received_.front(), std::vector<uint8_t>({0xAA, 0x44, 0x12, 0x1C, 0x8C, 0x00}));

  // 空消息必须被丢弃:先发一条空的,再发一条非空的,收到的下一条只能是后者。
  const size_t corr_before = corr.received_.size();
  robominer_msgs::msg::RtcmData empty_msg;  // rtcm_data 为空
  rtcm_pub->publish(empty_msg);
  rtcm.rtcm_data = {0x01};
  rtcm_pub->publish(rtcm);
  ASSERT_TRUE(wait_until(
      [&]() { return corr.received_.size() >= corr_before + 1; }, 10.0));
  EXPECT_EQ(corr.received_.back(), std::vector<uint8_t>({0x01}))
      << "空消息不应产生 RawStream 输出";
  // 再多等一拍,确认空消息没有额外产生一条空 RawStream。
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  EXPECT_EQ(corr.received_.size(), corr_before + 1);
}

TEST(RosStreamAdapterProcess, EmptyTopicParamRefusesToStart) {
  const std::string dir = make_temp_dir("ros_stream_adapter_test");
  ASSERT_FALSE(dir.empty());
  TempDirGuard guard(dir);
  // 配置缺项必须启动即失败(fail-fast,与 gnss_bringup 其它节点一致)。
  NodeProcess proc(ROS_STREAM_ADAPTER_PATH,
                   {"--ros-args", "-p", "rtcm_corrections.in_topic:=\"\""},
                   dir + "/adapter_bad.log", isolated_env(dir));
  const int rc = proc.wait_exit(30.0);
  EXPECT_NE(rc, 0) << "空 in_topic 应该拒绝启动,日志:\n" << proc.log();
}

}  // namespace
