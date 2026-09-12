#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <gnss_msgs/msg/raw_stream.hpp>
#include "gnss_bringup/tcp_stream.hpp"

namespace {

// 一路流 = 一个 TcpStream + 一个 publisher。收到多少发多少,不解析(spec §5.1)。
class BridgedStream {
public:
  BridgedStream(rclcpp::Node* node, const std::string& name)
      : node_(node), name_(name) {
    const std::string p = name + ".";
    const auto host = node->declare_parameter<std::string>(p + "host", "127.0.0.1");
    const auto port = node->declare_parameter<int>(p + "port", 0);
    const auto listen = node->declare_parameter<bool>(p + "listen", false);
    const auto topic = node->declare_parameter<std::string>(p + "topic", "/gnss/" + name);
    const auto frame = node->declare_parameter<std::string>(p + "frame_id", name);

    frame_ = frame;
    pub_ = node->create_publisher<gnss_msgs::msg::RawStream>(topic, rclcpp::QoS(100).reliable());

    gnss_bringup::TcpStreamConfig cfg;
    cfg.host = host;
    cfg.port = port;
    cfg.listen = listen;
    cfg.initial_backoff_s = node->declare_parameter<double>(p + "initial_backoff_s", 1.0);
    cfg.max_backoff_s = node->declare_parameter<double>(p + "max_backoff_s", 30.0);
    cfg.idle_timeout_s = node->declare_parameter<double>(p + "idle_timeout_s", 30.0);

    RCLCPP_INFO(node->get_logger(), "%s: %s %s:%d -> %s", name.c_str(),
                listen ? "listen" : "connect", host.c_str(), port, topic.c_str());

    stream_ = std::make_unique<gnss_bringup::TcpStream>(
        cfg,
        [this](const uint8_t* d, size_t n) { publish(d, n); },
        [this](bool connected, const std::string& detail) {
          RCLCPP_INFO(node_->get_logger(), "%s: %s %s", name_.c_str(),
                      connected ? "connected" : "disconnected", detail.c_str());
        });
    stream_->start();
  }

  ~BridgedStream() { if (stream_) stream_->stop(); }

private:
  void publish(const uint8_t* d, size_t n) {
    gnss_msgs::msg::RawStream msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = frame_;
    msg.data.assign(d, d + n);
    pub_->publish(msg);
    bytes_ += n;
  }

  rclcpp::Node* node_;
  std::string name_, frame_;
  rclcpp::Publisher<gnss_msgs::msg::RawStream>::SharedPtr pub_;
  std::unique_ptr<gnss_bringup::TcpStream> stream_;
  size_t bytes_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rtcm_bridge");
  // 默认两路:平台差分(A1)与板卡原始观测(A2)。streams 参数可增减。
  const auto names = node->declare_parameter<std::vector<std::string>>(
      "streams", std::vector<std::string>{"rtcm_corrections", "raw_obs"});
  std::vector<std::unique_ptr<BridgedStream>> streams;
  streams.reserve(names.size());
  for (const auto& n : names) streams.push_back(std::make_unique<BridgedStream>(node.get(), n));

  rclcpp::spin(node);
  streams.clear();          // 先停 TcpStream 线程,再让 node 析构
  rclcpp::shutdown();
  return 0;
}
