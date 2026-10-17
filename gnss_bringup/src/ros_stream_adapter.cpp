// ros_stream_adapter:把车上已经在 ROS 总线上的两路裸流适配成 gnss_msgs/RawStream。
//
// 背景(geacx 真车部署):gnss_bringup 原设计的输入是裸 TCP 端点,由 rtcm_bridge
// 桥进 ROS。geacx 上两路数据已经在 ROS 话题里,但类型不是 RawStream:
//   - 平台差分:北云驱动内置 NTRIP 客户端收到 RTCM3,发布 /soft_dtu/rtcm_data
//     (robominer_msgs/RtcmData, uint8[] rtcm_data)
//   - 板卡原始观测/星历:北云推送 rangecmpb + 星历二进制日志,驱动按
//     oem_raw_msgs 配置原样转发到 /bynav/oemraw
//     (novatel_oem7_msgs/Oem7RawMsg, uint8[] message_data,完整线格式帧
//     含 0xAA 0x44 0x12 同步头与 CRC32,RTKLIB nov/tersus 解码器可直接解)
//
// 本节点与 rtcm_bridge 同一"哑管道"语义:不解析任何字节,收到多少转多少,
// 只换消息类型。输出话题默认与 rtcm_bridge 相同(/gnss/rtcm_corrections、
// /gnss/raw_obs),因此对下游 rtkrcv_node 零改动。
//
// QoS:两个源发布者都是默认 reliable(北云驱动 rclcpp::QoS(queue_size)、
// rtcm_pub_ QoS(10)),这里订阅也用 reliable;发布侧 QoS(100).reliable()
// 与 rtcm_bridge 一致,rtkrcv_node 的订阅能直接匹配。

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <gnss_msgs/msg/raw_stream.hpp>
#include <novatel_oem7_msgs/msg/oem7_raw_msg.hpp>
#include <robominer_msgs/msg/rtcm_data.hpp>

namespace {

// 一路适配:订阅 SrcT,把字节数组字段原样搬进 RawStream 发出。
// 空消息直接跳过(不发出空的 RawStream——下游对 0 字节没有语义,发了只增噪声)。
template <typename SrcT>
class StreamAdapter {
public:
  // bytes_field 是指向源消息字节数组字段的成员指针,避免为两种消息写两份类。
  StreamAdapter(rclcpp::Node* node, const std::string& name,
                const std::vector<uint8_t> SrcT::*bytes_field,
                const std::string& default_in, const std::string& default_out)
      : node_(node), name_(name), bytes_field_(bytes_field) {
    const std::string p = name + ".";
    in_topic_ = node->declare_parameter<std::string>(p + "in_topic", default_in);
    out_topic_ = node->declare_parameter<std::string>(p + "out_topic", default_out);
    frame_id_ = node->declare_parameter<std::string>(p + "frame_id", name);
    if (in_topic_.empty() || out_topic_.empty()) {
      // 与 gnss_bringup 其它节点一致的 fail-fast:配置缺项必须启动即失败,
      // 不能让"节点起来了但一路没接"变成静默故障。
      throw std::invalid_argument(name_ + ": in_topic/out_topic 不能为空");
    }
    pub_ = node->create_publisher<gnss_msgs::msg::RawStream>(out_topic_,
                                                             rclcpp::QoS(100).reliable());
    sub_ = node->create_subscription<SrcT>(
        in_topic_, rclcpp::QoS(200).reliable(),
        [this](const typename SrcT::SharedPtr msg) { forward(*msg); });
    RCLCPP_INFO(node->get_logger(), "%s: %s -> %s", name_.c_str(),
                in_topic_.c_str(), out_topic_.c_str());
  }

private:
  void forward(const SrcT& msg) {
    const auto& bytes = msg.*bytes_field_;
    if (bytes.empty()) return;
    gnss_msgs::msg::RawStream out;
    out.header.stamp = node_->now();
    out.header.frame_id = frame_id_;
    out.data.assign(bytes.begin(), bytes.end());
    pub_->publish(out);

    // 节流统计:10 s 一条,现场判断"上游在不在发"只看这行,不用 ros2 topic hz。
    ++msg_count_;
    byte_count_ += bytes.size();
    ++total_count_;
    const auto now = std::chrono::steady_clock::now();
    if (now - last_log_ >= std::chrono::seconds(10)) {
      RCLCPP_INFO(node_->get_logger(),
                  "%s: 最近 10 s 转发 %zu 条 / %zu 字节(累计 %zu 条)",
                  name_.c_str(), msg_count_, byte_count_, total_count_);
      msg_count_ = 0;
      byte_count_ = 0;
      last_log_ = now;
    }
  }

  rclcpp::Node* node_;
  std::string name_, in_topic_, out_topic_, frame_id_;
  const std::vector<uint8_t> SrcT::*bytes_field_;
  typename rclcpp::Publisher<gnss_msgs::msg::RawStream>::SharedPtr pub_;
  typename rclcpp::Subscription<SrcT>::SharedPtr sub_;
  size_t msg_count_ = 0, byte_count_ = 0, total_count_ = 0;
  std::chrono::steady_clock::time_point last_log_ = std::chrono::steady_clock::now();
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ros_stream_adapter");
  // 适配器对象必须活到 spin() 结束之后——它们持有 subscriber/publisher,
  // 若在 try 块作用域内析构,节点会"活着但什么都不订不发"(静默哑掉,
  // 图上连话题都看不到)。与 rtcm_bridge_node.cpp 把 streams 放在 try 外、
  // spin 后 clear() 是同一个生命周期考虑(先停回调,再析构 node)。
  std::unique_ptr<StreamAdapter<robominer_msgs::msg::RtcmData>> rtcm;
  std::unique_ptr<StreamAdapter<novatel_oem7_msgs::msg::Oem7RawMsg>> obs;
  try {
    // 两路默认话题名:输入侧是 geacx 车上北云驱动的实际话题,输出侧与
    // rtcm_bridge 的默认输出一致——不接 yaml 单跑节点也能直接工作。
    rtcm = std::make_unique<StreamAdapter<robominer_msgs::msg::RtcmData>>(
        node.get(), "rtcm_corrections",
        &robominer_msgs::msg::RtcmData::rtcm_data,
        "/soft_dtu/rtcm_data", "/gnss/rtcm_corrections");
    obs = std::make_unique<StreamAdapter<novatel_oem7_msgs::msg::Oem7RawMsg>>(
        node.get(), "raw_obs",
        &novatel_oem7_msgs::msg::Oem7RawMsg::message_data,
        "/bynav/oemraw", "/gnss/raw_obs");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "启动失败,配置有误: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rtcm.reset();
  obs.reset();
  rclcpp::shutdown();
  return 0;
}
