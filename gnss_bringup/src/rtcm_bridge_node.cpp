#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <gnss_msgs/msg/raw_stream.hpp>
#include "gnss_bringup/rtcm_bridge_params.hpp"
#include "gnss_bringup/tcp_stream.hpp"

namespace {

// 一路流 = 一个 TcpStream + 一个 publisher。收到多少发多少,不解析(spec §5.1)。
class BridgedStream {
public:
  BridgedStream(rclcpp::Node* node, const std::string& name)
      : node_(node), name_(name) {
    const std::string p = name + ".";
    const auto host = node->declare_parameter<std::string>(p + "host", "127.0.0.1");
    // 注意:declare_parameter<int> 实际返回 int64_t(ParameterValue::get<int>()
    // 对所有非 bool 整型都返回 const int64_t&),所以这里必须显式接成 int64_t——
    // 用 auto 会让后面 RCLCPP_INFO 的 "%d" 与实参类型不符,是未定义行为(不只是
    // 编译警告):在 x86-64 上因为小整数恰好落在 64 位可变参数槽的低 32 位而"凑巧"
    // 打印正确,但标准不保证这一点,目标平台 aarch64 上不能依赖这个巧合。
    const int64_t port = node->declare_parameter<int>(p + "port", 0);
    const auto listen = node->declare_parameter<bool>(p + "listen", false);
    const auto topic = node->declare_parameter<std::string>(p + "topic", "/gnss/" + name);
    const auto frame = node->declare_parameter<std::string>(p + "frame_id", name);

    // 端口越界(如 99999)如果放过去,会在 TcpStream 内部
    // htons(static_cast<uint16_t>(port)) 处被静默截断成另一个端口——节点照常
    // 打印"listen ok",实际绑定的却是完全不同的端口,现场表现为"启动正常但
    // 差分永远收不到",且日志里没有任何异常。必须在这里挡住并明确报错。
    if (!gnss_bringup::is_valid_port(port)) {
      RCLCPP_ERROR(node->get_logger(),
                   "%s: 非法端口号 %ld(合法范围 0-65535,0 表示监听模式下由内核选择)",
                   name.c_str(), static_cast<long>(port));
      throw std::invalid_argument(name + ": port out of range: " + std::to_string(port));
    }

    frame_ = frame;
    pub_ = node->create_publisher<gnss_msgs::msg::RawStream>(topic, rclcpp::QoS(100).reliable());

    gnss_bringup::TcpStreamConfig cfg;
    cfg.host = host;
    cfg.port = static_cast<int>(port);  // 已校验在 [0, 65535],转换安全
    cfg.listen = listen;
    cfg.initial_backoff_s = declare_positive_seconds(p, "initial_backoff_s", 1.0);
    cfg.max_backoff_s = declare_positive_seconds(p, "max_backoff_s", 30.0);
    // idle_timeout_s<=0 会被 TcpStream::pump() 当成"彻底关闭空闲检测"的哨兵值
    // (timeout_ms=-1,poll() 永久阻塞直到有数据或被 stop() 打断)——这是本节点
    // 断线自愈能力的核心,不允许通过 YAML 里的 0 在现场被悄悄关掉。与
    // rtkrcv_node 对 sol_idle_timeout_s 的处理是同一个决定(见
    // gnss_bringup::is_positive_finite_backoff_seconds 的注释)。
    cfg.idle_timeout_s = declare_positive_seconds(p, "idle_timeout_s", 30.0);

    RCLCPP_INFO(node->get_logger(), "%s: %s %s:%d -> %s", name.c_str(),
                listen ? "listen" : "connect", host.c_str(), static_cast<int>(port), topic.c_str());

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
  // 校验一个"必须是正数秒"的参数(重连退避、空闲超时)。与 is_valid_port 的
  // 端口校验同一个道理:提前挡住、报得清楚,报错里点名是哪条流、哪个字段、
  // 收到了什么值,而不是让它在 TcpStream 内部变成一个隐蔽的钉死行为。
  double declare_positive_seconds(const std::string& prefix, const std::string& field,
                                   double default_value) {
    const double v = node_->declare_parameter<double>(prefix + field, default_value);
    if (!gnss_bringup::is_positive_finite_backoff_seconds(v)) {
      RCLCPP_ERROR(node_->get_logger(), "%s: %s 必须是正数秒(收到 %f)", name_.c_str(),
                   field.c_str(), v);
      throw std::invalid_argument(name_ + "." + field + ": 必须是正数秒(收到 " +
                                   std::to_string(v) + ")");
    }
    return v;
  }

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
  std::vector<std::unique_ptr<BridgedStream>> streams;

  // 配置错误(streams 类型不对、重名、非法流名、非法端口……)必须走"打日志 +
  // 非零退出",不能让异常一路捅到 main() 外面变成 std::terminate() / SIGABRT——
  // 现场操作人员看到的应该是一行 RCLCPP_ERROR,不是一段没有上下文的 core dump。
  try {
    // 默认两路:平台差分(A1)与板卡原始观测(A2)。streams 参数可增减。
    // declare_parameter 本身在类型不对时(例如误传成整数)会抛
    // InvalidParameterTypeException,同样要落进下面的 catch。
    const auto names = node->declare_parameter<std::vector<std::string>>(
        "streams", std::vector<std::string>{"rtcm_corrections", "raw_obs"});

    // 重名会在下面第二次 declare_parameter("<name>.host", ...) 时抛
    // ParameterAlreadyDeclaredException——那条报错不会说是哪个名字重复。
    // 提前查出来,报得清楚。
    if (const auto dup = gnss_bringup::find_duplicate_stream_name(names)) {
      RCLCPP_ERROR(node->get_logger(), "streams 参数里有重复的流名: %s", dup->c_str());
      rclcpp::shutdown();
      return 1;
    }

    streams.reserve(names.size());
    for (const auto& n : names) streams.push_back(std::make_unique<BridgedStream>(node.get(), n));
  } catch (const std::exception& e) {
    // 覆盖:非法流名(含 '/'、以数字开头、空字符串等)触发的
    // InvalidParameterNameException / InvalidTopicNameError,以及上面主动抛出的
    // 端口越界 invalid_argument,都是 std::exception 的子类,统一在这里收口。
    RCLCPP_ERROR(node->get_logger(), "启动失败,配置有误: %s", e.what());
    streams.clear();
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  streams.clear();          // 先停 TcpStream 线程,再让 node 析构
  rclcpp::shutdown();
  return 0;
}
