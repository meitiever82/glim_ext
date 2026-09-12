#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

namespace gnss_bringup {

using OnData = std::function<void(const uint8_t* data, size_t len)>;
// connected=true 进入连通态,false 断开;detail 供日志
using OnState = std::function<void(bool connected, const std::string& detail)>;

struct TcpStreamConfig {
  std::string host = "127.0.0.1";
  int port = 0;                     // listen 模式下 0 = 让内核选端口(测试用)
  bool listen = false;              // true: 本机监听等对端连进来;false: 主动连对端
  double initial_backoff_s = 1.0;
  double max_backoff_s = 30.0;
  double idle_timeout_s = 30.0;     // 静默视为断开:链路差时对端常不发 RST 就消失
};

// 一条裸字节流。不做任何解析,收到多少回调多少。
// 断开后按指数退避重连(客户端模式)或继续等待下一个连接(监听模式),永不放弃。
class TcpStream {
public:
  TcpStream(TcpStreamConfig cfg, OnData on_data, OnState on_state = {});
  ~TcpStream();
  TcpStream(const TcpStream&) = delete;
  TcpStream& operator=(const TcpStream&) = delete;

  void start();
  void stop();
  int bound_port() const { return bound_port_.load(); }   // 监听模式实际绑定端口

private:
  void run_client();
  void run_server();
  // 客户端(已连接的 fd)与服务端(已 accept 的 fd)共用的收数循环:
  // poll(fd, wake_fd) 等可读,idle_timeout_s 静默视为断开,recv 到的数据经
  // on_data_ 回调,状态变化(idle timeout / 对端关闭 / poll 或 recv 出错)
  // 经 report 回调上报。返回 true 表示 stop() 打断了本次 pump(调用方应结束
  // 整个 worker,不再重连/重新 accept);返回 false 表示这次连接自身结束了
  // (对端关闭、静默超时或 I/O 错误),调用方按各自策略处理——客户端退避重连,
  // 服务端直接回到 accept 等下一个对端。
  bool pump(int fd, const OnState& report);

  TcpStreamConfig cfg_;
  OnData on_data_;
  OnState on_state_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> bound_port_{-1};
  std::atomic<int> wake_fd_{-1};    // stop() 用来打断阻塞中的 recv/accept
};

}  // namespace gnss_bringup
