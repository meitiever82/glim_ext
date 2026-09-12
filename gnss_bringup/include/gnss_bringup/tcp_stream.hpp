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

  TcpStreamConfig cfg_;
  OnData on_data_;
  OnState on_state_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> bound_port_{-1};
  std::atomic<int> wake_fd_{-1};    // stop() 用来打断阻塞中的 recv/accept
};

}  // namespace gnss_bringup
