#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gnss_bringup {

// 把一条字节流在本机 TCP 上再服务出去,供 rtkrcv(inpstr*-type=tcpcli)连入。
// 写不动的客户端会被强制断开,避免内核发送缓冲无界增长拖垮节点。
class LocalReserver {
public:
  ~LocalReserver();
  // port=0 时由内核选端口,经 bound_port() 取回
  bool start(int port, const std::string& host = "127.0.0.1");
  void stop();
  int bound_port() const { return bound_port_.load(); }
  size_t client_count() const;
  void broadcast(const uint8_t* data, size_t len);

private:
  void accept_loop();

  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> bound_port_{-1};
  int listen_fd_ = -1, wake_fd_ = -1, wake_wr_ = -1;
  mutable std::mutex m_;
  std::vector<int> clients_;
};

}  // namespace gnss_bringup
