#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gnss_bringup {

// final-fix-wave 第 2 项:accept_loop 遇到不可恢复的错误(poll() 本身出错、
// 监听 socket 坏掉)时只会 running_.store(false) 后退出线程,调用方完全
// 没有办法知道——LocalReserver 原来没有任何形式的日志/回调,和 TcpStream
// 的 on_state_ 不是同一个起点。这个类本身不碰 ROS(便于单测/复用),不能
// 直接 RCLCPP_ERROR;detail 交给调用方决定怎么记(rtkrcv_node.cpp 记
// RCLCPP_ERROR)。
using OnFatalError = std::function<void(const std::string& detail)>;

// 把一条字节流在本机 TCP 上再服务出去,供 rtkrcv(inpstr*-type=tcpcli)连入。
// 写不动的客户端会被强制断开,避免内核发送缓冲无界增长拖垮节点。
//
// fd 归属规则(fix round 1:解决 broadcast() 线程和 accept 线程互相
// close() 对方正在用的 fd 这一类 use-after-close/fd 复用错乱):
//   - listen_fd_/wake_fd_/wake_wr_ 只由 stop() 关闭。accept_loop 内部
//     线程自己因为不可恢复的错误退出时,只置 running_=false 就返回,绝不
//     碰这三个 fd——清理统一留给迟早会被调用的 stop()(析构函数保证会
//     调用),仿照 tcp_stream.cpp 里 wake_fd_ 的处理方式。
//   - clients_ 里的客户端 fd 只由 accept_loop 线程 close()。它自己
//     recv()==0/出错检测到的死连接直接关闭+摘除;broadcast()(运行在
//     调用者/ROS 订阅回调线程上)发现写不动的客户端时,只把 fd 从
//     clients_ 挪到 to_drop_(不 close()),再往 wake_wr_ 写一个字节
//     唤醒 accept_loop 去真正关闭它。
class LocalReserver {
public:
  ~LocalReserver();
  // port=0 时由内核选端口,经 bound_port() 取回。on_fatal_error 在
  // accept_loop 因不可恢复错误永久退出时被调用恰好一次(poll() 本身出错/
  // 监听 socket 坏掉——见 accept_loop() 里的两处 running_.store(false)),
  // 跑在 accept_loop 的线程上;调用方不应该在回调里做耗时的事。默认空,
  // 保持向后兼容——不传就是"和以前一样,什么都不做"。
  bool start(int port, const std::string& host = "127.0.0.1",
             OnFatalError on_fatal_error = {});
  void stop();
  int bound_port() const { return bound_port_.load(); }
  size_t client_count() const;
  void broadcast(const uint8_t* data, size_t len);

private:
  void accept_loop();

  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> bound_port_{-1};
  // 原先是裸 int:stop() 和 accept_loop 的自清理路径会跨线程读写它们
  // (fix round 1 review 的 Important 3),改成 atomic 并统一用
  // exchange(-1) 模式关闭,避免竞态和双重 close()。
  std::atomic<int> listen_fd_{-1};
  std::atomic<int> wake_fd_{-1};
  std::atomic<int> wake_wr_{-1};
  mutable std::mutex m_;
  std::vector<int> clients_;
  // broadcast() 摘下来、等着被 accept_loop 真正 close() 的死连接;
  // 只有 accept_loop 会读它、清空它、close() 里面的 fd。
  std::vector<int> to_drop_;
  // 只在 start() 里被设置一次(设置之后 accept_loop 线程才会启动),
  // accept_loop 线程只读,不存在和 start()/stop() 的数据竞争。
  OnFatalError on_fatal_error_;
};

}  // namespace gnss_bringup
