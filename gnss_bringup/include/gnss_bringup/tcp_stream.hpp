#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

namespace gnss_bringup {

using OnData = std::function<void(const uint8_t* data, size_t len)>;
// connected=true 进入连通态,false 断开;detail 供日志。
//
// final-fix-wave 第 2 项新增的 terminal:true 表示这次回调对应的是 worker
// 线程本身正在永久退出(eventfd()/监听地址解析/socket()/bind()/listen() 失败,
// 或者 accept 循环里 poll() 本身出错/监听 socket 坏掉)——此后不会再有任何
// 连接尝试,这条流已经彻底死掉,只是对象仍然"看起来在跑"(bound_port()/
// running 状态不会主动告诉调用方)。false 表示这是重连/断线过程中的一次
// 普通跳变(对端关闭、空闲超时、单次 connect()/recv() 失败等),worker 还会
// 继续退避重试。调用方必须区分对待:terminal=true 应该记成 ERROR(现场唯一
// 的信号来源——两个节点都不会自动重启一个自行退出的 worker,也没有健康检查
// 话题),terminal=false 仍然按 INFO 记录,不能因为这次改动把正常的重连噪声
// 也升级成 ERROR。
using OnState = std::function<void(bool connected, const std::string& detail, bool terminal)>;

struct TcpStreamConfig {
  std::string host = "127.0.0.1";
  int port = 0;                     // listen 模式下 0 = 让内核选端口(测试用)
  // true: 本机监听等对端连进来;false: 主动连对端。
  // 监听模式同一时刻只服务一个对端,且是"新连接优先"语义:如果有活跃对端时
  // 又有新连接进来,会立刻断开旧的、转去服务新的(旧对端因此可能连一个字节
  // 都没收完就被挂断)。这是为"平台主动推流到车上"这个场景选的:链路
  // 抖动/NAT 超时导致旧连接变成没有 RST 的假活连接时,不能让它占着位置等
  // idle_timeout_s 超时,必须让重新连上来的新连接立刻顶替。
  bool listen = false;
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

  // pump() 的结束原因:
  //   kStopped     —— 被 stop() 打断(wake_fd 可读),调用方应结束整个 worker。
  //   kDisconnected —— 这次连接自身结束了(对端关闭 / 静默超时 / I/O 出错),
  //                    调用方按自己的策略处理——客户端退避重连,服务端回到
  //                    accept 等下一个对端。
  //   kPreempted   —— 仅服务端、且传了 preempt_fd 时才会出现:监听 socket
  //                    上已经有新连接排队,调用方必须立刻挂断当前对端、
  //                    上报断开、回到 accept 收下新对端(见 tcp_stream.hpp
  //                    里 `listen` 字段旁"新连接优先"的说明)。
  enum class PumpResult { kStopped, kDisconnected, kPreempted };

  // 客户端(已连接的 fd)与服务端(已 accept 的 fd)共用的收数循环:
  // poll(fd, wake_fd[, preempt_fd]) 等可读,idle_timeout_s 静默视为断开,
  // recv 到的数据经 on_data_ 回调,状态变化(idle timeout / 对端关闭 / poll
  // 或 recv 出错)经 report 回调上报。preempt_fd 传 >=0 时(仅服务端使用)
  // 一并监听该 fd——通常是监听 socket 本身——它可读即返回 kPreempted,
  // 不在这里做 accept,把"关旧连新"的动作留给调用方统一处理。
  PumpResult pump(int fd, const OnState& report, int preempt_fd = -1);

  TcpStreamConfig cfg_;
  OnData on_data_;
  OnState on_state_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> bound_port_{-1};
  std::atomic<int> wake_fd_{-1};    // stop() 用来打断阻塞中的 recv/accept
};

}  // namespace gnss_bringup
