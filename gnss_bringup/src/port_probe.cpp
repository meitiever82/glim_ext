#include "gnss_bringup/port_probe.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>

namespace gnss_bringup {

// fix round 1 的 Important 3(含独立评审附带的 Minor):换成 bind()+
// SO_REUSEADDR 探测,而不是原来的 connect() + poll() 超时。原来的 connect()
// 方案有两个用真实实验复现过的问题,bind() 直接从根上绕开:
//   1. 我们真正关心的问题是"待会儿 rtkrcv 自己去 bind()/listen() 这个端口
//      时会不会成功"——bind() 直接回答这个问题;connect() 回答的是一个不
//      完全一样的问题("现在有没有人在 accept 这个端口"),一个正在监听、
//      但 accept 队列恰好占满的孤儿(它是被遗弃的,没有人在 accept())会
//      被 connect() 探测误判成"空闲"(用 listen(fd,1) + 多个排队连接实测
//      复现过);bind() 不关心对方的 accept 队列状态,只关心这个地址:端口
//      有没有被别的 socket 占着。
//   2. bind() 是一次同步、立即返回、不会被信号打断的调用,不存在 connect()
//      方案里"非阻塞 connect() + poll() 超时,poll() 还可能被 EINTR 打断
//      需要重试"这一整类问题,也不需要给一个"多久算超时"的经验值,更不会
//      向一个可能存在的孤儿发起任何连接。
// SO_REUSEADDR 只放行"绑定到处于 TIME_WAIT 的地址"这一类场景,不会让两个
// 进程真的同时监听同一个地址:端口(那需要双方都设 SO_REUSEPORT)——对一个
// 已经在 LISTEN 的地址:端口,bind() 仍然会正确返回 EADDRINUSE,不受这个
// 选项影响。
PortProbeOutcome probe_local_port(int port) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return PortProbeOutcome{PortProbeResult::kProbeFailed, errno};
  }

  int one = 1;
  // 这里的 setsockopt 失败不影响正确性(顶多是没拿到 TIME_WAIT 复用这个
  // 优化),不值得单独判定成探测失败,忽略返回值。
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = ::htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) != 1) {
    // 探测目标固定是字面量 "127.0.0.1",正常不会走到这里;真出现了,如实
    // 报告成探测失败,不猜一个结果出来。
    const int err = errno;
    ::close(fd);
    return PortProbeOutcome{PortProbeResult::kProbeFailed, err};
  }

  PortProbeOutcome outcome;
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
    outcome = PortProbeOutcome{PortProbeResult::kFree, 0};
  } else if (errno == EADDRINUSE) {
    outcome = PortProbeOutcome{PortProbeResult::kListening, 0};
  } else {
    outcome = PortProbeOutcome{PortProbeResult::kProbeFailed, errno};
  }

  ::close(fd);
  return outcome;
}

}  // namespace gnss_bringup
