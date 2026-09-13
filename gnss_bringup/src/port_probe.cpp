#include "gnss_bringup/port_probe.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace gnss_bringup {

// 用一次 connect() 尝试判断 127.0.0.1:port 是否有人在监听——连得上就说明有人
// 监听(不关心对方是谁,`rtkrcv_node` 只用这个函数在启动时"占没占着"这个
// 二元判断上,不需要知道占用者的身份)。
//
// 非阻塞 connect() + 有界 poll() 超时,而不是一次阻塞 connect():这个函数在
// rtkrcv_node 的构造函数里同步调用(还没有起 executor/spin()),阻塞的
// connect() 一旦卡住(理论上不会发生在 127.0.0.1 上,但没有任何文档保证
// 这一点)就会让整个节点在启动阶段失去响应。127.0.0.1 本机回环上,
// connect() 无论成功还是被 RST 拒绝都应该是微秒级的,这里的超时只是一个
// "不可能触发,但触发了也不能让节点卡死"的安全网,不是期望的正常路径。
bool is_local_port_listening(int port) {
  const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
  if (fd < 0) {
    // socket() 都失败了(fd 耗尽等极端情况),没法探测,保守地当成"没有
    // 监听"——调用方(rtkrcv_node)在这个函数返回 false 时会继续正常启动
    // 流程,不会因为探测本身的失败而拒绝启动。
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = ::htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) != 1) {
    ::close(fd);
    return false;
  }

  bool listening = false;
  const int rc = ::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  if (rc == 0) {
    listening = true;
  } else if (errno == EINPROGRESS) {
    // 非阻塞 connect() 尚未完成:poll(POLLOUT) 等它有结果。200ms 对本机回环
    // 连接来说已经是一个非常宽裕的上限——见函数顶部的说明。
    pollfd pfd{fd, POLLOUT, 0};
    const int pr = ::poll(&pfd, 1, 200);
    if (pr > 0 && (pfd.revents & POLLOUT)) {
      int so_error = 0;
      socklen_t len = sizeof(so_error);
      if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len) == 0 && so_error == 0) {
        listening = true;
      }
    }
    // pr<=0(超时/poll 出错)或者 so_error!=0(ECONNREFUSED 等):没有人监听,
    // listening 保持 false。
  }
  // rc<0 且 errno!=EINPROGRESS:立即失败(比如 ECONNREFUSED——本机回环上
  // 最常见的"没人监听"信号),listening 保持 false。

  ::close(fd);
  return listening;
}

}  // namespace gnss_bringup
