#include "gnss_bringup/local_reserver.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>

namespace gnss_bringup {

LocalReserver::~LocalReserver() { stop(); }

bool LocalReserver::start(int port, const std::string& host) {
  if (running_.exchange(true)) {
    return false;  // 已在运行,重复 start() 视为失败,不动现有状态
  }

  // 上一次 start() 可能因 bind/listen 失败而线程自行退出、但没人调用过
  // stop() 来 join 它——此时 thread_ 仍然 joinable。直接把新线程赋给
  // thread_ 会在一个 joinable 的 std::thread 上触发 std::terminate()。
  // 先把上一条线程收尾,再关掉可能残留的 wake 管道,避免每次失败重试都
  // 泄漏 fd。
  if (thread_.joinable()) {
    thread_.join();
  }
  if (wake_fd_ >= 0) {
    ::close(wake_fd_);
    wake_fd_ = -1;
  }
  if (wake_wr_ >= 0) {
    ::close(wake_wr_);
    wake_wr_ = -1;
  }

  // 自管道(self-pipe)当"唤醒 fd":stop() 往写端写一个字节,任何阻塞在
  // poll(wake_fd_, ...) 上的地方都会立刻被唤醒。两端都设非阻塞,避免
  // 管道满了的极端情况下 stop() 自己卡在 write() 里。
  int fds[2] = {-1, -1};
  if (::pipe2(fds, O_NONBLOCK | O_CLOEXEC) != 0) {
    // fd 耗尽(EMFILE 等)时绝不能带着一个造不出唤醒通道的对象去启动线程:
    // accept 循环会阻塞在 poll(listen_fd, -1, ...) 上且没有任何唤醒来源,
    // stop()/析构里的 join() 会永久挂起。启动失败,报告给调用方,不留后台
    // 线程。
    running_.store(false);
    return false;
  }
  wake_fd_ = fds[0];
  wake_wr_ = fds[1];

  // 监听 socket 必须非阻塞:poll() 报告可读之后、accept() 真正被调用之前,
  // 对端完全可能已经把连接撤回(SYN 之后马上一个 RST)。socket 是阻塞的话
  // accept() 会在一个已经空了的队列上挂住,wake_fd 只在 poll() 里被监听,
  // 对陷在 accept() 里的线程无能为力,stop() 会永久挂起。
  const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
  if (fd < 0) {
    running_.store(false);
    ::close(wake_fd_);
    ::close(wake_wr_);
    wake_fd_ = -1;
    wake_wr_ = -1;
    return false;
  }
  int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = ::htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    // host 不是合法的 IPv4 字面量(本类只服务本机回环场景,不需要 DNS)。
    ::close(fd);
    ::close(wake_fd_);
    ::close(wake_wr_);
    wake_fd_ = -1;
    wake_wr_ = -1;
    running_.store(false);
    return false;
  }

  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    ::close(fd);
    ::close(wake_fd_);
    ::close(wake_wr_);
    wake_fd_ = -1;
    wake_wr_ = -1;
    running_.store(false);
    return false;
  }
  if (::listen(fd, 4) != 0) {
    ::close(fd);
    ::close(wake_fd_);
    ::close(wake_wr_);
    wake_fd_ = -1;
    wake_wr_ = -1;
    running_.store(false);
    return false;
  }

  sockaddr_in actual{};
  socklen_t len = sizeof(actual);
  ::getsockname(fd, reinterpret_cast<sockaddr*>(&actual), &len);
  bound_port_.store(::ntohs(actual.sin_port));

  listen_fd_ = fd;
  thread_ = std::thread([this] { accept_loop(); });
  return true;
}

void LocalReserver::stop() {
  if (!running_.exchange(false)) {
    // 未在运行:可能从未 start() 过,也可能已经 stop() 过一次——两种情况
    // 都无害地直接返回,保持幂等。
    return;
  }
  if (wake_wr_ >= 0) {
    const uint8_t one = 1;
    const ssize_t written = ::write(wake_wr_, &one, sizeof(one));
    (void)written;  // 唤醒信号,写失败也无妨(线程可能已经在退出路上)
  }
  if (thread_.joinable()) {
    thread_.join();
  }

  // 关闭监听 socket 是这里的关键动作:必须真正释放端口,而不是只是不再
  // accept——StopReleasesThePortAndIsIdempotent 要求 stop() 之后一个全新的
  // 实例能立刻绑定同一个端口,SO_REUSEADDR 不能替代真正 close() 掉旧的
  // 监听 fd。
  if (listen_fd_ >= 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
  if (wake_fd_ >= 0) {
    ::close(wake_fd_);
    wake_fd_ = -1;
  }
  if (wake_wr_ >= 0) {
    ::close(wake_wr_);
    wake_wr_ = -1;
  }
  bound_port_.store(-1);

  std::vector<int> to_close;
  {
    std::lock_guard<std::mutex> lock(m_);
    to_close.swap(clients_);
  }
  for (const int c : to_close) {
    ::close(c);
  }
}

size_t LocalReserver::client_count() const {
  std::lock_guard<std::mutex> lock(m_);
  return clients_.size();
}

void LocalReserver::broadcast(const uint8_t* data, size_t len) {
  std::vector<int> snapshot;
  {
    std::lock_guard<std::mutex> lock(m_);
    snapshot = clients_;
  }
  if (snapshot.empty()) return;

  // 丢弃策略:客户端 socket 是非阻塞的,send() 对写不动的对端(内核发送
  // 缓冲已满,即 rtkrcv 侧迟迟不读)返回 EAGAIN/EWOULDBLOCK。这里不重试、
  // 不排队缓冲——直接关闭并从 clients_ 里摘掉该客户端,防止一个卡住的
  // 客户端让内核发送缓冲无界增长,或者让 broadcast() 反过来阻塞发布者
  // (rclcpp 订阅回调)线程。MSG_NOSIGNAL 防止对端已经整个关闭连接时
  // send() 触发 SIGPIPE 杀掉进程。
  std::vector<int> dead;
  for (const int c : snapshot) {
    const ssize_t n = ::send(c, data, len, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        dead.push_back(c);
      } else if (errno != EINTR) {
        dead.push_back(c);
      }
      // EINTR: 不算失败,保留该客户端,下次 broadcast 再试。
    } else if (static_cast<size_t>(n) != len) {
      // 非阻塞 TCP socket 上的短写同样视为"写不动",丢弃该客户端而不是
      // 尝试续写剩余字节——续写需要缓冲这批未发完的数据,和"不排队缓冲"
      // 的丢弃策略矛盾。
      dead.push_back(c);
    }
  }

  if (dead.empty()) return;
  std::lock_guard<std::mutex> lock(m_);
  for (const int d : dead) {
    ::close(d);
    for (auto it = clients_.begin(); it != clients_.end(); ++it) {
      if (*it == d) {
        clients_.erase(it);
        break;
      }
    }
  }
}

void LocalReserver::accept_loop() {
  uint8_t discard[256];
  // stopped_by_wake 区分两种退出路径:
  //  - true  : stop() 写唤醒字节触发的正常退出,清理工作交给 stop()(它会
  //            在 join() 之后关闭 listen_fd_/wake_fd_/wake_wr_)。
  //  - false : 线程自己因为不可恢复的错误(poll 出错、监听 socket 坏掉)
  //            退出——这种情况下没有人会调用 stop() 来做清理,必须在这里
  //            自己关掉所有 fd、把 running_/bound_port_ 复位,否则对象会
  //            永远卡在"看起来在跑但线程已经死了"的状态,start() 也无法
  //            重试(参考 tcp_stream.cpp 里同一类"自行退出必须让失败可见"
  //            的处理)。
  bool stopped_by_wake = false;

  // accept() 遇到不会自愈的错误(EMFILE/ENFILE/ENOBUFS/EPERM 等)时用来
  // 停顿一下再继续的等待:同样通过 poll(wake_fd_) 等,这样 stop() 依然能
  // 立刻打断这段等待,而不是真的 sleep()。
  auto wait_a_bit = [this](double seconds) -> bool {
    pollfd pfd{wake_fd_, POLLIN, 0};
    const int timeout_ms = static_cast<int>(seconds * 1000.0);
    const int pr = ::poll(&pfd, 1, timeout_ms);
    return pr > 0 && (pfd.revents & POLLIN);
  };

  while (running_.load()) {
    std::vector<int> snapshot;
    {
      std::lock_guard<std::mutex> lock(m_);
      snapshot = clients_;
    }

    std::vector<pollfd> fds;
    fds.reserve(snapshot.size() + 2);
    fds.push_back({listen_fd_, POLLIN, 0});
    fds.push_back({wake_fd_, POLLIN, 0});
    for (const int c : snapshot) {
      fds.push_back({c, POLLIN, 0});
    }

    const int pr = ::poll(fds.data(), fds.size(), -1);
    if (pr < 0) {
      if (errno == EINTR) continue;
      // poll() 本身出错不是能自愈的瞬时状况,继续在这里 poll 只会原地
      // 打转——直接结束线程,交给下面的自清理逻辑。
      break;
    }

    if (fds[1].revents & POLLIN) {
      stopped_by_wake = true;
      break;  // stop() 唤醒
    }

    if (fds[0].revents & (POLLERR | POLLNVAL)) {
      // 监听 socket 本身坏掉了(不是"有连接进来"那种正常可读),继续 poll
      // 只会在这里原地打转——同样是"绝不能热循环"的情形,直接结束线程。
      break;
    }
    if (fds[0].revents & POLLIN) {
      // accept() 结果按 errno 分类:EAGAIN/EWOULDBLOCK/ECONNABORTED/EINTR
      // 是瞬时状况(poll 报告可读之后对端已经用 RST 撤回连接,或者是信号
      // 打断),直接回 poll 重试/继续排空队列。其它 errno
      // (EMFILE/ENFILE/ENOBUFS/EPERM 等)不会自愈——`if (fd < 0) continue;`
      // 这种写法在 EMFILE 下会产生验证过的 100% CPU 忙转圈:监听 socket
      // 上那个撤不掉的连接会让 poll 立刻再次报告 POLLIN。这里改为停顿一小
      // 段(仍然响应 wake_fd)再回到外层 poll,不在本轮里重试 accept()。
      bool backoff = false;
      for (;;) {
        const int c = ::accept4(listen_fd_, nullptr, nullptr, SOCK_NONBLOCK);
        if (c < 0) {
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;  // 队列已空,正常结束这一轮 accept
          }
          if (errno == ECONNABORTED || errno == EINTR) {
            continue;  // 瞬时状况,继续排空队列
          }
          backoff = true;
          break;
        }
        std::lock_guard<std::mutex> lock(m_);
        clients_.push_back(c);
      }
      if (backoff && wait_a_bit(0.1)) {
        stopped_by_wake = true;
        break;
      }
    }

    // 客户端不会主动发数据,但仍然要把到来的字节 recv() 丢掉,否则对端
    // 一旦真的写了点什么就会卡在内核发送缓冲里出不来。同时用 recv()==0
    // 或错误来检测"对端已断开",从 clients_ 里摘掉。
    std::vector<int> dead;
    for (size_t i = 2; i < fds.size(); ++i) {
      if (!(fds[i].revents & (POLLIN | POLLHUP | POLLERR))) continue;
      const int c = fds[i].fd;
      const ssize_t n = ::recv(c, discard, sizeof(discard), 0);
      if (n == 0) {
        dead.push_back(c);
      } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        dead.push_back(c);
      }
    }
    if (!dead.empty()) {
      std::lock_guard<std::mutex> lock(m_);
      for (const int d : dead) {
        ::close(d);
        for (auto it = clients_.begin(); it != clients_.end(); ++it) {
          if (*it == d) {
            clients_.erase(it);
            break;
          }
        }
      }
    }
  }

  if (!stopped_by_wake) {
    // 自行退出:没有 stop() 调用方等着在 join() 之后做清理,这里必须自己
    // 把状态收拾干净,否则 running_ 会永远卡在 true——之后既没人能再
    // start(),bound_port() 也会撒谎说还绑着一个其实已经死掉的监听 socket。
    std::vector<int> to_close;
    {
      std::lock_guard<std::mutex> lock(m_);
      to_close.swap(clients_);
    }
    for (const int c : to_close) {
      ::close(c);
    }
    if (listen_fd_ >= 0) {
      ::close(listen_fd_);
      listen_fd_ = -1;
    }
    if (wake_fd_ >= 0) {
      ::close(wake_fd_);
      wake_fd_ = -1;
    }
    if (wake_wr_ >= 0) {
      ::close(wake_wr_);
      wake_wr_ = -1;
    }
    bound_port_.store(-1);
    running_.store(false);
  }
}

}  // namespace gnss_bringup
