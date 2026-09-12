#include "gnss_bringup/tcp_stream.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace gnss_bringup {

namespace {
// 退避区间下限:0(或负数)会让 poll(timeout=0) 立刻返回,配合 backoff*2 恒为 0,
// 形成 100% CPU 的 socket/connect/close 忙转圈。下限选一个足够小、
// 不影响正常重连体验、但能避免忙等的值。
constexpr double kMinBackoffS = 0.05;
}  // namespace

TcpStream::TcpStream(TcpStreamConfig cfg, OnData on_data, OnState on_state)
    : cfg_(std::move(cfg)), on_data_(std::move(on_data)), on_state_(std::move(on_state)) {
  // 校验退避参数:0/负值会让退避睡眠要么忙等(0 或极小值经 poll 立即超时,
  // backoff*2 仍是 0)要么永久阻塞(负值转成的 timeout_ms 为负,poll 视为
  // "无限等待"),两者都会破坏"永不放弃"的重连语义。这里夹到一个安全下限,
  // 并保证 max >= initial,避免一次错误的 YAML 配置就让整条流失效。
  if (!(cfg_.initial_backoff_s >= kMinBackoffS)) {
    cfg_.initial_backoff_s = kMinBackoffS;
  }
  if (!(cfg_.max_backoff_s >= cfg_.initial_backoff_s)) {
    cfg_.max_backoff_s = cfg_.initial_backoff_s;
  }
}

TcpStream::~TcpStream() { stop(); }

void TcpStream::start() {
  if (running_.exchange(true)) {
    return;  // 已在运行,忽略重复 start()
  }

  // 上一次 start() 可能因监听 socket/bind/listen 失败而自行把 running_ 置 false
  // 并返回,但没有人调用过 stop() 来 join 它——此时 thread_ 仍然 joinable。
  // 直接把新线程赋给 thread_ 会在一个 joinable 的 std::thread 上触发
  // std::terminate()。先把上一条线程收尾,再复用 wake_fd_ 的旧 fd(如果有)
  // 一并关掉,避免每次失败重试都泄漏一个 eventfd。
  if (thread_.joinable()) {
    thread_.join();
  }
  const int old_efd = wake_fd_.exchange(-1);
  if (old_efd >= 0) {
    ::close(old_efd);
  }

  // eventfd 同时充当"唤醒 fd":stop() 往里写一个计数,
  // 任何阻塞在 poll(wake_fd_, ...) 上的地方都会立刻被唤醒。
  const int efd = ::eventfd(0, EFD_NONBLOCK);
  if (efd < 0) {
    // fd 耗尽(EMFILE 等)时绝不能把 -1 存进 wake_fd_ 后仍然启动线程:
    // poll() 会忽略负数 fd,监听模式的 worker 将阻塞在 poll(..., -1) 上
    // 且没有任何唤醒来源,stop()/析构里的 join() 会永久挂起——这正是
    // "机器人绝不能挂死"要避免的情况。启动失败,报告给调用方,不留后台线程。
    running_.store(false);
    if (on_state_) {
      on_state_(false, std::string("eventfd() failed: ") + std::strerror(errno));
    }
    return;
  }
  wake_fd_.store(efd);

  thread_ = std::thread([this] {
    if (cfg_.listen) {
      // Task 1 只实现客户端模式;监听模式在此仅做最小实现——
      // 绑定端口、暴露 bound_port()、能被 stop()/析构 干净地关掉,
      // 完整的“接受连接后转发数据”的行为留给 Task 2。
      int fd = ::socket(AF_INET, SOCK_STREAM, 0);
      if (fd < 0) {
        running_.store(false);
        return;
      }
      int one = 1;
      ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

      sockaddr_in addr{};
      addr.sin_family = AF_INET;
      addr.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
      addr.sin_port = ::htons(static_cast<uint16_t>(cfg_.port));
      if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
          ::listen(fd, 4) != 0) {
        ::close(fd);
        running_.store(false);
        return;
      }
      socklen_t len = sizeof(addr);
      ::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len);
      bound_port_.store(::ntohs(addr.sin_port));

      while (running_.load()) {
        pollfd fds[2] = {{fd, POLLIN, 0}, {wake_fd_.load(), POLLIN, 0}};
        const int pr = ::poll(fds, 2, -1);
        if (pr < 0) {
          if (errno == EINTR) continue;
          break;
        }
        if (fds[1].revents & POLLIN) break;  // stop() 唤醒
        if (fds[0].revents & POLLIN) {
          const int c = ::accept(fd, nullptr, nullptr);
          if (c >= 0) ::close(c);  // Task 1 不处理数据,直接关闭
        }
      }
      ::close(fd);
      bound_port_.store(-1);
    } else {
      run_client();
    }
  });
}

void TcpStream::stop() {
  running_.store(false);
  const int efd = wake_fd_.load();
  if (efd >= 0) {
    const uint64_t one = 1;
    const ssize_t written = ::write(efd, &one, sizeof(one));
    (void)written;  // 唤醒信号,写失败也无妨(线程可能已退出)
  }
  if (thread_.joinable()) {
    thread_.join();
  }
  if (efd >= 0) {
    ::close(efd);
    wake_fd_.store(-1);
  }
}

void TcpStream::run_client() {
  bool have_last_state = false;
  bool last_state = false;
  auto report = [&](bool connected, const std::string& detail) {
    if (!on_state_) return;
    if (have_last_state && last_state == connected) return;  // 仅在跳变时上报
    on_state_(connected, detail);
    have_last_state = true;
    last_state = connected;
  };

  // 退避睡眠:用 poll() 等唤醒 fd,而不是 sleep(),这样 stop() 能立刻打断。
  // 返回 true 表示应当结束(被唤醒或已被要求停止)。
  auto wait_backoff = [this](double seconds) -> bool {
    pollfd pfd{wake_fd_.load(), POLLIN, 0};
    const int timeout_ms = static_cast<int>(seconds * 1000.0);
    const int pr = ::poll(&pfd, 1, timeout_ms);
    if (pr > 0 && (pfd.revents & POLLIN)) return true;
    return !running_.load();
  };

  double backoff = cfg_.initial_backoff_s;

  while (running_.load()) {
    // 用 getaddrinfo() 解析主机名/IP。之前直接用 inet_pton() 且不检查返回值:
    // 一旦 cfg_.host 是主机名(真实差分定位服务大多是域名,如
    // "rtk-caster.cn"),inet_pton() 会失败并把已清零的 sin_addr 原样留在
    // 0.0.0.0——Linux 上 connect(0.0.0.0:port) 等价于 connect(127.0.0.1:port)。
    // 如果本机恰好有别的东西监听那个端口(哪怕是自己 Task 2 的监听 socket),
    // 就会悄悄连到自己、报 connected、转发垃圾数据,没有任何报错或状态跳变。
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_NUMERICSERV;
    addrinfo* resolved = nullptr;
    const int gai_rc =
        ::getaddrinfo(cfg_.host.c_str(), std::to_string(cfg_.port).c_str(), &hints, &resolved);
    if (gai_rc != 0 || resolved == nullptr) {
      report(false, "resolve " + cfg_.host + " failed: " + ::gai_strerror(gai_rc));
      if (resolved) ::freeaddrinfo(resolved);
      if (!running_.load()) break;
      if (wait_backoff(backoff)) break;
      backoff = std::min(backoff * 2.0, cfg_.max_backoff_s);
      continue;
    }
    sockaddr_in addr{};
    std::memcpy(&addr, resolved->ai_addr, sizeof(addr));
    ::freeaddrinfo(resolved);

    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      report(false, std::string("socket() failed: ") + std::strerror(errno));
      if (wait_backoff(backoff)) break;
      backoff = std::min(backoff * 2.0, cfg_.max_backoff_s);
      continue;
    }

    // 非阻塞 connect,好让 stop() 也能打断"正在连接中"的等待。
    // 连接建立后也刻意保持非阻塞(见下方读循环),不切回阻塞模式——
    // 一旦切回阻塞,poll() 报告 POLLIN 之后到 recv() 之间如果没有可读字节
    // (例如只有 POLLHUP/短暂的虚假唤醒),recv() 会阻塞且没有唤醒路径,
    // stop() 就会挂死在 join() 上,和 eventfd 分配失败是同一类"绝不能挂死"问题。
    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    int rc = ::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    bool connect_failed = false;
    bool woken_while_connecting = false;
    if (rc != 0 && errno == EINPROGRESS) {
      pollfd fds[2] = {{fd, POLLOUT, 0}, {wake_fd_.load(), POLLIN, 0}};
      const int pr = ::poll(fds, 2, -1);
      if (pr < 0) {
        connect_failed = true;
      } else if (fds[1].revents & POLLIN) {
        woken_while_connecting = true;
      } else if (fds[0].revents & (POLLOUT | POLLERR | POLLHUP)) {
        int soerr = 0;
        socklen_t slen = sizeof(soerr);
        ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &slen);
        if (soerr != 0) {
          errno = soerr;
          connect_failed = true;
        }
      }
    } else if (rc != 0) {
      connect_failed = true;
    }

    if (woken_while_connecting) {
      ::close(fd);
      break;
    }
    if (connect_failed) {
      report(false, std::string("connect() failed: ") + std::strerror(errno));
      ::close(fd);
      if (!running_.load()) break;
      if (wait_backoff(backoff)) break;
      backoff = std::min(backoff * 2.0, cfg_.max_backoff_s);
      continue;
    }

    backoff = cfg_.initial_backoff_s;
    report(true, "connected to " + cfg_.host + ":" + std::to_string(cfg_.port));

    uint8_t buf[4096];
    bool woken = false;
    while (running_.load()) {
      pollfd fds[2] = {{fd, POLLIN, 0}, {wake_fd_.load(), POLLIN, 0}};
      const int timeout_ms = cfg_.idle_timeout_s > 0
                                  ? static_cast<int>(cfg_.idle_timeout_s * 1000.0)
                                  : -1;
      const int pr = ::poll(fds, 2, timeout_ms);
      if (pr < 0) {
        if (errno == EINTR) continue;
        report(false, std::string("poll() failed: ") + std::strerror(errno));
        break;
      }
      if (fds[1].revents & POLLIN) {
        woken = true;
        break;
      }
      if (pr == 0) {
        // 静默超过 idle_timeout_s:链路差时对端常不发 RST 就消失,视为断开重连。
        report(false, "idle timeout");
        break;
      }
      if (fds[0].revents & (POLLIN | POLLHUP | POLLERR)) {
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n > 0) {
          if (on_data_) on_data_(buf, static_cast<size_t>(n));
        } else if (n == 0) {
          report(false, "peer closed connection");
          break;
        } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;  // 非阻塞 fd 上的虚假唤醒,重新 poll
        } else {
          report(false, std::string("recv() failed: ") + std::strerror(errno));
          break;
        }
      }
    }
    ::close(fd);
    if (woken || !running_.load()) break;

    if (wait_backoff(backoff)) break;
    backoff = std::min(backoff * 2.0, cfg_.max_backoff_s);
  }
}

}  // namespace gnss_bringup
