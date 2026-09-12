#include "gnss_bringup/tcp_stream.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace gnss_bringup {

TcpStream::TcpStream(TcpStreamConfig cfg, OnData on_data, OnState on_state)
    : cfg_(std::move(cfg)), on_data_(std::move(on_data)), on_state_(std::move(on_state)) {}

TcpStream::~TcpStream() { stop(); }

void TcpStream::start() {
  if (running_.exchange(true)) {
    return;  // 已在运行,忽略重复 start()
  }

  // eventfd 同时充当"唤醒 fd":stop() 往里写一个计数,
  // 任何阻塞在 poll(wake_fd_, ...) 上的地方都会立刻被唤醒。
  const int efd = ::eventfd(0, EFD_NONBLOCK);
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
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      report(false, std::string("socket() failed: ") + std::strerror(errno));
      if (wait_backoff(backoff)) break;
      backoff = std::min(backoff * 2.0, cfg_.max_backoff_s);
      continue;
    }

    // 非阻塞 connect,好让 stop() 也能打断"正在连接中"的等待。
    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = ::htons(static_cast<uint16_t>(cfg_.port));
    ::inet_pton(AF_INET, cfg_.host.c_str(), &addr.sin_addr);

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

    // 连接成功:恢复阻塞模式(读循环用 poll() 控制,不需要非阻塞语义),退避复位。
    ::fcntl(fd, F_SETFL, flags);
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
