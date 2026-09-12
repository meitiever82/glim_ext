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
      run_server();
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
    //
    // 但普通 getaddrinfo() 是不可打断的阻塞调用:一旦 cfg_.host 是真主机名且
    // DNS 慢/不可达,worker 会卡在里面,wake_fd_ 完全不会被 poll 到——
    // stop()/析构里的 join() 会跟着卡 glibc 解析器重试完所有 nameserver 的
    // 那几十秒,重新引入 finding 2/7 刚关掉的"绝不能挂死"问题,只是提前到了
    // 连接前的解析阶段。先用 AI_NUMERICHOST 走一次零 DNS 流量的快路径:
    // cfg_.host 是 IPv4 字面量时(本包当前两条部署配置都是 IP:port,不是
    // 主机名)该调用不发任何网络请求,立即返回。只有当它不是数字地址时才退回
    // 完整解析——这一步仍然可能阻塞,是本包目前唯一已知的残留阻塞点:真配置
    // 成主机名且 DNS 慢/不可达时,stop() 就不再是"及时"的。收窄而非消除,
    // 因为不引入超时/可取消的自研解析器(或改全局 resolver 配置,后者明确
    // 被要求不做)就换不来更好的方案,留给以后真的需要主机名时再处理。
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_NUMERICSERV | AI_NUMERICHOST;
    addrinfo* resolved = nullptr;
    int gai_rc =
        ::getaddrinfo(cfg_.host.c_str(), std::to_string(cfg_.port).c_str(), &hints, &resolved);
    if (gai_rc != 0) {
      // 不是数字 IP 字面量:退回真正的(可能触发 DNS、可能阻塞)解析。
      if (resolved) {
        ::freeaddrinfo(resolved);
        resolved = nullptr;
      }
      hints.ai_flags = AI_NUMERICSERV;
      gai_rc =
          ::getaddrinfo(cfg_.host.c_str(), std::to_string(cfg_.port).c_str(), &hints, &resolved);
    }
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

    const PumpResult result = pump(fd, report);
    ::close(fd);
    if (result == PumpResult::kStopped || !running_.load()) break;

    if (wait_backoff(backoff)) break;
    backoff = std::min(backoff * 2.0, cfg_.max_backoff_s);
  }
}

TcpStream::PumpResult TcpStream::pump(int fd, const OnState& report, int preempt_fd) {
  uint8_t buf[4096];
  while (running_.load()) {
    pollfd fds[3];
    fds[0] = {fd, POLLIN, 0};
    fds[1] = {wake_fd_.load(), POLLIN, 0};
    nfds_t nfds = 2;
    if (preempt_fd >= 0) {
      fds[2] = {preempt_fd, POLLIN, 0};
      nfds = 3;
    }
    const int timeout_ms = cfg_.idle_timeout_s > 0
                                ? static_cast<int>(cfg_.idle_timeout_s * 1000.0)
                                : -1;
    const int pr = ::poll(fds, nfds, timeout_ms);
    if (pr < 0) {
      if (errno == EINTR) continue;
      if (report) report(false, std::string("poll() failed: ") + std::strerror(errno));
      return PumpResult::kDisconnected;
    }
    if (fds[1].revents & POLLIN) {
      return PumpResult::kStopped;  // stop() 唤醒
    }
    if (nfds == 3 && (fds[2].revents & POLLIN)) {
      // 监听 socket 上已经有新对端排队。哪怕当前连接还在正常收数据也不例外
      // ——"新连接优先"是这里刻意选的语义(见 tcp_stream.hpp `listen`
      // 字段旁的说明),不在这里 accept,交回给 run_server() 的 accept 循环
      // 去做,避免这里重复一份 accept 的错误处理。
      return PumpResult::kPreempted;
    }
    if (pr == 0) {
      // 静默超过 idle_timeout_s:链路差时对端常不发 RST 就消失,视为断开。
      if (report) report(false, "idle timeout");
      return PumpResult::kDisconnected;
    }
    if (fds[0].revents & (POLLIN | POLLHUP | POLLERR)) {
      const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
      if (n > 0) {
        if (on_data_) on_data_(buf, static_cast<size_t>(n));
      } else if (n == 0) {
        if (report) report(false, "peer closed connection");
        return PumpResult::kDisconnected;
      } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;  // 非阻塞 fd 上的虚假唤醒,重新 poll
      } else {
        if (report) report(false, std::string("recv() failed: ") + std::strerror(errno));
        return PumpResult::kDisconnected;
      }
    }
  }
  // running_ 被外部置 false,但不是走 stop() 的 wake fd 路径(理论上不应该
  // 发生,防御性兜底):同样应该结束整个 worker,不能当成"这次连接自己
  // 断了"去重连/重新 accept。
  return PumpResult::kStopped;
}

void TcpStream::run_server() {
  bool have_last_state = false;
  bool last_state = false;
  auto report = [&](bool connected, const std::string& detail) {
    if (!on_state_) return;
    if (have_last_state && last_state == connected) return;  // 仅在跳变时上报
    on_state_(connected, detail);
    have_last_state = true;
    last_state = connected;
  };

  // 解析监听地址:用 cfg_.host 而不是硬编码 INADDR_LOOPBACK——之前的最小实现
  // 忽略了 cfg_.host,配置成别的本机地址(例如要对外网卡监听)也不起作用。
  // 和 run_client() 一样先走 AI_NUMERICHOST 零 DNS 快路径,cfg_.host 是
  // IP 字面量(当前两条部署配置都是)时立即返回,不引入解析阻塞。
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_NUMERICSERV | AI_NUMERICHOST | AI_PASSIVE;
  addrinfo* resolved = nullptr;
  int gai_rc =
      ::getaddrinfo(cfg_.host.c_str(), std::to_string(cfg_.port).c_str(), &hints, &resolved);
  if (gai_rc != 0) {
    if (resolved) {
      ::freeaddrinfo(resolved);
      resolved = nullptr;
    }
    hints.ai_flags = AI_NUMERICSERV | AI_PASSIVE;
    gai_rc =
        ::getaddrinfo(cfg_.host.c_str(), std::to_string(cfg_.port).c_str(), &hints, &resolved);
  }
  if (gai_rc != 0 || resolved == nullptr) {
    // 绑定地址解析失败必须上报,否则调用方只会看到 bound_port() 恒为 -1,
    // 却查不到原因(这正是 Task 1 遗留的最小实现里缺失的一环)。
    report(false, "resolve listen address " + cfg_.host + " failed: " + ::gai_strerror(gai_rc));
    if (resolved) ::freeaddrinfo(resolved);
    running_.store(false);
    return;
  }
  sockaddr_in bind_addr{};
  std::memcpy(&bind_addr, resolved->ai_addr, sizeof(bind_addr));
  bind_addr.sin_port = ::htons(static_cast<uint16_t>(cfg_.port));
  ::freeaddrinfo(resolved);

  // 监听 socket 本身必须非阻塞。原因:poll() 报告监听 fd 可读之后、accept()
  // 真正被调用之前,对端完全可能已经把连接撤回(SYN 之后马上一个 RST)——
  // 这段时间差不受这里代码控制。socket 是阻塞的话,accept() 就会在一个
  // 已经空了的队列上挂住,而 wake_fd 只在 poll() 里被监听,对陷在 accept()
  // 里的线程完全无能为力,stop()/析构里的 join() 会永久挂起——和 eventfd
  // 分配失败、client 侧 recv() 前不切回阻塞模式,是同一类"绝不能挂死"问题。
  int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
  if (fd < 0) {
    report(false, std::string("socket() failed: ") + std::strerror(errno));
    running_.store(false);
    return;
  }
  int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  if (::bind(fd, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0) {
    report(false, std::string("bind() failed: ") + std::strerror(errno));
    ::close(fd);
    running_.store(false);
    return;
  }
  if (::listen(fd, 4) != 0) {
    report(false, std::string("listen() failed: ") + std::strerror(errno));
    ::close(fd);
    running_.store(false);
    return;
  }

  sockaddr_in actual{};
  socklen_t len = sizeof(actual);
  ::getsockname(fd, reinterpret_cast<sockaddr*>(&actual), &len);
  bound_port_.store(::ntohs(actual.sin_port));

  // accept() 失败后用 poll(wake_fd) 等一小段时间再重试,而不是立刻重新
  // poll+accept:EMFILE/ENFILE/ENOBUFS/EPERM 这类错误不会自愈,那个连接会
  // 一直留在内核的 accept 队列里,poll() 会立刻再次报告 POLLIN——如果这里
  // 不停顿就重试,就是 100% CPU 的忙转圈(fd 耗尽时复现过)。用 wake_fd 上
  // 的 poll 等,这样 stop() 依然能立刻打断这段等待。
  auto wait_a_bit = [this](double seconds) -> bool {
    pollfd pfd{wake_fd_.load(), POLLIN, 0};
    const int timeout_ms = static_cast<int>(seconds * 1000.0);
    const int pr = ::poll(&pfd, 1, timeout_ms);
    if (pr > 0 && (pfd.revents & POLLIN)) return true;
    return !running_.load();
  };

  while (running_.load()) {
    pollfd fds[2] = {{fd, POLLIN, 0}, {wake_fd_.load(), POLLIN, 0}};
    const int pr = ::poll(fds, 2, -1);
    if (pr < 0) {
      if (errno == EINTR) continue;
      report(false, std::string("poll() failed: ") + std::strerror(errno));
      break;
    }
    if (fds[1].revents & POLLIN) break;  // stop() 唤醒
    if (!(fds[0].revents & POLLIN)) {
      if (fds[0].revents & (POLLERR | POLLNVAL)) {
        // 监听 socket 本身坏掉了(不是"有连接进来"那种正常可读),不是
        // 重试能恢复的瞬时状况——继续 poll 只会在这里原地打转,同样是
        // finding 里点名的那种热循环。上报后直接结束这个 worker。
        report(false, "listen socket error");
        break;
      }
      continue;  // 其它虚假唤醒,重新 poll
    }

    const int c = ::accept(fd, nullptr, nullptr);
    if (c < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ECONNABORTED ||
          errno == EINTR) {
        // 瞬时状况:EAGAIN/EWOULDBLOCK 是监听 socket 非阻塞后,poll 报告
        // 可读但对端在 accept() 之前已经用 RST 撤回了连接;ECONNABORTED 是
        // 同一类"对端半路撤回";EINTR 是信号打断。三者都应该直接回 poll
        // 重试,不需要上报也不需要停顿。
        continue;
      }
      // EMFILE/ENFILE/ENOBUFS/EPERM 等不会自愈:上报,然后停顿一下再回到
      // accept 循环,而不是原地忙转圈(见 wait_a_bit 上面的注释)。
      report(false, std::string("accept() failed: ") + std::strerror(errno));
      if (wait_a_bit(cfg_.initial_backoff_s)) break;
      continue;
    }

    // 和客户端 socket 一样保持非阻塞:pump() 的 poll+recv 组合依赖这一点
    // 才能在虚假唤醒时安全地重新 poll,而不是阻塞在 recv() 上。
    const int flags = ::fcntl(c, F_GETFL, 0);
    ::fcntl(c, F_SETFL, flags | O_NONBLOCK);
    report(true, "peer connected");

    // 把监听 fd 本身作为 preempt_fd 一并交给 pump():新连接优先——服务当前
    // 对端期间如果监听 fd 又变得可读(有新对端排队),pump() 立刻返回
    // kPreempted,不等 idle_timeout_s,也不等当前对端自己断开。
    const PumpResult result = pump(c, report, fd);
    ::close(c);
    if (result == PumpResult::kStopped) break;
    if (result == PumpResult::kPreempted) {
      // kPreempted 时 pump() 自己没有调用过 report(false, ...)(它只负责
      // 检测、不负责挂断),这里补上"断开"这一跳变。kDisconnected 分支
      // 不需要补,pump() 内部已经在对应的 idle timeout / 对端关闭 / I/O
      // 出错路径上报过了。
      report(false, "displaced by newer peer");
    }
    // 无论 kDisconnected 还是 kPreempted 都回到 accept 循环等下一个对端,
    // 不退出线程。kPreempted 时新连接已经排在监听 socket 的 accept 队列
    // 里,下一轮 poll(fd, wake_fd) 会立刻返回 POLLIN。
  }
  ::close(fd);
  bound_port_.store(-1);
}

}  // namespace gnss_bringup
