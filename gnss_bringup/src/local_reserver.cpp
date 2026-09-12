#include "gnss_bringup/local_reserver.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>

namespace gnss_bringup {

namespace {
// exchange(-1) 拿到的旧值 >=0 才真正 close(),否则(-1,即"本来就没开过或
// 已经被另一侧关过了")什么都不做。用来保证 listen_fd_/wake_fd_/wake_wr_
// 只会被实际 close() 一次,即便 start()/stop() 被反复调用。
int take_and_close(std::atomic<int>& fd) {
  const int v = fd.exchange(-1);
  if (v >= 0) {
    ::close(v);
  }
  return v;
}
}  // namespace

LocalReserver::~LocalReserver() { stop(); }

bool LocalReserver::start(int port, const std::string& host) {
  if (running_.exchange(true)) {
    return false;  // 已在运行,重复 start() 视为失败,不动现有状态
  }

  // 上一次 start() 可能是自行退出(线程内部检测到不可恢复的错误后只置
  // running_=false 就返回,没有人调用过 stop() 来做清理——见头文件的 fd
  // 归属注释),此时 thread_ 仍然 joinable 且 listen_fd_/wake_fd_/
  // wake_wr_/clients_/to_drop_ 都可能有残留。直接把新线程赋给 thread_
  // 会在一个 joinable 的 std::thread 上触发 std::terminate();残留的 fd
  // 不清理则会一次次泄漏。这里统一收尾。
  if (thread_.joinable()) {
    thread_.join();
  }
  take_and_close(listen_fd_);
  take_and_close(wake_fd_);
  take_and_close(wake_wr_);
  {
    std::lock_guard<std::mutex> lock(m_);
    for (const int c : clients_) ::close(c);
    clients_.clear();
    for (const int d : to_drop_) ::close(d);
    to_drop_.clear();
  }

  // 自管道(self-pipe)当"唤醒 fd":stop() 或 broadcast() 往写端写一个
  // 字节,任何阻塞在 poll(wake_fd_, ...) 上的地方都会立刻被唤醒。两端都
  // 设非阻塞,避免管道满了的极端情况下写者卡在 write() 里。
  int fds[2] = {-1, -1};
  if (::pipe2(fds, O_NONBLOCK | O_CLOEXEC) != 0) {
    // fd 耗尽(EMFILE 等)时绝不能带着一个造不出唤醒通道的对象去启动线程:
    // accept 循环会阻塞在 poll(listen_fd, -1, ...) 上且没有任何唤醒来源,
    // stop()/析构里的 join() 会永久挂起。启动失败,报告给调用方,不留后台
    // 线程。
    running_.store(false);
    return false;
  }
  wake_fd_.store(fds[0]);
  wake_wr_.store(fds[1]);

  // 监听 socket 必须非阻塞:poll() 报告可读之后、accept() 真正被调用之前,
  // 对端完全可能已经把连接撤回(SYN 之后马上一个 RST)。socket 是阻塞的话
  // accept() 会在一个已经空了的队列上挂住,wake_fd 只在 poll() 里被监听,
  // 对陷在 accept() 里的线程无能为力,stop() 会永久挂起。
  const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
  if (fd < 0) {
    running_.store(false);
    take_and_close(wake_fd_);
    take_and_close(wake_wr_);
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
    running_.store(false);
    take_and_close(wake_fd_);
    take_and_close(wake_wr_);
    return false;
  }

  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    ::close(fd);
    running_.store(false);
    take_and_close(wake_fd_);
    take_and_close(wake_wr_);
    return false;
  }
  if (::listen(fd, 4) != 0) {
    ::close(fd);
    running_.store(false);
    take_and_close(wake_fd_);
    take_and_close(wake_wr_);
    return false;
  }

  sockaddr_in actual{};
  socklen_t len = sizeof(actual);
  ::getsockname(fd, reinterpret_cast<sockaddr*>(&actual), &len);
  bound_port_.store(::ntohs(actual.sin_port));

  listen_fd_.store(fd);
  thread_ = std::thread([this] { accept_loop(); });
  return true;
}

void LocalReserver::stop() {
  // 无条件走完整个流程,不根据 running_ 的旧值提前返回:accept_loop 可能
  // 已经自行把 running_ 置成了 false(见头文件的 fd 归属注释),如果这里
  // 因此提前 return,thread_ 就永远不会被 join()——析构函数里
  // std::thread 的析构器在一个仍然 joinable 的线程上跑会直接
  // std::terminate() 干掉整个进程(fix round 1 的 Critical 2)。
  // running_.store()/后面的每一步都用 exchange(-1)/joinable() 检查保证
  // 幂等,重复调用 stop() 依然安全。
  running_.store(false);

  // fix round 2:唤醒字节的 write() 挪到和 broadcast() 相同的锁下面做
  // ——broadcast() 现在也在持有 m_ 的情况下才 write() wake_wr_(见
  // broadcast() 里的注释),两边用同一把锁互斥,就不会有 stop() 正在
  // close() wake_wr_、broadcast() 又同时往(已经或即将被关闭/复用的)
  // 那个 fd 编号里写字节的情况。
  {
    std::lock_guard<std::mutex> lock(m_);
    const int wr = wake_wr_.load();
    if (wr >= 0) {
      const uint8_t one = 1;
      const ssize_t written = ::write(wr, &one, sizeof(one));
      (void)written;  // 唤醒信号,写失败也无妨(线程可能已经在退出路上)
    }
  }
  if (thread_.joinable()) {
    thread_.join();
  }

  // 关闭监听 socket 是这里的关键动作:必须真正释放端口,而不是只是不再
  // accept——StopReleasesThePortAndIsIdempotent 要求 stop() 之后一个全新的
  // 实例能立刻绑定同一个端口,SO_REUSEADDR 不能替代真正 close() 掉旧的
  // 监听 fd。这三个 fd 从始至终只由 stop() 关闭,accept_loop 自己退出时
  // 不会碰它们,所以这里不会和 accept 线程竞争同一次 close()。listen_fd_/
  // wake_fd_(读端)broadcast() 从不触碰,不需要额外持锁;wake_wr_
  // (写端)broadcast() 会写,所以它的 close() 也放进同一把锁里,和上面
  // 那次持锁的 write() 对称。
  take_and_close(listen_fd_);
  take_and_close(wake_fd_);
  {
    std::lock_guard<std::mutex> lock(m_);
    take_and_close(wake_wr_);
  }
  bound_port_.store(-1);

  // accept_loop 已经 join() 完毕,不会再有人往 clients_/to_drop_ 里
  // 增删,这里把剩下的都关掉是安全的。
  std::vector<int> to_close;
  {
    std::lock_guard<std::mutex> lock(m_);
    to_close.swap(clients_);
    to_close.insert(to_close.end(), to_drop_.begin(), to_drop_.end());
    to_drop_.clear();
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
  // fix round 1 追加修复(ThreadSanitizer 在 60 次 shuffled 跑里稳定复现出
  // 的真实竞态,大约 8/30 次命中):只把"摘除 fd"这个列表操作放进
  // std::lock_guard 是不够的。accept_loop 会独立地(靠自己的 recv()==0
  // 检测)在持锁的情况下直接 close() 一个死连接的 fd;如果 broadcast()
  // 只在取快照、和最后摘除列表两处短暂持锁,中间那段真正调用 send() 的
  // 窗口是不持锁的——accept_loop 完全可能在这段窗口里,基于它自己独立
  // 观察到的 POLLHUP/recv()==0,把同一个 fd 编号 close() 掉。
  // send()/close() 在没有同步的情况下并发作用于同一个 fd 编号,是
  // use-after-close/fd 编号被提前复用的经典根源——即便"谁来 close()"
  // 仍然只有 accept_loop 一个线程,`send()` 和 `close()` 本身仍需要互斥。
  //
  // 修法:把遍历 clients_、调用 send()、判定死活、从 clients_ 摘除这几步
  // 全部放进同一次 std::lock_guard 里。send() 是非阻塞 socket 上的
  // 非阻塞调用,持锁时长有界(不会退化成"阻塞发布者线程"),但这样一来,
  // accept_loop 那边同样在持锁状态下做的 close()(见下面 accept_loop 里
  // "dead" 处理块的注释)就不可能和这里的 send() 交叠——两段临界区互斥,
  // 竞态消失。经 ThreadSanitizer(30 次 shuffled 跑,ASLR 关闭以绕开这台
  // 沙箱环境下 TSan 自身的 shadow-memory 映射问题)验证不再报告任何
  // data race。
  //
  // 丢弃策略不变:send() 对写不动的对端(内核发送缓冲已满,即 rtkrcv 侧
  // 迟迟不读)返回 EAGAIN/EWOULDBLOCK,短写同样视为"写不动"——不重试、
  // 不排队缓冲字节,立刻摘除该客户端,防止一个卡住的客户端让内核发送
  // 缓冲无界增长。EINTR 例外:没有任何字节被发送,原地重试而不是当成
  // 失败——否则会悄悄丢掉这一整块数据,对端收到的流会有一个洞,而连接
  // 本身还活着、不会触发任何重连/重新同步。
  //
  // 真正的 close() 仍然只由 accept_loop 做(见头文件的 fd 归属注释):
  // 这里只把失败的 fd 从 clients_ 挪到 to_drop_,再唤醒 accept 线程去
  // close()——保证每个客户端 fd 只有一个线程会调用 close()。
  // fix round 2 追加修复(Important,新引入的问题):唤醒字节的 write()
  // 原来在锁外面做,而 stop() 会在 join() 之后(不持锁)close() 掉
  // wake_wr_——这一读一关没有任何同步,broadcast()(运行在 ROS 订阅
  // 回调线程上,节点关闭时随时可能被调用)可能正好在 stop() close()
  // 掉 wake_wr_ 的同一时刻往它里面写,把一个字节写进一个刚被内核复用给
  // 别的连接的 fd 编号——性质和 Important 3 一样,只是配对从
  // "stop() vs worker" 换成了实际部署里真会发生的"broadcast() vs
  // stop()"(ROS 回调线程在节点关闭过程中送来一批差分数据)。
  // 修法:把这次 write() 挪进和上面摘除逻辑同一把锁里,并让 stop() 也在
  // 持有同一把锁的情况下才去 close() wake_wr_——两边永远不会交叠。
  bool should_wake = false;
  {
    std::lock_guard<std::mutex> lock(m_);
    for (auto it = clients_.begin(); it != clients_.end();) {
      const int c = *it;
      ssize_t n;
      for (;;) {
        n = ::send(c, data, len, MSG_NOSIGNAL);
        if (n < 0 && errno == EINTR) continue;  // 没有字节发出去,原地重试
        break;
      }
      bool ok = n >= 0 && static_cast<size_t>(n) == len;
      if (!ok) {
        // EAGAIN/EWOULDBLOCK、其它硬错误,或者非阻塞 socket 上的短写
        // (同样视为"写不动"——不去补发剩下的字节,补发需要缓冲这批
        // 未发完的数据,和"不排队缓冲"的策略矛盾,直接丢弃,让对端看到
        // 一次干净的断开,而不是一条被截断又拼错的流)。
        it = clients_.erase(it);
        to_drop_.push_back(c);
        should_wake = true;
      } else {
        ++it;
      }
    }
    if (should_wake) {
      const int wr = wake_wr_.load();
      if (wr >= 0) {
        const uint8_t one = 1;
        const ssize_t written = ::write(wr, &one, sizeof(one));
        (void)written;  // 唤醒信号,写失败也无妨(accept 线程可能已经在退出)
      }
    }
  }
}

void LocalReserver::accept_loop() {
  uint8_t discard[256];
  uint8_t wake_buf[64];

  // accept() 遇到不会自愈的错误(EMFILE/ENFILE/ENOBUFS/EPERM 等)时用来
  // 停顿一下再继续的等待:同样通过 poll(wake_fd_) 等,这样 stop() 依然能
  // 立刻打断这段等待,而不是真的 sleep()。
  auto wait_a_bit = [this](double seconds) {
    pollfd pfd{wake_fd_.load(), POLLIN, 0};
    const int timeout_ms = static_cast<int>(seconds * 1000.0);
    ::poll(&pfd, 1, timeout_ms);
  };

  while (running_.load()) {
    std::vector<int> snapshot;
    {
      std::lock_guard<std::mutex> lock(m_);
      snapshot = clients_;
    }

    const int lfd = listen_fd_.load();
    const int wfd = wake_fd_.load();

    std::vector<pollfd> fds;
    fds.reserve(snapshot.size() + 2);
    fds.push_back({lfd, POLLIN, 0});
    fds.push_back({wfd, POLLIN, 0});
    for (const int c : snapshot) {
      fds.push_back({c, POLLIN, 0});
    }

    const int pr = ::poll(fds.data(), fds.size(), -1);
    if (pr < 0) {
      if (errno == EINTR) continue;
      // poll() 本身出错不是能自愈的瞬时状况,继续在这里 poll 只会原地
      // 打转。退出线程,不碰 listen_fd_/wake_fd_/wake_wr_——清理交给
      // 迟早会被调用的 stop()(见头文件的 fd 归属注释)。
      running_.store(false);
      break;
    }

    if (fds[1].revents & POLLIN) {
      // 唤醒字节有两种来源:stop()(附带 running_=false)或 broadcast()
      // 通知"to_drop_ 里有等着被关闭的死连接"。先把管道里排队的所有字节
      // 读空(可能被写了不止一次),再看 running_ 判断这次到底是哪一种;
      // 不是 stop() 的话就往下走,处理完 to_drop_ 后继续循环,不退出。
      for (;;) {
        const ssize_t n = ::read(wfd, wake_buf, sizeof(wake_buf));
        if (n <= 0) break;
      }
      if (!running_.load()) break;  // stop() 唤醒,结束线程
    }

    if (fds[0].revents & (POLLERR | POLLNVAL)) {
      // 监听 socket 本身坏掉了(不是"有连接进来"那种正常可读),继续 poll
      // 只会在这里原地打转——同样是"绝不能热循环"的情形。退出线程,不碰
      // 那三个共享 fd,清理交给 stop()。
      running_.store(false);
      break;
    }
    if (fds[0].revents & POLLIN) {
      // accept() 结果按 errno 分类:EAGAIN/EWOULDBLOCK 是队列已空(poll
      // 报告可读之后对端已经用 RST 撤回连接的瞬时状况),ECONNABORTED/
      // EINTR 同样是瞬时状况,直接继续排空队列。其它 errno
      // (EMFILE/ENFILE/ENOBUFS/EPERM 等)不会自愈——`if (fd < 0) continue;`
      // 这种写法在 EMFILE 下会产生验证过的 100% CPU 忙转圈:监听 socket
      // 上那个撤不掉的连接会让 poll 立刻再次报告 POLLIN。这里改为停顿
      // 一小段(仍然响应 wake_fd)再回到外层 poll,不在本轮里重试 accept()。
      for (;;) {
        const int c = ::accept4(lfd, nullptr, nullptr, SOCK_NONBLOCK);
        if (c < 0) {
          if (errno == EAGAIN || errno == EWOULDBLOCK) break;
          if (errno == ECONNABORTED || errno == EINTR) continue;
          wait_a_bit(0.1);
          break;
        }
        std::lock_guard<std::mutex> lock(m_);
        clients_.push_back(c);
      }
    }

    // 处理 broadcast() 摘下来、等着被真正 close() 的死连接——全程只有
    // accept 线程会 close() 客户端 fd(见头文件的 fd 归属注释),
    // broadcast() 只挪列表、不 close(),避免两个线程各自 close() 同一个
    // fd 的竞态。同时顺手拍一张 to_drop_ 剩余内容(如果这次 swap 之后、
    // 下面的 recv() 扫描之前,broadcast() 又并发塞了新条目进来)的快照,
    // 给下面的扫描当"这个 fd 已经被别人认领,别再动它"的参考——这只是
    // 优化/减少无谓的 recv() 调用,真正防止 double close() 的是扫描末尾
    // "先确认还在 clients_ 里,再 close()"这一步(见下面注释)。
    std::vector<int> to_close;
    {
      std::lock_guard<std::mutex> lock(m_);
      to_close.swap(to_drop_);
    }
    for (const int d : to_close) {
      ::close(d);
    }
    std::vector<int> pending_drop;
    {
      std::lock_guard<std::mutex> lock(m_);
      pending_drop = to_drop_;  // 非破坏性地看一眼此刻还在排队等 close() 的
    }

    // fix round 2 修的 Critical:客户端不会主动发数据,但仍然要把到来的
    // 字节 recv() 丢掉,否则对端一旦真的写了点什么就会卡在内核发送缓冲里
    // 出不来。同时用 recv()==0 或错误来检测"对端已断开"。
    //
    // 这里曾经有个 double close() 的洞:recv() 扫描用的 `fds` 数组是本轮
    // 循环开始时拍的快照,如果 broadcast()(另一个线程)在这之后、这次
    // 扫描之前,凭它自己的 EAGAIN 检测已经把同一个 fd 摘进了 to_drop_
    // (但还没被上面的 to_close 处理关掉——因为它是在 to_close.swap() 之
    // 后才被塞进去的),而这里的 recv() 扫描又凭 POLLHUP/recv()==0 独立
    // 判定同一个 fd 已死,原来的代码会在这里无条件 close() 一次;那个 fd
    // 仍然留在 to_drop_ 里,之后会被下一轮/被 stop() 再 close() 一次,
    // 期间那个 fd 编号很可能已经被 accept4() 分给了全新的客户端——这就是
    // review 抓到的"stale to_drop_ 条目关掉一个刚复用的活连接"。
    //
    // 修法:close() 之前先在 clients_ 里确认这个 fd 确实还在(under lock)、
    // 并且是"这一次调用"把它从 clients_ 里摘下来的——摘成功了才 close()。
    // 摘不到,说明 broadcast() 已经先一步摘掉它、放进了 to_drop_(不管
    // 关没关),交给那条路径去关,这里绝不重复 close()。这样一来,不变式
    // 成立:每个客户端 fd 只会被"第一个成功把它从 clients_ 里 erase 掉的
    // 那次操作"close() 恰好一次,别的路径发现 erase 不到就什么都不做。
    std::vector<int> dead;
    for (size_t i = 2; i < fds.size(); ++i) {
      const int c = fds[i].fd;
      if (std::find(to_close.begin(), to_close.end(), c) != to_close.end()) continue;
      if (std::find(pending_drop.begin(), pending_drop.end(), c) != pending_drop.end()) continue;
      if (!(fds[i].revents & (POLLIN | POLLHUP | POLLERR))) continue;
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
        bool found = false;
        for (auto it = clients_.begin(); it != clients_.end(); ++it) {
          if (*it == d) {
            clients_.erase(it);
            found = true;
            break;
          }
        }
        if (found) {
          ::close(d);
        }
        // 没找到:d 已经不在 clients_ 里了——上面的 pending_drop 快照有
        // 时间窗口(它是在 recv() 扫描"之前"拍的,broadcast() 仍可能在
        // 扫描进行期间才把 d 塞进 to_drop_),所以这个兜底检查不能少。
        // 找不到就意味着 broadcast() 已经先一步摘除并接管了它,这里绝不
        // 重复 close(),避免对一个可能已经关闭、fd 编号可能已经复用给
        // 别的连接的值再次 close()。
      }
    }
  }
  // 不在这里关闭 listen_fd_/wake_fd_/wake_wr_,也不清空遗留的
  // clients_/to_drop_——无论是被 stop() 正常唤醒退出,还是自己检测到
  // 不可恢复错误退出,清理统一交给 stop()(析构函数保证最终会调用到),
  // 避免这个线程和调用 stop() 的线程互相 close() 对方正在用的同一个 fd
  // (fix round 1 的 Critical 1/2 都是这一类竞态)。
}

}  // namespace gnss_bringup
