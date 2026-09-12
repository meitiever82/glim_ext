#include "gnss_bringup/process_supervisor.hpp"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>

namespace gnss_bringup {

ProcessSupervisor::ProcessSupervisor(ProcessSupervisorConfig cfg) : cfg_(std::move(cfg)) {}

ProcessSupervisor::~ProcessSupervisor() { stop(); }

void ProcessSupervisor::start() {
  if (running_.exchange(true)) {
    return;  // 已经在跑,重复 start() 什么都不做,不去构造第二个 std::thread
  }

  // 上一轮通常是 stop() 正常收尾过的,thread_ 应该已经不可 join;这里仍然
  // 兜底一下——不这样做的话,在一个仍然 joinable 的 std::thread 成员上
  // 直接赋新线程会触发 std::terminate()。
  if (thread_.joinable()) {
    thread_.join();
  }

  int fds[2] = {-1, -1};
  if (::pipe2(fds, O_NONBLOCK | O_CLOEXEC) != 0) {
    // 造不出唤醒管道,就不能起一个 stop() 打断不了的后台线程:run() 的
    // 退避睡眠靠 poll(wake_fd_, ...) 才能被 stop() 立刻打断,没有这根管子
    // 就只能退化成真正的 sleep(),stop()/析构里的 join() 可能因此长时间
    // 挂起。宁可 start() 失败,也不留一个不可控的线程在后台。
    running_.store(false);
    return;
  }
  wake_fd_ = fds[0];
  wake_wr_ = fds[1];
  current_delay_.store(0.0);

  thread_ = std::thread([this] { run(); });
}

void ProcessSupervisor::stop() {
  running_.store(false);

  // 叫醒可能正阻塞在退避 poll() 里的 run() 线程,让它立刻回到循环顶部
  // 重新检查 running_。
  if (wake_wr_ >= 0) {
    const uint8_t one = 1;
    const ssize_t written = ::write(wake_wr_, &one, sizeof(one));
    (void)written;  // 只是个唤醒信号,写失败也无妨(线程可能已经在退出路上)
  }

  // 对进程组发信号,直到 run() 线程把 child_pid_ 收回成 -1(说明它已经
  // waitpid() 把子进程回收了)为止;超过 5 s 还没死就升级成 SIGKILL。
  // 每一轮都重新读取 child_pid_,而不是只在进入 stop() 时读一次缓存:
  // run() 完全可能在这期间已经因为子进程自然退出而重启出下一个子进程,
  // 如果只信号"进来时那一个 pid",新起的这个就会被漏掉、白白多活一轮。
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  for (;;) {
    const int pid = child_pid_.load();
    if (pid <= 0) break;  // 已经被 run() 回收,或者从未 start() 过
    const bool escalate = std::chrono::steady_clock::now() >= deadline;
    const int sig = escalate ? SIGKILL : SIGTERM;
    // 对整个进程组(-pid)发信号,而不是只发给直接子进程本身:rtkrcv 之类
    // 的目标程序会派生孙进程,只杀直接子进程会把孙进程留成孤儿。
    if (::kill(-pid, sig) != 0 && errno != ESRCH) {
      // ESRCH(进程组已经不存在,子进程恰好在这一刻自己退出/被回收)是
      // 预期情况,忽略;其它 errno 这里也没有更好的补救办法,下一轮循环
      // 会用最新的 child_pid_ 重试。
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // 不管上面这段走到哪一步,只要线程 joinable 就必须 join——绝不能用
  // running_ 之类的旗标去决定要不要 join。run() 因为子进程自己退出、
  // fork() 失败反复重试等原因,可能已经在跑到某处时自己观察到
  // running_==false 并返回,把线程变成"已经跑完但还没被 join"的状态,
  // 这时候如果这里因为某个旗标提前 return,thread_ 就永远不会被
  // join()——析构函数里 std::thread 的析构器在一个仍然 joinable 的线程
  // 上运行会直接 std::terminate() 干掉整个进程。
  if (thread_.joinable()) {
    thread_.join();
  }

  // 线程已经 join 完毕,不会再有任何人碰这两个 fd,这里关闭是安全的。
  if (wake_wr_ >= 0) {
    ::close(wake_wr_);
    wake_wr_ = -1;
  }
  if (wake_fd_ >= 0) {
    ::close(wake_fd_);
    wake_fd_ = -1;
  }
}

void ProcessSupervisor::run() {
  // 退避/重试之间的可打断等待:poll(wake_fd_, ...) 而不是 sleep(),这样
  // stop() 往 wake_wr_ 里写一个字节就能立刻结束等待,不用真的等满
  // `seconds`。poll() 本身被信号打断(EINTR)时重试,不当成错误;万一
  // poll() 真的出了不可恢复的错误,也只是提前结束这一次等待、回到外层
  // while(running_) 循环,不会让线程在没有清空 running_ 的情况下退出
  // ——保证不会出现"对象自认为还在跑,但线程已经死了"的状态。
  auto interruptible_wait = [this](double seconds) {
    const auto deadline = std::chrono::steady_clock::now() +
                           std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                               std::chrono::duration<double>(seconds));
    while (running_.load()) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) break;
      const double remaining_s = std::chrono::duration<double>(deadline - now).count();
      int timeout_ms = static_cast<int>(remaining_s * 1000.0);
      if (timeout_ms < 0) timeout_ms = 0;

      pollfd pfd{wake_fd_, POLLIN, 0};
      const int pr = ::poll(&pfd, 1, timeout_ms);
      if (pr < 0) {
        if (errno == EINTR) continue;
        break;  // 不可恢复的 poll 错误:提前结束这次等待,外层循环继续正常工作
      }
      if (pr > 0 && (pfd.revents & POLLIN)) {
        // 排空唤醒管道里的字节。是不是 stop() 发的唤醒,由循环顶部的
        // running_ 检查来判断,这里不需要关心。
        uint8_t buf[64];
        while (::read(wake_fd_, buf, sizeof(buf)) > 0) {
        }
      }
    }
  };

  while (running_.load()) {
    const pid_t pid = ::fork();
    if (pid < 0) {
      // fork() 失败(常见于 EAGAIN/ENOMEM,进程数或内存吃紧):根本没有
      // 子进程活下来,不计入 spawn_count_。稍等一下再试,同时仍然响应
      // stop()。
      interruptible_wait(cfg_.restart_delay_s > 0.0 ? cfg_.restart_delay_s : 1.0);
      continue;
    }

    if (pid == 0) {
      // ---- 子进程 ----
      // 独立进程组:目标程序(Task 7 里是 rtkrcv)可能派生孙进程,stop()
      // 需要能对整组发信号才不会留下孤儿。setsid() 失败(比如已经是
      // session leader)不影响后续流程,忽略返回值。
      ::setsid();
      if (!cfg_.cwd.empty() && ::chdir(cfg_.cwd.c_str()) != 0) {
        _exit(127);  // 用 _exit 而不是 exit:fork 出来的子进程不能重复跑
                      // 一遍父进程的 atexit/全局对象析构。
      }

      std::vector<std::string> arg_storage;
      arg_storage.reserve(cfg_.args.size() + 1);
      arg_storage.push_back(cfg_.binary);
      for (const auto& a : cfg_.args) arg_storage.push_back(a);

      std::vector<char*> argv;
      argv.reserve(arg_storage.size() + 1);
      for (auto& a : arg_storage) argv.push_back(const_cast<char*>(a.c_str()));
      argv.push_back(nullptr);

      ::execv(cfg_.binary.c_str(), argv.data());
      // execv 只有失败才会返回(二进制不存在/没有执行权限等)。同样用
      // _exit 而不是 exit,并且退出码固定用 127(约定俗成的"命令不存在/
      // 不可执行"),父进程会把这次短命当成崩溃循环处理并退避,不会崩溃
      // 或卡死。
      _exit(127);
    }

    // ---- 父进程 ----
    child_pid_.store(pid);
    spawn_count_.fetch_add(1);
    const auto t0 = std::chrono::steady_clock::now();

    // 阻塞等子进程退出。这里没有再套一层 poll(wake_fd_, ...):子进程的
    // 死亡本身就是这里唯一的解锁条件,而 stop() 正是通过对进程组发
    // SIGTERM/SIGKILL 来主动促成这次死亡的(见 stop() 的实现),所以
    // waitpid() 总会在有限时间内返回,不需要额外的唤醒机制。EINTR
    // (被信号打断)重试而不是当成失败——否则会把一次正常的子进程退出
    // 误判掉,导致这个子进程永远不被回收,变成僵尸。
    int status = 0;
    for (;;) {
      const pid_t w = ::waitpid(pid, &status, 0);
      if (w == pid) break;
      if (w < 0 && errno == EINTR) continue;
      break;  // 理论上不会走到(比如 ECHILD):当作已经结束处理,避免线程卡死
    }
    child_pid_.store(-1);  // 已回收:stop() 靠这个字段判断还需不需要继续发信号

    const double lifetime_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (lifetime_s < cfg_.crash_loop_life_s) {
      // 崩溃循环(坏二进制/坏 conf 导致子进程刚起来就死):重启间隔翻倍,
      // 直到 max_restart_delay_s 封顶,避免一天刷出几万条 connected/
      // disconnected。
      const double prev = current_delay_.load();
      current_delay_.store(prev <= 0.0 ? cfg_.restart_delay_s
                                        : std::min(prev * 2.0, cfg_.max_restart_delay_s));
    } else {
      // 活得够久,说明这次不是崩溃循环,重启间隔复位成正常值。
      current_delay_.store(cfg_.restart_delay_s);
    }

    if (!running_.load()) break;  // stop() 正在等这次退出收尾,不再重启
    interruptible_wait(current_delay_.load());
  }
}

}  // namespace gnss_bringup
