#include "gnss_bringup/process_supervisor.hpp"

#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>

namespace gnss_bringup {

namespace {
// 给一个子进程发信号:直接对 pid 本身发一次,再对它所在的整个进程组
// (-pid)发一次,合起来只算"发一次"这个信号级别。
//
// review round 2:为什么两次都要发、但都只发一次——
//   - kill(pid, sig) 直接找 pid 本身:从 fork() 成功那一刻起 pid 就是
//     有效目标,不依赖进程组是否已经建立(子进程调用 setsid() 生效
//     之前,父进程这边可能已经开始发信号了,谁先谁后没有保证),保证
//     这一次尝试一定有真实进程可以收到,不需要重试。
//   - kill(-pid, sig) 找进程组:覆盖 rtkrcv 这类会派生孙进程的目标程序。
//     这次如果 setsid() 还没跑完,可能会因为进程组尚不存在而扑空
//     (ESRCH)——但那意味着子进程才刚 fork 出来,不可能已经有孙进程,
//     扑空没有任何实际影响,不需要因此重试。
//   - 早期版本靠"每 20ms 无条件重发同一个信号"来规避上面这个 setsid()
//     竞态,结果是一个愿意在退出前 flush 数据的目标程序(rtkrcv 正是
//     如此)在等待期里会收到几十上百次同一个信号,处理函数不断被新到
//     的信号打断(EINTR),反而可能永远做不完那次收尾——完全违背了
//     "先发 SIGTERM 给它一个机会,再等一等才 SIGKILL"这个设计本身的
//     用意。改成直接对 pid 发送之后,不再需要重发就能保证送达,一个
//     信号级别只需要真正调用一次。
void signal_child(pid_t pid, int sig) {
  if (::kill(pid, sig) != 0 && errno != ESRCH) {
    // ESRCH(进程已经不存在,自己退出/被回收了)是预期情况;其它 errno
    // 这里也没有更好的处理方式。
  }
  if (::kill(-pid, sig) != 0 && errno != ESRCH) {
    // 同上,针对进程组的那一次。
  }
}
}  // namespace

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

  // 对子进程发信号,直到 run() 线程把 child_pid_ 收回成 -1(说明它已经
  // waitpid() 把子进程回收了)为止。每一轮都重新读取 child_pid_,而不是
  // 只在进入 stop() 时读一次缓存:run() 完全可能在这期间已经因为子进程
  // 自然退出而重启出下一个子进程,如果只信号"进来时那一个 pid",新起的
  // 这个就会被漏掉、白白多活一轮——一旦发现 pid 变了(新的子进程),就把
  // "这个信号级别发过没有"的状态和升级时限一起重置,当成一个全新的目标
  // 从头处理。
  //
  // review round 2:每个信号级别(SIGTERM / 升级后的 SIGKILL)只真正调用
  // 一次 signal_child(),不再是"每 20ms 无条件重发"——原因见 signal_child()
  // 上面的注释。这里仍然按 20ms 一轮轮询 child_pid_ 是否已经被回收,只是
  // 轮询不等于重发。
  //
  // review round 1 的 Important:这个循环原来唯一的退出条件是
  // child_pid_<=0,如果子进程卡在不可中断的 D 状态(挂死的 I/O),或者
  // kill() 一直失败(比如 EPERM),就会永远每 20 ms 转一圈、永不退出。
  // 这里加一个独立的绝对放弃时限——超过之后不再继续在这里轮询,直接
  // 往下走到 join()。放弃并不代表子进程杀不死:run() 线程自己在竞态窗口
  // 被触发时也会走同一套发信号/升级逻辑(见下面 run() 里的说明),这里
  // 放弃只是不想让 stop() 这个循环自己也跟着永远转下去;真正卡在内核
  // D 状态的极端情况,任何用户态代码都无能为力,join() 之后会一直阻塞到
  // 子进程真的退出为止。
  const auto abort_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  int last_pid_seen = -1;
  bool sigterm_sent = false;
  bool sigkill_sent = false;
  std::chrono::steady_clock::time_point escalate_deadline{};
  for (;;) {
    const int pid = child_pid_.load();
    if (pid <= 0) break;  // 已经被 run() 回收,或者从未 start() 过
    if (pid != last_pid_seen) {
      last_pid_seen = pid;
      sigterm_sent = false;
      sigkill_sent = false;
      escalate_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= abort_deadline) break;
    if (!sigterm_sent) {
      sigterm_sent = true;
      signal_child(pid, SIGTERM);
    } else if (!sigkill_sent && now >= escalate_deadline) {
      sigkill_sent = true;
      signal_child(pid, SIGKILL);
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
  // 崩溃循环退避参数在进入循环前做一次合法性校验,避免配置失误(0、负数、
  // restart_delay_s 比 max_restart_delay_s 还大)把行为搞坏:
  //   - restart_delay_s<=0(含 0/负数):钉死在 0 会让崩溃循环的第一次翻倍
  //     永远还是 0,变成一个真正的热重启循环;退化成一个很小的正数下限。
  //   - max_restart_delay_s<=0:上限本身不合法,退化成不封顶(等于基准值,
  //     不再继续增长,但至少不会钉在 0)。
  //   - restart_delay_s > max_restart_delay_s:基准本身就已经超过上限,
  //     直接钉到上限,不然从第一次崩溃开始就已经违反"不得超过上限"。
  constexpr double kMinDelaySeconds = 0.001;
  double base_delay = cfg_.restart_delay_s > 0.0 ? cfg_.restart_delay_s : kMinDelaySeconds;
  double max_delay = cfg_.max_restart_delay_s > 0.0 ? cfg_.max_restart_delay_s : base_delay;
  if (base_delay > max_delay) base_delay = max_delay;

  // 退避/重试之间的可打断等待:poll(wake_fd_, ...) 而不是 sleep(),这样
  // stop() 往 wake_wr_ 里写一个字节就能立刻结束等待,不用真的等满
  // `seconds`。poll() 被信号打断(EINTR)时重试,不当成错误。
  //
  // review round 1 的 Important(晋升自 Minor):poll() 真的遇到不可恢复
  // 的错误时,原来的写法是直接 break,把这次等待当成"已经等完了"——那样
  // 一来外层 while 循环会立刻再 fork() 一次,变成没有任何延迟的热重启
  // 循环,这个包里已经因为同一类"系统调用出错就当成瞬间成功"的写法在别处
  // (accept 循环的 EMFILE 场景)踩过一次验证过的 100% CPU 忙转的坑。这里
  // 改成退化成一段有界的真 sleep(),之后回到循环顶部重新检查 running_、
  // 重新尝试 poll(),而不是提前结束整段等待。
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
        // 不可恢复的 poll 错误:退化成一段有界的 chunked sleep(至多
        // 100 ms,或者剩余时间,取较小值),而不是把这次等待当成已经结束
        // ——避免外层循环变成热 fork() 循环。
        std::this_thread::sleep_for(
            std::chrono::milliseconds(std::min(timeout_ms, 100)));
        continue;
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

  // fork()+exec() 需要的 argv 在进入循环前一次性建好,而不是每次重启都
  // 现建:cfg_ 从构造之后就不再变化,没必要重复分配。更重要的是异步信号
  // 安全——review round 1 的 Important:fork() 出来的子进程在 exec() 之前
  // 只有一个线程,但它是从一个多线程父进程的内存状态直接复制出来的,glibc
  // 的 malloc 内部锁完全可能在 fork() 那一刻正被父进程的另一个线程持有
  // (fork() 只复制内存,不会连带复制/释放锁的持有权)。子进程分支里如果
  // 再构造 std::vector<std::string> 之类会触发 malloc() 的对象,就有确定
  // 会在某些时候死锁在子进程里的风险——而且是那种既不崩溃也不重试的静默
  // 失败(父进程只会看到子进程一直不退出、也不真正跑起来)。这里把所有会
  // 分配内存的构建工作都挪到 fork() 之前的父进程里做一次,子进程分支只读
  // 已经建好的 argv,只调用 fork()/execv() 那张公认异步信号安全清单里的
  // 函数。
  std::vector<std::string> arg_storage;
  arg_storage.reserve(cfg_.args.size() + 1);
  arg_storage.push_back(cfg_.binary);
  for (const auto& a : cfg_.args) arg_storage.push_back(a);
  std::vector<char*> argv;
  argv.reserve(arg_storage.size() + 1);
  for (auto& a : arg_storage) argv.push_back(const_cast<char*>(a.c_str()));
  argv.push_back(nullptr);

  // 子进程关闭继承 fd 时,close_range() 万一在这台内核上不可用,退化到
  // 逐个 close() 的老办法要用到的上限——同样在 fork() 之前算好:
  // sysconf() 是否会分配内存不在异步信号安全的保证范围内,放在父进程这边
  // 调用没有任何顾虑。
  const long max_fd = ::sysconf(_SC_OPEN_MAX);

  while (running_.load()) {
    // review round 3 的 Important:子进程在把信号处置重置成 SIG_DFL 之前
    // (下面的 setsid() 到 sigaction 循环这段窗口),携带的仍然是从父进程
    // 这个线程继承来的信号处置——Task 7 的父进程是 rclcpp 节点,rclcpp
    // 自己会装一个 SIGTERM 处理器,这是常态而不是边缘情况。如果父进程在
    // 这段窗口里对刚 fork 出来的 pid 直接 kill(pid, SIGTERM)(stop() 的
    // signal_child() 正是这么做的,而且理由充分——见 signal_child() 上面
    // 的注释),子进程会用继承来的、和目标程序完全不相关的处理器把这个
    // SIGTERM "处理"掉(处理器返回,进程继续往下跑),既没有终止子进程,
    // 也没有把这个关闭请求转交给稍后才会 execv() 进来的目标程序——真实
    // strace 复现:kill(pid,SIGTERM) 成功送达、被继承的处理器吞掉,随后
    // kill(-pid,SIGTERM) 因为 setsid() 还没跑完而 ESRCH,子进程继续跑到
    // sigaction(SIG_DFL) 才把处置重置对,但为时已晚,execv() 进去的目标
    // 程序从头到尾没见过这次 SIGTERM,只能傻等 5s 后的 SIGKILL——这正是
    // "先发 SIGTERM 给目标程序一个 flush 数据的机会"这个设计想避免的
    // 结果。
    //
    // 光把上面的 sigaction 循环挪到 setsid() 前面并不能解决问题:
    // kill(pid, ...) 完全可能在子进程被调度、执行任何一行代码之前就已经
    // 送达并被继承的处理器处理掉了——这是"谁先被调度"的竞态,不是"子进程
    // 代码里哪一步先做"的顺序问题。
    //
    // 真正的修法是在 fork() 之前,把这个线程自己的信号掩码整体设成"全部
    // 阻塞":子进程会原样继承这个"全部阻塞"的掩码,这段窗口期间任何发给
    // 它的信号都只会变成 pending,不会被继承来的处理器提前跑掉。子进程在
    // 完成 setsid()、把所有处置都重置成 SIG_DFL 之后,再解除阻塞(见下面
    // sigprocmask(SIG_SETMASK, empty) 那一步)——如果这期间确实有信号
    // 变成了 pending,这时候会在 SIG_DFL 下正确生效(SIGTERM 的默认动作
    // 是终止),子进程会干净地退出而不会带着"一个从没被正确处理过的关闭
    // 请求"糊里糊涂地继续跑到 execv()。父进程这边的阻塞只是这个线程自己
    // 的临时状态(pthread_sigmask 只影响调用线程),fork() 一返回就立刻
    // 恢复,不会泄漏到 run() 线程后续的正常工作里,也不影响进程里的其它
    // 线程。
    sigset_t block_all, saved_mask;
    ::sigfillset(&block_all);
    const bool masked =
        (::pthread_sigmask(SIG_SETMASK, &block_all, &saved_mask) == 0);

    const pid_t pid = ::fork();

    if (pid != 0 && masked) {
      // 父进程分支(fork 成功或失败都算):马上恢复这个线程自己原来的
      // 信号掩码,不能让"全部阻塞"这个临时状态泄漏出去。
      ::pthread_sigmask(SIG_SETMASK, &saved_mask, nullptr);
    }

    if (pid < 0) {
      // fork() 失败(常见于 EAGAIN/ENOMEM,进程数或内存吃紧):根本没有
      // 子进程活下来,不计入 spawn_count_。稍等一下再试,同时仍然响应
      // stop()。
      interruptible_wait(base_delay);
      continue;
    }

    if (pid == 0) {
      // ---- 子进程:自此只允许调用异步信号安全的函数(见上面 argv/
      // max_fd 为什么要挪到 fork() 之前算好的注释)。所有信号此刻仍然
      // 处于阻塞状态(继承自上面对父进程线程做的 pthread_sigmask),直到
      // 下面 sigprocmask(SIG_SETMASK, empty) 那一步才会解除——见本段
      // 开头的说明。----

      // 独立进程组:目标程序(Task 7 里是 rtkrcv)可能派生孙进程,stop()
      // 需要能对整组发信号才不会留下孤儿。setsid() 失败(比如已经是
      // session leader)不影响后续流程,忽略返回值。
      ::setsid();

      // review round 1 的 Important:execv() 会原样保留调用者的信号处置
      // (SIG_IGN 会被继承,只有 SIG_DFL 才会被 exec 重置)。这里显式把
      // 所有信号的处置重置成 SIG_DFL——sigaction()/sigemptyset()/
      // sigprocmask() 都是异步信号安全的,不涉及任何内存分配——保证子
      // 进程对信号的反应完全由它自己的默认行为决定,不受父进程当时那个
      // 线程的信号状态影响。
      //
      // review round 3:最后这一步 sigprocmask(SIG_SETMASK, empty) 现在
      // 身兼两职——解除阻塞,同时也是"补交"上面那段窗口期间可能已经变成
      // pending 的信号(比如提前送达的 SIGTERM)的地方。到这一行之前,
      // 所有处置都已经改成 SIG_DFL 了,所以这次解除阻塞不会再重蹈"用继承
      // 来的处理器把信号吞掉"的覆辙。
      {
        struct sigaction sa {};
        sa.sa_handler = SIG_DFL;
        ::sigemptyset(&sa.sa_mask);
        for (int sig = 1; sig < NSIG; ++sig) {
          ::sigaction(sig, &sa, nullptr);  // SIGKILL/SIGSTOP 等会失败,忽略即可
        }
        sigset_t empty_mask;
        ::sigemptyset(&empty_mask);
        ::sigprocmask(SIG_SETMASK, &empty_mask, nullptr);
      }

      // review round 1 的 Important:关掉继承下来的、0/1/2 之外的所有 fd。
      // 本类自己的唤醒管道已经是 O_CLOEXEC,但同一个进程里其它模块(比如
      // Task 7 会用到的 LocalReserver 的监听 socket)未必是——不清掉的话,
      // rtkrcv 或者它派生的孙进程会一直攥着那个 fd 的一份拷贝,父进程这边
      // close() 掉监听 socket 也不能真正释放端口,下次重新绑定会失败。
      //
      // review round 2 的 Important:第一版在这里逐个 fd 调 close(),循环
      // 上限是 sysconf(_SC_OPEN_MAX) 也就是 RLIMIT_NOFILE。这台机器上软
      // 限制是 1048576,实测这个循环要占子进程 ~95ms 的 CPU 时间(execv
      // 之前);容器环境里 RLIMIT_NOFILE 默认到 2^30 级别很常见,那样一次
      // spawn 会在 close() 里卡上几分钟,表现得像是卡死。而且 sysconf()
      // 失败时返回 -1,原来的写法完全没处理这种情况——`fd < max_fd` 因为
      // max_fd==-1 恒为假,一个 fd 都不会关,悄悄地把这一轮之前刚堵上的
      // fd 继承漏洞又重新打开了,不会有任何报错或提示。
      //
      // 换成优先用 close_range(3, ~0u, 0)(glibc 2.34+/内核 5.9+都支持):
      // 它直接对内核维护的 fd 表操作,耗时只取决于"真正打开了多少个 fd",
      // 和 RLIMIT_NOFILE 这个上限无关;close_range() 本身也在异步信号
      // 安全的清单里,fork()/execv() 之间调用没有问题。只有在它失败时
      // (比如运行在更老、没有这个系统调用的内核上,返回 ENOSYS)才退化到
      // 逐个 close() 的老办法,并且这次显式处理 sysconf() 返回 -1 的情况
      // ——用一个保守的兜底上限,而不是让循环因为上限是 -1 而直接跳过、
      // 什么都不关。
      if (::close_range(3, ~0u, 0) != 0) {
        long fallback_max_fd = max_fd;
        if (fallback_max_fd < 0) {
          fallback_max_fd = 65536;  // sysconf 说不清楚上限时的保守兜底
        }
        for (long fd = 3; fd < fallback_max_fd; ++fd) {
          ::close(fd);
        }
      }

      if (!cfg_.cwd.empty() && ::chdir(cfg_.cwd.c_str()) != 0) {
        _exit(127);  // 用 _exit 而不是 exit:fork 出来的子进程不能重复跑
                      // 一遍父进程的 atexit/全局对象析构。
      }

      ::execv(cfg_.binary.c_str(), argv.data());
      // execv 只有失败才会返回(二进制不存在/没有执行权限等)。同样用
      // _exit 而不是 exit,并且退出码固定用 127(约定俗成的"命令不存在/
      // 不可执行"),父进程会把这次短命当成崩溃循环处理并退避,不会崩溃
      // 或卡死。
      _exit(127);
    }

    // ---- 父进程 ----
    last_spawned_pid_.store(pid);
    child_pid_.store(pid);
    spawn_count_.fetch_add(1);
    const auto t0 = std::chrono::steady_clock::now();

    // review round 1 的 Critical:fork() 成功到上面 child_pid_.store(pid)
    // 之间存在一个真实的竞态窗口。如果 stop() 恰好在这段窗口里把它自己的
    // 整个 kill 循环跑完——它当时读到的 child_pid_ 还是 -1,判定"没有子
    // 进程要杀",直接去 thread_.join()——这次信号就永远发不出去了:子
    // 进程变成没人管的孤儿,而这个线程接下来会阻塞在等它退出上,没有人
    // 会再唤醒它,join() 因此永久卡死(已经用真实的调用栈复现:主线程卡
    // 在 pthread_join,工作线程卡在 waitpid,外加一个活着的孤儿子进程)。
    //
    // 这里只需要在这一个时间点做一次性检查,不需要之后每一轮 tick 都重复
    // 检查:running_.store(false) 是 stop() 的第一条语句,严格发生在它
    // 读取 child_pid_、调用 join() 之前;而这里的检查发生在
    // child_pid_.store(pid) 之后。如果这时候读到 running_ 已经是 false,
    // 说明 stop() 有可能已经(或即将)在错误的时机读到 child_pid_==-1,
    // 这个线程必须自己独立完成"发 SIGTERM → 超时 → SIGKILL"这一整套
    // 动作,不能指望 stop() 会来救。反过来,如果这时候 running_ 还是
    // true,那么在此之后任何时间点调用的 stop() 都一定能读到这里已经
    // 落盘的 child_pid_,会按正常路径处理——不需要这个线程再重复插一脚,
    // 否则就是 review round 2 指出的"两边都发,子进程收到两倍信号"。
    bool self_escalate = !running_.load();
    bool sigterm_sent = false;
    bool sigkill_sent = false;
    std::chrono::steady_clock::time_point escalate_deadline{};
    if (self_escalate) {
      escalate_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    }

    // 等子进程退出:非阻塞 waitpid(WNOHANG) + poll(wake_fd_, tick) 轮询,
    // 而不是一个不可打断的 waitpid(pid, &status, 0)——原来那个阻塞
    // waitpid() 本身就是一个没有轮询唤醒描述符的阻塞点,违反了这个包里
    // "每个阻塞点都要 poll 唤醒描述符"的既定规则。
    int status = 0;
    for (;;) {
      const pid_t w = ::waitpid(pid, &status, WNOHANG);
      if (w == pid) break;
      if (w < 0) {
        if (errno == EINTR) continue;
        break;  // 理论上不会走到(比如 ECHILD):当作已经结束处理,避免线程卡死
      }
      // w == 0:子进程还活着。只有在上面那次一次性检查判定需要"自救"时才
      // 会走到这里发信号;否则完全依赖 stop() 处理,这里只是被动等待,
      // 不会重复发送——这正是这一轮 review 要修的"两边都发,子进程收到
      // 两倍信号"。
      if (self_escalate) {
        const auto now = std::chrono::steady_clock::now();
        if (!sigterm_sent) {
          sigterm_sent = true;
          signal_child(pid, SIGTERM);
        } else if (!sigkill_sent && now >= escalate_deadline) {
          sigkill_sent = true;
          signal_child(pid, SIGKILL);
        }
      }

      // 轮询间隔:不需要发信号,不代表也不需要“反应快”——stop() 那边一旦
      // 把 running_ 置 false,这里应该尽快用更短的 tick 去发现子进程已经
      // 死了并回收它,不然默认的 200 ms tick 会让每一次 stop() 都平白多
      // 等上一小段时间(这里只是查一下 running_ 这个原子量,不发送任何
      // 信号,和上面"是否要自己发信号"是两件独立的事——揉在一起正是这一
      // 轮 review 要修的那个"重复发信号"问题的根源)。还没被要求停止时,
      // 轮询间隔放宽到 200 ms,减少无谓的系统调用。
      const bool winding_down = self_escalate || !running_.load();
      const int timeout_ms = winding_down ? 20 : 200;
      pollfd pfd{wake_fd_, POLLIN, 0};
      const int pr = ::poll(&pfd, 1, timeout_ms);
      if (pr < 0) {
        if (errno == EINTR) continue;
        // 不可恢复的 poll 错误:和 interruptible_wait() 里同一类问题一样,
        // 不能直接 continue——那样 waitpid(WNOHANG) + poll() 会在两个都
        // 出错的情况下背靠背地空转,变成一个热循环。退化成一段有界的真
        // sleep() 再回到循环顶部重新尝试。
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
        continue;
      }
      if (pr > 0 && (pfd.revents & POLLIN)) {
        uint8_t buf[64];
        while (::read(wake_fd_, buf, sizeof(buf)) > 0) {
        }
      }
    }
    child_pid_.store(-1);  // 已回收:stop() 靠这个字段判断还需不需要继续发信号

    const double lifetime_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (lifetime_s < cfg_.crash_loop_life_s) {
      // 崩溃循环(坏二进制/坏 conf 导致子进程刚起来就死):重启间隔翻倍,
      // 直到 max_delay 封顶,避免一天刷出几万条 connected/disconnected。
      const double prev = current_delay_.load();
      current_delay_.store(prev <= 0.0 ? base_delay : std::min(prev * 2.0, max_delay));
    } else {
      // 活得够久,说明这次不是崩溃循环,重启间隔复位成正常值。
      current_delay_.store(base_delay);
    }

    if (!running_.load()) break;  // stop() 正在等这次退出收尾,不再重启
    interruptible_wait(current_delay_.load());
  }
}

}  // namespace gnss_bringup
