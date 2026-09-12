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

namespace {
// 对整个进程组发信号,统一处理"进程组已经不存在"的预期情况。stop() 和
// run() 自己(见 run() 里 fork()/child_pid_ 赋值之间竞态窗口的说明)都要
// 用同一套逻辑发信号,抽成一个函数避免两处实现慢慢长歪。
void signal_group(pid_t pid, int sig) {
  if (::kill(-pid, sig) != 0 && errno != ESRCH) {
    // ESRCH(进程组已经不存在,子进程恰好在这一刻自己退出/被回收)是预期
    // 情况,忽略;其它 errno 这里也没有更好的补救办法,调用方通常会在
    // 下一次循环里用最新状态重试。
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

  // 对进程组发信号,直到 run() 线程把 child_pid_ 收回成 -1(说明它已经
  // waitpid() 把子进程回收了)为止;超过 5 s 还没死就升级成 SIGKILL。
  // 每一轮都重新读取 child_pid_,而不是只在进入 stop() 时读一次缓存:
  // run() 完全可能在这期间已经因为子进程自然退出而重启出下一个子进程,
  // 如果只信号"进来时那一个 pid",新起的这个就会被漏掉、白白多活一轮。
  //
  // review round 1 的 Important:这个循环原来唯一的退出条件是
  // child_pid_<=0,如果子进程卡在不可中断的 D 状态(挂死的 I/O),或者
  // kill() 一直失败(比如 EPERM),就会永远每 20 ms 转一圈、永不退出。
  // 这里加一个独立的绝对放弃时限——超过之后不再继续在这里发信号,直接
  // 往下走到 join()。放弃并不代表子进程杀不死:run() 线程自己也在跑同一套
  // 发信号/升级逻辑(见下面 run() 里的说明),这里放弃只是不想让 stop()
  // 这个循环自己也跟着永远转下去;真正卡在内核 D 状态的极端情况,任何
  // 用户态代码都无能为力,join() 之后会一直阻塞到子进程真的退出为止。
  const auto escalate_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  const auto abort_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  for (;;) {
    const int pid = child_pid_.load();
    if (pid <= 0) break;  // 已经被 run() 回收,或者从未 start() 过
    const auto now = std::chrono::steady_clock::now();
    if (now >= abort_deadline) {
      break;  // 放弃继续在这里重发信号,交给 run() 线程自己的逻辑和/或
              // 内核去把子进程收尾
    }
    const int sig = (now >= escalate_deadline) ? SIGKILL : SIGTERM;
    // 对整个进程组(-pid)发信号,而不是只发给直接子进程本身:rtkrcv 之类
    // 的目标程序会派生孙进程,只杀直接子进程会把孙进程留成孤儿。
    signal_group(pid, sig);
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

  // 同样的道理:子进程用来"关掉继承下来的 fd"的上限也在 fork() 之前算好
  // ——sysconf() 是否会分配内存不在异步信号安全的保证范围内,放在父进程
  // 这边调用没有任何顾虑。
  const long max_fd = ::sysconf(_SC_OPEN_MAX);

  while (running_.load()) {
    const pid_t pid = ::fork();
    if (pid < 0) {
      // fork() 失败(常见于 EAGAIN/ENOMEM,进程数或内存吃紧):根本没有
      // 子进程活下来,不计入 spawn_count_。稍等一下再试,同时仍然响应
      // stop()。
      interruptible_wait(base_delay);
      continue;
    }

    if (pid == 0) {
      // ---- 子进程:自此只允许调用异步信号安全的函数(见上面 argv/
      // max_fd 为什么要挪到 fork() 之前算好的注释)----

      // 独立进程组:目标程序(Task 7 里是 rtkrcv)可能派生孙进程,stop()
      // 需要能对整组发信号才不会留下孤儿。setsid() 失败(比如已经是
      // session leader)不影响后续流程,忽略返回值。
      ::setsid();

      // review round 1 的 Important:execv() 会原样保留调用者的信号屏蔽字
      // 和信号处置(SIG_IGN 会被继承,只有 SIG_DFL 才会被 exec 重置)。如果
      // start() 恰好是在一个屏蔽了/忽略了 SIGTERM 的线程上调用的(rclcpp
      // 的执行器线程、某些第三方库在初始化时确实会这么干),目标程序就会
      // 对 stop() 发的 SIGTERM 完全免疫,每次都要撑满 5 s 再挨 SIGKILL,
      // 且来不及做任何优雅收尾(flush 日志、关闭底层串口连接等)。这里
      // 显式把所有信号的处置重置成 SIG_DFL、屏蔽字清空——sigaction()/
      // sigemptyset()/sigprocmask() 都是异步信号安全的,不涉及任何内存
      // 分配——保证子进程对信号的反应完全由它自己的默认行为决定,不受
      // 父进程当时那个线程的信号状态影响。
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
      // close() 是异步信号安全的,循环本身不分配内存。
      for (long fd = 3; fd < max_fd; ++fd) {
        ::close(fd);
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
    // 结论是:下面这段等待逻辑不能再假设"stop() 一定已经看到了正确的
    // child_pid_ 并且在替我处理信号升级"。它必须自己独立地能完成"发现
    // running_ 变 false → 发 SIGTERM → 超时 → SIGKILL"这一整套动作,
    // 不依赖 stop() 是否抢到了正确的时机——无论最终是 stop() 那边先发现,
    // 还是这里自己先发现,子进程最终都会被杀死、被回收。
    bool stop_seen = !running_.load();
    std::chrono::steady_clock::time_point escalate_deadline{};
    if (stop_seen) {
      escalate_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    }

    // 等子进程退出:非阻塞 waitpid(WNOHANG) + poll(wake_fd_, tick) 轮询,
    // 而不是一个不可打断的 waitpid(pid, &status, 0)。
    //
    // review round 1 的 Critical 里同时指出:原来那个阻塞 waitpid() 本身
    // 就是一个没有轮询唤醒描述符的阻塞点,违反了这个包里"每个阻塞点都要
    // poll 唤醒描述符"的既定规则——理由(“stop() 会主动促成子进程死亡,
    // 所以 waitpid 总会在有限时间内返回”)在 stop() 因为上面那个竞态窗口
    // 错过 pid 的时候恰好不成立。现在换成有限超时的轮询:不管是 stop()
    // 通过唤醒管道通知,还是这里自己在轮询间隙里发现 running_ 已经变
    // false,都能在至多一个轮询周期内注意到并开始/推进信号升级,不再有
    // 任何一条路径依赖"对方一定会来救"。
    //
    // 排查过程中额外发现的一个坑:stop_seen 刚变 true 的那一刻,子进程
    // 未必已经跑到 setsid()(fork() 之后子进程和父进程并发执行,调度顺序
    // 不保证)。如果这时候只发一次 kill(-pid, SIGTERM),而子进程的进程组
    // 还没变成 pid(依然是继承自父进程的旧组),内核会直接报 ESRCH——这个
    // 值本身是"进程组不存在"的正常信号,但这里的语境下其实是"太早发了,
    // 没打中",而不是"已经死了"。旧写法把这次信号当成"发过了"就不再重发,
    // 会导致子进程一直活到 5 s 后的 SIGKILL 才死,拖慢了收尾(在专门复现
    // 这条竞态的回归测试里,直接表现为超过了测试给的等待上限)。修法很
    // 简单:和 stop() 自己的循环一样,只要 stop_seen 为真就每一轮 tick都
    // 重发一次当前应该发的信号(SIGTERM 或者升级后的 SIGKILL),而不是
    // "发现的时候发一次就不再管"——子进程真正完成 setsid() 通常只需要
    // 微秒级时间,下一轮 tick(至多 20 ms 后)重发就会命中。
    int status = 0;
    for (;;) {
      const pid_t w = ::waitpid(pid, &status, WNOHANG);
      if (w == pid) break;
      if (w < 0) {
        if (errno == EINTR) continue;
        break;  // 理论上不会走到(比如 ECHILD):当作已经结束处理,避免线程卡死
      }
      // w == 0:子进程还活着。

      if (!stop_seen && !running_.load()) {
        stop_seen = true;
        escalate_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      }
      if (stop_seen) {
        const int sig =
            (std::chrono::steady_clock::now() >= escalate_deadline) ? SIGKILL : SIGTERM;
        signal_group(pid, sig);
      }

      // 还没被要求停止时,轮询间隔可以放宽一些(200 ms),减少无谓的系统
      // 调用;一旦进入停止流程,收紧到 20 ms,让重发信号、升级判断和最终
      // 回收都能及时发生。
      const int timeout_ms = stop_seen ? 20 : 200;
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
        if (!stop_seen && !running_.load()) {
          stop_seen = true;
          escalate_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
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
