#pragma once
#include <atomic>
#include <string>
#include <thread>
#include <vector>

namespace gnss_bringup {

struct ProcessSupervisorConfig {
  std::string binary;
  std::vector<std::string> args;
  std::string cwd;
  double restart_delay_s = 5.0;
  // 活得比这还短就认为是崩溃循环(坏二进制 / 坏 conf),重启间隔翻倍直到上限,
  // 否则一天能刷出几万条 connected/disconnected
  double crash_loop_life_s = 30.0;
  double max_restart_delay_s = 60.0;
};

// 起一个子进程并在它退出后重启,永不放弃。子进程放进独立进程组,
// stop() 对整个进程组发 SIGTERM,超时再 SIGKILL —— rtkrcv 会派生孙进程。
class ProcessSupervisor {
public:
  explicit ProcessSupervisor(ProcessSupervisorConfig cfg);
  ~ProcessSupervisor();
  void start();
  void stop();
  int spawn_count() const { return spawn_count_.load(); }
  double current_delay_s() const { return current_delay_.load(); }
  // 最近一次成功 fork() 出来的 pid,回收之后也不会被清空(不同于内部的
  // child_pid_,后者在子进程被 waitpid() 收走之后会复位成 -1)。仅供测试
  // 在 stop() 返回之后核实子进程确实已经不存在(kill(pid, 0) == ESRCH),
  // 不是给业务逻辑用的。
  int last_child_pid() const { return last_spawned_pid_.load(); }

private:
  void run();

  ProcessSupervisorConfig cfg_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> spawn_count_{0};
  std::atomic<double> current_delay_{0.0};
  std::atomic<int> child_pid_{-1};
  std::atomic<int> last_spawned_pid_{-1};
  int wake_fd_ = -1, wake_wr_ = -1;
};

}  // namespace gnss_bringup
