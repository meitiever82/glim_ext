#pragma once
// 测试专用:把一个节点可执行文件当子进程起起来、把 stdout/stderr 抓进日志文件、
// 等它退出。fork 之后子进程里只调用 async-signal-safe 的 open/dup2/execve/_exit
// ——调用方(比如 Task 6 的测试)可能已经起了 rclcpp 的线程。
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

extern char** environ;

namespace gnss_bringup_test {

// round 1 review(最初在 test_gnss_diag_node_process.cpp 里写出来的教训):
// 原来每个用例末尾显式 fs::remove_all(dir),但 ASSERT_* 失败时 gtest 会在那
// 一行直接 return,末尾的清理代码整段跳不到,临时目录就漏删了。改成
// RAII——构造顺序必须在 NodeProcess 之前,这样它的析构(删除目录)一定晚于
// NodeProcess 的析构(SIGKILL+waitpid,保证子进程已经退出、不会再往目录里写
// 东西),LIFO 析构顺序自动保证"先等子进程死透,再删目录",不需要额外的
// 先后协调。搬进这个共享头文件,供多个节点级测试文件复用。
class TempDirGuard {
 public:
  explicit TempDirGuard(std::string dir) : dir_(std::move(dir)) {}
  ~TempDirGuard() {
    if (!dir_.empty()) std::filesystem::remove_all(dir_);
  }
  TempDirGuard(const TempDirGuard&) = delete;
  TempDirGuard& operator=(const TempDirGuard&) = delete;

 private:
  std::string dir_;
};

inline std::string make_temp_dir(const std::string& prefix) {
  const char* base = std::getenv("TMPDIR");
  std::string tmpl = std::string(base ? base : "/tmp") + "/" + prefix + "XXXXXX";
  std::vector<char> buf(tmpl.begin(), tmpl.end());
  buf.push_back('\0');
  if (::mkdtemp(buf.data()) == nullptr) return {};
  return std::string(buf.data());
}

// 让内核挑一个空闲端口再关掉。有极小的竞态窗口,测试里可以接受。
inline int pick_free_port() {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) return -1;
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  int port = -1;
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
    socklen_t len = sizeof(addr);
    if (::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) == 0) port = ntohs(addr.sin_port);
  }
  ::close(fd);
  return port;
}

inline std::string read_file(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

inline std::size_t count_occurrences(const std::string& hay, const std::string& needle) {
  std::size_t n = 0;
  for (std::size_t pos = hay.find(needle); pos != std::string::npos; pos = hay.find(needle, pos + needle.size())) ++n;
  return n;
}

// poll 默认 50 ms;"启动中途中断"一类测试要在构造函数剩下的几毫秒窗口里送出信号,
// 需要传 1 ms 这样的细粒度轮询。
inline bool wait_until(const std::function<bool()>& pred, double timeout_s,
                       std::chrono::milliseconds poll = std::chrono::milliseconds(50)) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < end) {
    if (pred()) return true;
    std::this_thread::sleep_for(poll);
  }
  return pred();
}

// 在 127.0.0.1 上占住一个端口并 listen,析构时释放——模拟"孤儿 rtkrcv 占着 sol_port"。
class ListeningSocket {
public:
  ListeningSocket() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (fd_ >= 0 && ::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0 &&
        ::listen(fd_, 4) == 0) {
      socklen_t len = sizeof(addr);
      if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len) == 0) port_ = ntohs(addr.sin_port);
    }
  }
  ~ListeningSocket() { if (fd_ >= 0) ::close(fd_); }
  ListeningSocket(const ListeningSocket&) = delete;
  ListeningSocket& operator=(const ListeningSocket&) = delete;
  int port() const { return port_; }

private:
  int fd_ = -1;
  int port_ = -1;
};

class NodeProcess {
public:
  NodeProcess(const std::string& exe, const std::vector<std::string>& args, const std::string& log_path,
              const std::vector<std::pair<std::string, std::string>>& env_overrides)
      : log_path_(log_path) {
    // argv / envp 全部在 fork 之前建好
    std::vector<std::string> argv_s{exe};
    argv_s.insert(argv_s.end(), args.begin(), args.end());
    std::vector<std::string> env_s;
    for (char** e = environ; *e != nullptr; ++e) {
      const std::string kv(*e);
      bool overridden = false;
      for (const auto& [k, v] : env_overrides) {
        if (kv.rfind(k + "=", 0) == 0) overridden = true;
      }
      if (!overridden) env_s.push_back(kv);
    }
    for (const auto& [k, v] : env_overrides) env_s.push_back(k + "=" + v);
    std::vector<char*> argv, envp;
    for (auto& s : argv_s) argv.push_back(s.data());
    argv.push_back(nullptr);
    for (auto& s : env_s) envp.push_back(s.data());
    envp.push_back(nullptr);

    pid_ = ::fork();
    if (pid_ == 0) {
      // 子进程第一件事:约定父进程(这里是发起 fork() 的 gtest 主线程)退出/
      // 被杀时,内核给子进程发 SIGKILL——ctest 超时或测试二进制被 kill -9,
      // node 和它拉起的 rtkrcv 不能继续跑着占用端口。必须放在 execve() 之前、
      // 且是异步信号安全调用。PDEATHSIG 在“发起 fork 的线程”退出时触发,
      // 这里就是 main 线程(NodeProcess 按简报要求在 rclcpp::init 之前构造),
      // 符合预期。
      ::prctl(PR_SET_PDEATHSIG, SIGKILL, 0, 0, 0);
      const int fd = ::open(log_path_.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
      if (fd >= 0) {
        ::dup2(fd, STDOUT_FILENO);
        ::dup2(fd, STDERR_FILENO);
      }
      ::execve(argv[0], argv.data(), envp.data());
      _exit(127);
    }
  }

  ~NodeProcess() {
    if (pid_ > 0) {
      ::kill(pid_, SIGKILL);
      ::waitpid(pid_, nullptr, 0);
    }
  }
  NodeProcess(const NodeProcess&) = delete;
  NodeProcess& operator=(const NodeProcess&) = delete;

  // 返回退出码;被信号终止返回 128+信号;超时则 SIGKILL 并返回 -1。
  int wait_exit(double timeout_s) {
    if (pid_ <= 0) return -1;
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    for (;;) {
      int status = 0;
      const pid_t w = ::waitpid(pid_, &status, WNOHANG);
      if (w == pid_) {
        pid_ = -1;
        if (WIFEXITED(status)) return WEXITSTATUS(status);
        if (WIFSIGNALED(status)) return 128 + WTERMSIG(status);
        return -1;
      }
      if (std::chrono::steady_clock::now() >= end) {
        ::kill(pid_, SIGKILL);
        ::waitpid(pid_, nullptr, 0);
        pid_ = -1;
        return -1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void interrupt() { if (pid_ > 0) ::kill(pid_, SIGINT); }
  std::string log() const { return read_file(log_path_); }
  pid_t pid() const { return pid_; }

private:
  std::string log_path_;
  pid_t pid_ = -1;
};

}  // namespace gnss_bringup_test
