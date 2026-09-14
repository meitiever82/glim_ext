#pragma once
#include <string>

namespace gnss_bringup {

// rtkrcv_node 的 sol 流状态日志去重闸门(纯逻辑,调用方负责加锁)。
//
// 隧道里 rtkrcv 长时间没有解算输出时,TcpStream 每 sol_idle_timeout_s 报一次
// "idle timeout" 断开、紧接着一次 connected;不加控制会在 INFO 级别刷一整夜,
// 把真正要看的日志淹掉。规则:
//   - 第一次 idle timeout 照常打印,并进入安静期;
//   - 安静期内的 idle timeout 与随后的 connected 都降为 DEBUG;
//   - 收到一条有效解算行,或出现 idle timeout 以外的断开(对端关闭 / 连不上,
//     说明 rtkrcv 本身出了状况)时退出安静期,并且那条断开照常打印。
class SolStreamLogGate {
public:
  // 返回 true:按原级别打印;false:降为 DEBUG。
  bool on_status(bool connected, const std::string& detail) {
    if (connected) return !quiet_;
    if (detail == "idle timeout") {
      if (quiet_) return false;
      quiet_ = true;
      return true;
    }
    quiet_ = false;
    return true;
  }

  void on_solution_line() { quiet_ = false; }
  bool quiet() const { return quiet_; }

private:
  bool quiet_ = false;
};

}  // namespace gnss_bringup
