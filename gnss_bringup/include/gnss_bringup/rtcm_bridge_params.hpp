#pragma once
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace gnss_bringup {

// rtcm_bridge_node 自己的、跟 TcpStream 无关的纯逻辑:参数校验与去重检测。
// 抽出来是因为这两条是这个节点本身的、可独立测试的判断,不是 TcpStream 的行为。

// TCP 端口合法范围 [0, 65535]。0 在监听模式下表示"由内核选择"(TcpStream 测试用)。
// 越界值(如 99999)如果不在这里挡住,会在 htons(static_cast<uint16_t>(port)) 处
// 被静默截断——节点会绑定一个跟日志里打印的完全不同的端口,且没有任何报错。
inline bool is_valid_port(int64_t port) {
  return port >= 0 && port <= 65535;
}

// final-fix-wave 第 1 项:端口校验按连接方向再收紧一次。0 只在监听方(内核
// 选端口,LocalReserver/TcpStream 的服务端场景)才有意义;在连接方
// (TcpStream 客户端去连一个对端——rtcm_bridge 的 listen=false 流,以及
// rtkrcv_node 拿 sol_port 去连 rtkrcv 的 outstr1)connect(...:0) 没有任何
// 操作系统语义,不会报任何错误,只会 connect() 到"端口 0",内核会立刻
// 拒绝,worker 因此永远退避重试——这正是"dials nothing, silently, forever"
// 这个最坏的静默故障模式,而且和 is_valid_port 本身的越界检查一样,不能靠
// TcpStream 内部去分辨"我是被哪种场景创建的",必须在读参数这一层、知道
// "这个端口到底是拿去 listen 还是拿去 connect"的地方挡住。
inline bool is_valid_port_for_direction(int64_t port, bool listen) {
  if (!is_valid_port(port)) return false;
  if (port == 0 && !listen) return false;
  return true;
}

// 校验一个"必须是正数的秒数"参数(重连退避、空闲超时……)。
// <=0 或非有限值(NaN/inf)如果放过去,会在 TcpStream 内部造成钉死行为——
// idle_timeout_s<=0 会让 pump() 里的 timeout_ms 变成 -1,poll() 因此永久阻塞
// 直到真正有数据或者被 stop() 打断(见 tcp_stream.cpp pump() 的这段逻辑),
// 悄悄关掉这个节点断线自愈的核心能力(参考实现 rtk-monitor 的注释同样把
// "对端消失不发 RST"列为隧道弱链路下最需要被检测的场景)。与
// is_valid_port 同一个道理:提前挡住、报得清楚,而不是让一个 YAML 里的 0
// 在现场变成一次静默的自愈失效——与 rtkrcv_node 对 sol_idle_timeout_s 的
// declare_positive_seconds 是同一个决定的镜像,不是新发明的校验风格。
//
// 命名特意不叫 is_positive_finite_seconds:rtk_fix_mapping.hpp 里已经有一个
// 同名同签名的谓词(rtkrcv_node 专用),而 rtkrcv_node.cpp 同时 include 这个
// 头文件(取 is_valid_port)和 rtk_fix_mapping.hpp——两个头文件在同一个 TU
// 里定义同名自由函数会导致 "redefinition" 编译错误(已实测复现)。这里的
// is_valid_port/is_positive_finite_seconds 编号后果对 rtcm_bridge_params.hpp
// 来说本就是独立的一份纯逻辑(CMakeLists.txt 里 test_rtcm_bridge_params 也
// 没有链接 gnss_core / include rtk_fix_mapping.hpp),用不同的名字避免这个
// 头文件之间的隐式耦合,比反过来改 Task 1-7 的 rtk_fix_mapping.hpp 更安全。
inline bool is_positive_finite_backoff_seconds(double v) {
  return std::isfinite(v) && v > 0.0;
}

// 返回 names 中第一个重复出现的名字;没有重复则返回 nullopt。
// 用于在 declare_parameter("<name>.host", ...) 因重复声明而抛
// ParameterAlreadyDeclaredException 之前,给出指名道姓的诊断。
inline std::optional<std::string> find_duplicate_stream_name(const std::vector<std::string>& names) {
  std::unordered_set<std::string> seen;
  seen.reserve(names.size());
  for (const auto& n : names) {
    if (!seen.insert(n).second) return n;
  }
  return std::nullopt;
}

}  // namespace gnss_bringup
