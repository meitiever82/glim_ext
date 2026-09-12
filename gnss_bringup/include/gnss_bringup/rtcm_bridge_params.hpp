#pragma once
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
