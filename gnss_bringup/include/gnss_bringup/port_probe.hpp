#pragma once

namespace gnss_bringup {

// 探测 127.0.0.1:port 是否已经被别的进程占着。用于启动时发现残留的
// rtkrcv 孤儿,或误开的第二个实例——两者都会让本节点连上别人的解流,把
// 陈旧解当新鲜数据发出去。
//
// fix round 1 的 Important 3:探测本身也会失败(fd 耗尽、意料之外的系统
// 调用错误……),这和"探测成功、端口确实空闲"是两种完全不同的结果,调用方
// 必须能区分,不能像最初的版本那样把两者都悄悄折叠成同一个 false——一次
// 因为 EMFILE 而失败的探测,如果被当成"端口空闲"处理,会放过一个真正被
// 占用的端口:新起的 rtkrcv 绑不上,TcpStream 转而连上残留的孤儿,复现的
// 正是这个探测器存在的意义要防住的那类故障,而且不会有任何日志——这违反了
// 这个包"never a silent giveup"的既定规则(参见 LocalReserver::OnFatalError
// 的先例)。
enum class PortProbeResult {
  kFree,         // 探测成功,端口当前空闲
  kListening,    // 探测成功,端口已经被占用
  kProbeFailed,  // 探测本身失败,不能据此判断端口到底空不空
};

struct PortProbeOutcome {
  PortProbeResult result = PortProbeResult::kProbeFailed;
  // 仅在 result == kProbeFailed 时有意义:失败时的 errno,供调用方写进日志。
  int probe_errno = 0;
};

PortProbeOutcome probe_local_port(int port);

}  // namespace gnss_bringup
