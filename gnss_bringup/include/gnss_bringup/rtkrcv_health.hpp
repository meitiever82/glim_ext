#pragma once
// rtkrcv_node 自身健康(轮 3b 设计决定 9),1 Hz 发布到 ~/diagnostics。纯函数:节点只负责采样输入。
//   ERROR:rtkrcv 子进程未在运行(崩溃后退避等待重启,或解析不到二进制)
//   WARN :超过 no_solution_warn_s 没有解算行;区分"上行也没数据"(链路问题)与
//          "上行有数据但无解"(conf 问题:base_pos_type 所需的 1005/1006、obs_format)
//   OK   :其余;还没出过解且未超过门限时提示"等待首条解算输出"
#include <cstdio>
#include <optional>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace gnss_bringup {

struct RtkrcvHealthInput {
  bool child_running = false;
  double since_start_s = 0.0;                // 节点起来多久
  std::optional<double> since_solution_s;    // 距最近一条解算行;从未收到为空
  std::optional<double> since_uplink_s;      // 距最近一次上行(差分或观测)字节;从未收到为空
};

inline diagnostic_msgs::msg::DiagnosticStatus evaluate_rtkrcv_health(const RtkrcvHealthInput& in,
                                                                     double no_solution_warn_s) {
  using S = diagnostic_msgs::msg::DiagnosticStatus;
  S st;
  st.name = "rtkrcv_node";
  st.hardware_id = "rtkrcv";
  const auto add = [&st](const char* key, const std::optional<double>& v) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f", v.value_or(0.0));
    kv.value = v ? buf : "-";
    st.values.push_back(std::move(kv));
  };
  diagnostic_msgs::msg::KeyValue running;
  running.key = "child_running";
  running.value = in.child_running ? "true" : "false";
  st.values.push_back(running);
  add("since_solution_s", in.since_solution_s);
  add("since_uplink_s", in.since_uplink_s);

  char msg[256];
  if (!in.child_running) {
    st.level = S::ERROR;
    st.message = "rtkrcv 子进程未在运行——见节点日志里的退出记录";
    return st;
  }
  const double quiet_s = in.since_solution_s.value_or(in.since_start_s);
  if (quiet_s > no_solution_warn_s) {
    st.level = S::WARN;
    const bool uplink = in.since_uplink_s && *in.since_uplink_s <= no_solution_warn_s;
    std::snprintf(msg, sizeof(msg),
                  uplink ? "%.0f s 没有解算输出,上行有数据但无解——检查 base_pos_type 所需的 RTCM 1005/1006 与 obs_format"
                         : "%.0f s 没有解算输出,上行无数据——检查 rtcm_bridge 与平台/板卡链路",
                  quiet_s);
    st.message = msg;
    return st;
  }
  st.level = S::OK;
  st.message = in.since_solution_s ? "解算输出正常" : "等待首条解算输出";
  return st;
}

}  // namespace gnss_bringup
