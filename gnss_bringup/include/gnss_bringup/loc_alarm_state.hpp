// Copyright (c) 2026 Boonray. All rights reserved.
// 移植说明：本文件移植自 boonray_ws/src/localization/localization_parser
// （include/localization_parser/ros_bridge/loc_alarm_state.hpp），
// Copyright (c) Boonray，2026-11-16 移植到 gnss_bringup，仅改命名空间。
//
// 定位故障自上报（告警码 202012）—— 纯逻辑部分：状态机 + 描述生成。
// 零 ROS 依赖（仅本包枚举 + std），可独立单测；ROS 发布器封装在 bridge 节点。
//
// 触发/恢复条件：
// - 触发：is_valid == false（等价 fault_code != None）—— 即时 FAULT_ADD；
// - 恢复：is_valid == true 且持续 recover_stable_s（默认 2s）—— FAULT_RESET；
// - 启动抑制：startup_grace_s（默认 10s）窗口内不判定。
//
// 启动抑制窗口只覆盖链路建立（TCP 接入 + 首帧数据）。整车上电、惯导冷启动
// 收敛中超过窗口仍未收敛会报 202012「INS未收敛」，这是期望行为：
//   1) CRITICAL 的 guardian 处置为暂停任务，开机无任务期间告警无副作用；
//   2) 该告警是 guardian/云端感知定位就绪状态的唯一上报通道，抑制会掩盖
//      NoData（设备未上电/网线断）等真实故障；
//   3) 处理器内部已有去抖（北云 sticky_grace_time 等），收敛期不会反复抖动。
// 若现场认为冷启动期告警频繁，调大 startup_grace_s，而不是按 fault_code 豁免。
//
// 状态机范式对齐 soft_dtu_alarm.h（SoftDtuAlarm）：
// !active && alarm_raw            → 即时 FAULT_ADD（含当时描述）
// active  && 描述变化              → min_interval 防抖后重发 FAULT_ADD
// active  && 描述不变              → cooldown 到期补发 FAULT_ADD
// active  && !alarm_raw && 稳定 2s → 即时 FAULT_RESET
//
// 线程模型：仅由发布定时器线程单线程驱动，状态无锁。
#pragma once
#include <chrono>
#include <cstdio>
#include <optional>
#include <string>

#include "gnss_bringup/localization_types.hpp"

namespace gnss_bringup::loc {

/// 告警判定输入（发布周期从定位数据快照构造）
struct LocAlarmInput {
  bool is_valid{true};
  FaultCode fault_code{FaultCode::None};
  RtkStatus rtk_status{RtkStatus::RtkInit};
  uint16_t svs{0};   ///< 第一天线使用卫星数
  uint16_t svs2{0};  ///< 第二天线使用卫星数
  double age{0.0};   ///< 差分龄期（秒）
  bool client_connected{false};  ///< 设备连接状态（无 ROS 传输层时由 bridge 填 false）
};

/// 状态机输出动作（发布器据此组装 FAULT_ADD / FAULT_RESET）
struct LocAlarmAction {
  bool add{true};   ///< true=FAULT_ADD，false=FAULT_RESET
  std::string msg;  ///< 告警/恢复描述
};

/// 单码 202012 告警状态机（纯逻辑，无 ROS 依赖）
/// 时间由调用方注入（steady_clock 时间点），测试可用固定时间轴确定性驱动。
class LocAlarmState {
 public:
  struct Params {
    double startup_grace_s{10.0};  ///< 启动抑制窗口（秒）
    double recover_stable_s{2.0};  ///< 恢复稳定期（秒）
    double min_interval_s{10.0};   ///< 描述变化防抖间隔（秒）
    double cooldown_s{300.0};      ///< 持续告警补发周期（秒）
  };

  explicit LocAlarmState(const Params& p,
                         std::chrono::steady_clock::time_point start =
                             std::chrono::steady_clock::now())
      : start_(start) {
    // 防御：!(v>0) 同时覆盖 <=0 与 NaN（NaN 比较恒 false）
    params_.startup_grace_s = fix_param(p.startup_grace_s, 10.0);
    params_.recover_stable_s = fix_param(p.recover_stable_s, 2.0);
    params_.min_interval_s = fix_param(p.min_interval_s, 10.0);
    params_.cooldown_s = fix_param(p.cooldown_s, 300.0);
  }

  /// 驱动一次状态机（单线程调用）
  /// @param alarm_raw 原始告警条件（!is_valid）
  /// @param desc      当前告警描述（alarm_raw=false 时忽略）
  /// @param now       当前时刻（注入便于测试）
  /// @return 需要发布的动作；nullopt = 无动作
  auto update(bool alarm_raw, const std::string& desc,
              std::chrono::steady_clock::time_point now)
      -> std::optional<LocAlarmAction> {
    // 启动抑制窗口：窗口内不判定（不 ADD 也不累计恢复期）
    if (now - start_ < std::chrono::duration<double>(params_.startup_grace_s)) {
      return std::nullopt;
    }

    // 恢复稳定期跟踪：无效→有效下降沿记录 valid_since_，期间再次无效则作废
    if (alarm_raw) {
      valid_since_.reset();
    } else if (!valid_since_.has_value()) {
      valid_since_ = now;
    }

    if (!active_) {
      if (!alarm_raw) return std::nullopt;
      // 无告警 → 有告警：即时上报
      active_ = true;
      active_desc_ = desc;
      last_publish_ = now;
      return LocAlarmAction{true, desc};
    }

    if (alarm_raw) {
      if (desc != active_desc_) {
        // 描述变化（如卫星数不足恶化/不同门控切换）：min_interval 防抖后重发
        if (now - last_publish_ >=
            std::chrono::duration<double>(params_.min_interval_s)) {
          active_desc_ = desc;
          last_publish_ = now;
          return LocAlarmAction{true, desc};
        }
      } else if (now - last_publish_ >=
                 std::chrono::duration<double>(params_.cooldown_s)) {
        // 描述不变：cooldown 到期补发，保证告警不丢失
        last_publish_ = now;
        return LocAlarmAction{true, desc};
      }
      return std::nullopt;
    }

    // active && !alarm_raw：需有效持续 recover_stable_s 才撤销（抖动保护）
    if (valid_since_.has_value() &&
        now - *valid_since_ >=
            std::chrono::duration<double>(params_.recover_stable_s)) {
      active_ = false;
      active_desc_.clear();
      valid_since_.reset();
      last_publish_ = now;
      return LocAlarmAction{false, kRecoverMsg};
    }
    return std::nullopt;
  }

  bool active() const { return active_; }

  /// 生成告警描述（纯函数，动态携带上下文便于排查）
  /// 格式："定位无效：<具体原因>（rtk=<状态> svs=<n>/<n> age=<x>s）"；
  /// NoData/DataTimeout 时上下文换成设备连接状态（rtk/svs/age 此时无意义）。
  static auto build_fault_msg(const LocAlarmInput& in) -> std::string {
    const char* reason;
    switch (in.fault_code) {
      case FaultCode::InsNotConverged:       reason = "INS未收敛"; break;
      case FaultCode::GraceTimeout:          reason = "非固定解超过宽限期"; break;
      case FaultCode::HardwareWarning:       reason = "组合导航硬件告警"; break;
      case FaultCode::SatelliteInsufficient: reason = "卫星数不足"; break;
      case FaultCode::BaselineAbnormal:      reason = "双天线基线异常"; break;
      case FaultCode::EnuOutOfBounds:        reason = "ENU坐标越界"; break;
      case FaultCode::ParseFail:             reason = "连续解析定位数据失败"; break;
      case FaultCode::DataTimeout:           reason = "定位数据超时"; break;
      case FaultCode::NoData:                reason = "未收到定位数据"; break;
      default:                               reason = "未知原因"; break;
    }
    std::string msg = std::string("定位无效：") + reason;

    char ctx[128];
    if (in.fault_code == FaultCode::NoData ||
        in.fault_code == FaultCode::DataTimeout) {
      std::snprintf(ctx, sizeof(ctx), "（设备%s）",
                    in.client_connected ? "已连接" : "未连接");
    } else {
      std::snprintf(ctx, sizeof(ctx), "（rtk=%s svs=%u/%u age=%.1fs）",
                    rtk_status_to_string(in.rtk_status),
                    static_cast<unsigned>(in.svs),
                    static_cast<unsigned>(in.svs2), in.age);
    }
    return msg + ctx;
  }

  /// 恢复文案（固定）
  static constexpr const char* kRecoverMsg = "定位恢复正常。";

 private:
  /// 参数防御：<=0 / NaN 回退默认值（!(v>0) 同时覆盖 NaN）
  static auto fix_param(double v, double def) -> double {
    return (v > 0.0) ? v : def;
  }

  Params params_;
  std::chrono::steady_clock::time_point start_;  ///< 启动抑制窗口起点

  // 以下状态仅 update 单线程访问，无需加锁
  bool active_{false};
  std::string active_desc_;
  std::chrono::steady_clock::time_point last_publish_{};
  std::optional<std::chrono::steady_clock::time_point> valid_since_{};
};

}  // namespace gnss_bringup::loc
