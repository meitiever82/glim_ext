// 移植说明：本文件移植自 boonray_ws/src/localization/localization_parser
// （src/core/processor/beiyun_processor.cpp CheckValidity 与
// include/localization_parser/core/processor/processor_base.hpp 的硬门控），
// Copyright (c) Boonray，2026-11-16 移植到 gnss_bringup。
//
// 判定顺序、粘滞（sticky）语义、宽限超时、fault_code 优先级与旧实现逐条一致；
// 唯一的结构性改动是把 steady_clock::now() 改成由调用方注入 now，便于确定性单测。
//
// 纯逻辑：不依赖 ROS；输入是驱动话题已解析出的标量，输出是
// is_valid / rtk_status / fault_code 三件套。
#pragma once
#include <chrono>
#include <cmath>
#include <cstdint>

#include "gnss_bringup/localization_types.hpp"

namespace gnss_bringup::loc {

/// 单帧门控输入（由 bridge 从驱动话题组装）
struct GateInput {
  PosType pos_type{PosType::None};        ///< INSPVAX pos_type
  InsStatus ins_status{InsStatus::Unknown};  ///< INSPVAX ins_status
  double age{0.0};                        ///< 差分龄期（秒）
  uint16_t svs{0};                        ///< 主天线使用卫星数
  uint16_t svs2{0};                       ///< 第二天线使用卫星数
  bool heading_narrow_int{false};         ///< 双天线航向是否为 NARROW_INT 解
  double heading_length{0.0};             ///< 双天线基线长度（米）
  double x{0.0};                          ///< ENU 东（米），本帧换算结果
  double y{0.0};                          ///< ENU 北（米）
  double z{0.0};                          ///< ENU 天（米）
};

struct GateParams {
  /// 粘滞宽限：INS 良好但解降级为差分/单点/无解时，沿用上一帧结果的最长时间
  double sticky_grace_time{30.0};
  /// 单点解差分龄期阈值：INS 未收敛时，单点解且 age 小于该值只降级不判无效
  double single_point_age_threshold{10.0};
  /// ENU 坐标越界阈值（米）
  double xyz_threshold{10000.0};
  /// 卫星数门控阈值（任一天线低于该值即无效）
  uint16_t stars_threshold{10};
  /// 双天线标称基线长度（米）
  double heading_length{1.276};
  /// 基线长度允许偏差（米）
  double heading_length_threshold{0.2};
};

struct GateOutput {
  bool is_valid{false};
  RtkStatus rtk_status{RtkStatus::RtkInit};
  FaultCode fault_code{FaultCode::None};
};

/// 门控状态机。rtk_status 的权威来源是本类（与旧实现一致），
/// localization_mapping.hpp 的 rtk_status_from_pos_type 只用于旁路/诊断，
/// 不要在 bridge 里用它覆盖本类输出，否则单点宽限与粘滞语义会丢失。
class LocalizationGate {
 public:
  explicit LocalizationGate(const GateParams& params = GateParams())
      : params_(params) {
    // 旧实现首帧缓存为空 → is_valid=false / RtkInit / None
    prev_ = GateOutput{};
  }

  /// 驱动一帧门控。now 由调用方注入（生产用 steady_clock，测试用固定时间轴）。
  auto update(const GateInput& in,
              std::chrono::steady_clock::time_point now) -> GateOutput {
    // 旧实现在分支未命中时沿用上一帧的 is_valid/rtk_status/fault_code，
    // 下面以 prev_ 作初值复刻该粘滞语义。
    GateOutput out = prev_;

    const bool ins_good = (in.ins_status == InsStatus::InsSolutionGood);
    bool branch_hit = false;

    if (in.pos_type == PosType::InsRtkfixed) {
      // 第一分支：固定解直接可用（不看 INS 状态）
      out.is_valid = true;
      out.rtk_status = RtkStatus::RtkFix;
      out.fault_code = FaultCode::None;
      branch_hit = true;
    } else if (in.pos_type == PosType::InsRtkfloat && ins_good) {
      // 第二分支：浮点解且 INS 良好 → 可用（无宽限）
      out.is_valid = true;
      out.rtk_status = RtkStatus::RtkFloat;
      out.fault_code = FaultCode::None;
      branch_hit = true;
    } else if (!ins_good) {
      // 第三分支：INS 未收敛
      if (in.pos_type == PosType::InsPsrsp &&
          in.age < params_.single_point_age_threshold) {
        // 单点解且龄期小于阈值：只降级 rtk_status，不改 is_valid / fault_code
        // （宽限降级，沿用上一帧有效性）
        out.rtk_status = RtkStatus::RtkFloat;
      } else {
        out.is_valid = false;
        out.rtk_status = RtkStatus::RtkInit;
        out.fault_code = FaultCode::InsNotConverged;
      }
      branch_hit = true;
    }

    if (!branch_hit) {
      // 粘滞路径：INS 良好且解为差分/单点/无解。宽限内沿用上一帧；
      // 超过 sticky_grace_time 判无效并上报 GraceTimeout，同时把 rtk_status
      // 修正为本帧的实际解类型（无对应等级时保持上一帧）。
      if (!in_sticky_) {
        in_sticky_ = true;
        sticky_start_ = now;
      }
      const double sticky_sec =
          std::chrono::duration<double>(now - sticky_start_).count();
      if (sticky_sec > params_.sticky_grace_time) {
        out.is_valid = false;
        raise_fault_code(out.fault_code, FaultCode::GraceTimeout);
        if (in.pos_type == PosType::InsPsrsp) {
          out.rtk_status = RtkStatus::Aposit;
        } else if (in.pos_type == PosType::InsPsrdiff) {
          out.rtk_status = RtkStatus::RtcmDiff;
        }
      }
    } else {
      // 任一确定性分支命中即退出粘滞：再次降级时重新起算宽限
      in_sticky_ = false;
    }

    // === 硬门控 ===
    // rtk_status 不被门控压 0：门控失效只通过 is_valid / fault_code 表达，
    // 便于排障时区分"设备解算差"与"解算好但被门控"。
    if (enu_out_of_bounds(in.x, in.y, in.z)) {
      out.is_valid = false;
      raise_fault_code(out.fault_code, FaultCode::EnuOutOfBounds);
    }

    // 卫星数门控：任一天线低于阈值即无效（旧实现无 svs2==0 豁免；
    // 第二天线数据缺失时 svs2 为 0，同样判无效）
    if (in.svs < params_.stars_threshold || in.svs2 < params_.stars_threshold) {
      out.is_valid = false;
      raise_fault_code(out.fault_code, FaultCode::SatelliteInsufficient);
    }

    // 双天线基线门控：仅 NARROW_INT 解类型参与
    if (in.heading_narrow_int && heading_baseline_abnormal(in.heading_length)) {
      out.is_valid = false;
      raise_fault_code(out.fault_code, FaultCode::BaselineAbnormal);
    }

    prev_ = out;
    return out;
  }

  const GateOutput& last() const { return prev_; }
  bool in_sticky() const { return in_sticky_; }

 private:
  auto enu_out_of_bounds(double x, double y, double z) const -> bool {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return true;
    return std::abs(x) > params_.xyz_threshold ||
           std::abs(y) > params_.xyz_threshold ||
           std::abs(z) > params_.xyz_threshold;
  }

  auto heading_baseline_abnormal(double length) const -> bool {
    if (!std::isfinite(length)) return true;
    return std::abs(length - params_.heading_length) >
           params_.heading_length_threshold;
  }

  GateParams params_;
  GateOutput prev_;  ///< 上一帧输出（粘滞语义的初值来源）
  bool in_sticky_{false};
  std::chrono::steady_clock::time_point sticky_start_{};
};

}  // namespace gnss_bringup::loc
