// Judge 实现：逐行对齐旧 processor 的分支结构，改动只在"状态显式化 + 时间注入"。
// 每段注释标明对应的旧代码位置，评审时可逐行对照。
#include "gnss_core/judge.hpp"
#include <cmath>

namespace gnss_core::judge {

namespace {

bool IsEnuOutOfBounds(double x, double y, double z, int threshold) {
  return std::abs(x) > threshold || std::abs(y) > threshold || std::abs(z) > threshold;
}

}  // namespace

// 对齐 beiyun_processor.cpp::CheckValidity（行 192-300）
Verdict judge_beiyun(const Evidence& e, GateState& s, const Config& cfg) {
  Verdict v;
  // 旧代码：无分支命中时沿用上一帧 is_valid/rtk_status/fault_code（缓存初值）
  v.is_valid = s.prev_valid;
  v.rtk_status = s.prev_rtk;
  v.fault_code = s.prev_fault;

  bool branch_hit = false;
  if (e.pos_type == PosType::RtkFixed) {
    // 第一分支：RTK 固定解直通（旧代码不查 ins_good——golden by_003 实证的行为）
    v.is_valid = true;
    v.rtk_status = RtkStatus::RtkFix;
    v.fault_code = FaultCode::None;
    branch_hit = true;
  } else if (e.pos_type == PosType::RtkFloat && e.ins_good) {
    // 第二分支：RTK 浮点 + INS 良好 → 可用（无宽限）
    v.is_valid = true;
    v.rtk_status = RtkStatus::RtkFloat;
    v.fault_code = FaultCode::None;
    branch_hit = true;
  } else if (!e.ins_good) {
    // 第三分支：INS 未收敛
    if (e.pos_type == PosType::Single && e.diff_age < cfg.single_point_age_threshold) {
      // 单点宽限：仅降级 rtk_status，is_valid/fault_code 保留
      v.rtk_status = RtkStatus::RtkFloat;
    } else {
      v.is_valid = false;
      v.rtk_status = RtkStatus::RtkInit;
      v.fault_code = FaultCode::InsNotConverged;
    }
    branch_hit = true;
  }

  if (!branch_hit) {
    // 粘滞路径（ins_good 且非固定/浮点）：sticky_grace_time 内沿用上一帧，
    // 超时判 GraceTimeout 并按实际解类型修正 rtk_status
    if (!s.in_sticky) {
      s.in_sticky = true;
      s.sticky_start = e.now;
    }
    if (e.now - s.sticky_start > cfg.sticky_grace_time) {
      v.is_valid = false;
      RaiseFault(v.fault_code, FaultCode::GraceTimeout);
      if (e.pos_type == PosType::Single) {
        v.rtk_status = RtkStatus::Aposit;
      } else if (e.pos_type == PosType::Psrdiff) {
        v.rtk_status = RtkStatus::RtcmDiff;
      }
      // 其余解类型：无映射，保持上一帧
    }
  } else {
    s.in_sticky = false;
  }

  // === 硬门控（两品牌共用语义）===
  if (IsEnuOutOfBounds(e.enu_x, e.enu_y, e.enu_z, cfg.xyz_threshold)) {
    v.is_valid = false;
    RaiseFault(v.fault_code, FaultCode::EnuOutOfBounds);
  }
  if (e.sats_main < static_cast<uint16_t>(cfg.stars_threshold) ||
      e.sats_aux < static_cast<uint16_t>(cfg.stars_threshold)) {
    v.is_valid = false;
    RaiseFault(v.fault_code, FaultCode::SatelliteInsufficient);
  }
  // 双天线基线（仅 NARROW_INT）
  if (e.heading_valid_frame && e.heading_narrow_int &&
      std::abs(e.heading_length - cfg.heading_length) > cfg.heading_length_threshold) {
    v.is_valid = false;
    RaiseFault(v.fault_code, FaultCode::BaselineAbnormal);
  }

  s.prev_valid = v.is_valid;
  s.prev_rtk = v.rtk_status;
  s.prev_fault = v.fault_code;
  return v;
}

// 对齐 huace_processor.cpp::CheckValidity（行 228-300）
Verdict judge_huace(const Evidence& e, GateState& s, const Config& cfg) {
  Verdict v;
  FaultCode fault = FaultCode::None;

  // 时间基线惰性初始化
  if (!s.has_fix_time_base) {
    s.last_rtkfixed_t = e.now;
    s.has_fix_time_base = true;
  }

  const bool ins_not_combined = !e.ins_good;  // 适配器已映射 sta低4位!=2
  if (e.hw_warning || ins_not_combined) {
    v.is_valid = false;
    if (e.hw_warning) RaiseFault(fault, FaultCode::HardwareWarning);
    if (ins_not_combined) RaiseFault(fault, FaultCode::InsNotConverged);
  } else {
    v.is_valid = true;
  }

  // rtk_status 无条件透传 GNSS_pos_type
  v.rtk_status = static_cast<RtkStatus>(e.huace_gnss_pos_type);

  // 固定且 GPS 消息正常 → 刷新时间基线；否则按模式宽限
  constexpr uint8_t kGnssRtkFix = 4;
  constexpr uint8_t kGnssCombinedNavMode = 3;
  constexpr uint8_t kGnssRtkFloat = 5;
  if (e.huace_gnss_pos_type == kGnssRtkFix && e.huace_gnss_pos_valid) {
    s.last_rtkfixed_t = e.now;
  } else {
    const double unfixed_len = e.now - s.last_rtkfixed_t;
    double grace = cfg.unfixed_time;
    if (e.huace_gnss_pos_type == kGnssCombinedNavMode) {
      grace = cfg.combined_nav_mod_time;
    } else if (e.huace_gnss_pos_type == kGnssRtkFloat) {
      grace = cfg.rtk_float_grace_time;
    }
    if (unfixed_len > grace) {
      v.is_valid = false;
      // INS 未收敛时不叠加 GraceTimeout（避免遮蔽根因，对齐旧代码）
      if (!ins_not_combined) RaiseFault(fault, FaultCode::GraceTimeout);
    }
  }

  // 硬门控
  if (IsEnuOutOfBounds(e.enu_x, e.enu_y, e.enu_z, cfg.xyz_threshold)) {
    v.is_valid = false;
    RaiseFault(fault, FaultCode::EnuOutOfBounds);
  }
  if (e.sats_main < static_cast<uint16_t>(cfg.stars_threshold) ||
      e.sats_aux < static_cast<uint16_t>(cfg.stars_threshold)) {
    v.is_valid = false;
    RaiseFault(fault, FaultCode::SatelliteInsufficient);
  }

  v.fault_code = fault;
  s.prev_valid = v.is_valid;
  s.prev_rtk = v.rtk_status;
  s.prev_fault = v.fault_code;
  return v;
}

}  // namespace gnss_core::judge
