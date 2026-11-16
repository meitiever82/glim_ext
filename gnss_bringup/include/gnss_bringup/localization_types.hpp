// 移植说明：本文件移植自 boonray_ws/src/localization/localization_parser
// （include/localization_parser/core/common/types.hpp），Copyright (c) Boonray，
// 2026-11-16 移植到 gnss_bringup，仅改命名空间与文件布局，枚举值与语义保持不变。
// 枚举值直接对齐 robominer_msgs 的 LocalizationDataAux / LocalizationStatus 契约，
// 下游兼容性由这些数值保证，不得重排或改值。
#pragma once
#include <cstdint>
#include <string>

namespace gnss_bringup::loc {

/// INS 组合导航状态（北云 INSPVAX ins_status、华测对应状态）
enum class InsStatus : uint8_t {
  Unknown                = 255,  ///< 未知（默认值，解析失败/未匹配时使用）
  InsInactive            = 0,    ///< INS 未激活
  InsAligning            = 1,    ///< INS 初始对准中
  InsHighVariance        = 2,    ///< INS 高方差（精度差）
  WaitingAzimuth         = 3,    ///< 等待航向角初始化
  InsSolutionFree        = 4,    ///< INS 自由解（卫星结果较差不可用）
  InsAlignmentComplete   = 5,    ///< INS 对准完成
  DeterminingOrientation = 6,    ///< 方位角判定中
  WaitingInitialPos      = 7,    ///< 等待初始位置
  InsSolutionGood        = 8,    ///< INS 解算良好（可用）
  InitializingBiases     = 9,    ///< 偏差初始化中
  MotionDetect           = 10,   ///< 运动检测状态
};

/// GNSS 定位解类型（INSPVAX pos_type）
enum class PosType : uint8_t {
  None        = 0,  ///< 无解
  InsPsrsp    = 1,  ///< INS 单点定位
  InsPsrdiff  = 2,  ///< INS 差分定位（伪距差分）
  InsRtkfloat = 3,  ///< INS RTK 浮点解
  InsRtkfixed = 4,  ///< INS RTK 固定解
};

/// RTK 解算状态 / 定位质量等级（LocalizationStatus.status 契约）
enum class RtkStatus : uint8_t {
  RtkInit     = 0,   ///< 定位不可用（初始化 / INS 未收敛）
  Aposit      = 1,   ///< 单点定位
  RtcmDiff    = 2,   ///< 差分定位
  NotUsed     = 3,   ///< 保留值
  RtkFix      = 4,   ///< RTK 固定解
  RtkFloat    = 5,   ///< RTK 浮点解
  RtkOther    = 6,   ///< 航迹推算
  DataTimeout = 11,  ///< 数据超时（无数据输入）
};

/// 定位门控故障原因码（LocalizationDataAux.fault_code 契约）
/// 赋值规则：同一帧多种门控并存时保留优先级最高者（枚举值越大优先级越高），
/// 见 raise_fault_code()。
enum class FaultCode : uint8_t {
  None                  = 0,  ///< 无门控触发
  InsNotConverged       = 1,  ///< INS 未收敛
  GraceTimeout          = 2,  ///< 非固定解超过宽限期（北云粘滞路径超时同用此码）
  HardwareWarning       = 3,  ///< 设备硬件告警（华测）
  SatelliteInsufficient = 4,  ///< 卫星数不足
  BaselineAbnormal      = 5,  ///< 双天线基线长度异常
  EnuOutOfBounds        = 6,  ///< ENU 坐标越界
  ParseFail             = 7,  ///< 持续解析失败
  DataTimeout           = 8,  ///< 数据超时
  NoData                = 9,  ///< 未收到首帧数据
};

/// 门控原因赋值：仅当新原因优先级更高时覆盖
inline void raise_fault_code(FaultCode& cur, FaultCode next) {
  if (next > cur) cur = next;
}

constexpr auto rtk_status_to_string(RtkStatus s) -> const char* {
  switch (s) {
    case RtkStatus::RtkInit:     return "RtkInit(0)";
    case RtkStatus::Aposit:      return "Aposit(1)";
    case RtkStatus::RtcmDiff:    return "RtcmDiff(2)";
    case RtkStatus::NotUsed:     return "NotUsed(3)";
    case RtkStatus::RtkFix:      return "RtkFix(4)";
    case RtkStatus::RtkFloat:    return "RtkFloat(5)";
    case RtkStatus::RtkOther:    return "RtkOther(6)";
    case RtkStatus::DataTimeout: return "DataTimeout(11)";
    default:                     return "Unknown";
  }
}

constexpr auto fault_code_to_string(FaultCode c) -> const char* {
  switch (c) {
    case FaultCode::None:                  return "None(0)";
    case FaultCode::InsNotConverged:       return "InsNotConverged(1)";
    case FaultCode::GraceTimeout:          return "GraceTimeout(2)";
    case FaultCode::HardwareWarning:       return "HardwareWarning(3)";
    case FaultCode::SatelliteInsufficient: return "SatelliteInsufficient(4)";
    case FaultCode::BaselineAbnormal:      return "BaselineAbnormal(5)";
    case FaultCode::EnuOutOfBounds:        return "EnuOutOfBounds(6)";
    case FaultCode::ParseFail:             return "ParseFail(7)";
    case FaultCode::DataTimeout:           return "DataTimeout(8)";
    case FaultCode::NoData:                return "NoData(9)";
    default:                               return "Unknown";
  }
}

/// INS 状态字符串 → 枚举（未匹配返回 Unknown，调用方决定是否沿用上一帧）
inline auto ins_status_from_string(const std::string& s) -> InsStatus {
  if (s == "INS_INACTIVE")            return InsStatus::InsInactive;
  if (s == "INS_ALIGNING")            return InsStatus::InsAligning;
  if (s == "INS_HIGH_VARIANCE")       return InsStatus::InsHighVariance;
  if (s == "INS_SOLUTION_GOOD")       return InsStatus::InsSolutionGood;
  if (s == "INS_SOLUTION_FREE")       return InsStatus::InsSolutionFree;
  if (s == "INS_ALIGNMENT_COMPLETE")  return InsStatus::InsAlignmentComplete;
  if (s == "DETERMINING_ORIENTATION") return InsStatus::DeterminingOrientation;
  if (s == "WAITING_INITIALPOS")      return InsStatus::WaitingInitialPos;
  if (s == "WAITING_AZIMUTH")         return InsStatus::WaitingAzimuth;
  if (s == "INITIALIZING_BIASES")     return InsStatus::InitializingBiases;
  if (s == "MOTION_DETECT")           return InsStatus::MotionDetect;
  return InsStatus::Unknown;
}

/// pos_type 字符串 → 枚举（未匹配返回 None）
inline auto pos_type_from_string(const std::string& s) -> PosType {
  if (s == "INS_PSRSP")    return PosType::InsPsrsp;
  if (s == "INS_PSRDIFF")  return PosType::InsPsrdiff;
  if (s == "INS_RTKFLOAT") return PosType::InsRtkfloat;
  if (s == "INS_RTKFIXED") return PosType::InsRtkfixed;
  return PosType::None;
}

/// 设备解算质量 → rtk_status（纯设备质量映射，不受门控影响）
/// 与旧实现的关系：旧实现在门控分支里顺带写 rtk_status，导致未命中的解类型
/// 沿用上一帧；本函数让 rtk_status 始终反映本帧设备的解类型，门控只写
/// is_valid / fault_code，符合"rtk_status 纯粹反映设备解算质量"的既定设计。
constexpr auto rtk_status_from_pos_type(PosType t) -> RtkStatus {
  switch (t) {
    case PosType::InsRtkfixed: return RtkStatus::RtkFix;
    case PosType::InsRtkfloat: return RtkStatus::RtkFloat;
    case PosType::InsPsrsp:    return RtkStatus::Aposit;
    case PosType::InsPsrdiff:  return RtkStatus::RtcmDiff;
    default:                   return RtkStatus::RtkInit;
  }
}

}  // namespace gnss_bringup::loc
