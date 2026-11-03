#pragma once
// Judge: 定位门控判决纯函数（GNSS 生产线 §3.4）。
// 从 localization_parser 的 Beiyun/HuaCe processor 抽出的门控规则，纯函数、无 ROS。
// Evidence 是"一帧证据"（由适配器从品牌报文归一化），GateState 显式持有跨帧状态
// （粘滞/宽限计时），Verdict 是判决输出。
//
// 行为等价由 golden parity 锚定（localization/golden/）：本实现的逐帧输出必须与
// 旧 processor 在相同输入+相同配置下的输出一致（keep 类用例）。
//
// 枚举值域与 localization_parser/core/common/types.hpp 对齐（fault_code 契约
// 同步 robominer_msgs/msg/LocalizationDataAux.msg 注释）。
#include <cstdint>
#include <optional>
#include <string>

namespace gnss_core::judge {

// 与 localization_parser common::RtkStatus 同值域
enum class RtkStatus : uint8_t {
  RtkInit = 0, Aposit = 1, RtcmDiff = 2, NotUsed = 3,
  RtkFix = 4, RtkFloat = 5, RtkOther = 6, DataTimeout = 11,
};

// 与 localization_parser common::FaultCode 同值域
enum class FaultCode : uint8_t {
  None = 0, InsNotConverged = 1, GraceTimeout = 2, HardwareWarning = 3,
  SatelliteInsufficient = 4, BaselineAbnormal = 5, EnuOutOfBounds = 6,
  ParseFail = 7, DataTimeout = 8, NoData = 9,
};

// 多因并存保留优先级最高者（枚举值越大优先级越高，对齐 RaiseFaultCode）
inline void RaiseFault(FaultCode& cur, FaultCode next) {
  if (static_cast<uint8_t>(next) > static_cast<uint8_t>(cur)) cur = next;
}

// 归一化的定位类型（适配器把品牌字符串/位域映射到这里）
enum class PosType : uint8_t { None = 0, Single, Psrdiff, RtkFloat, RtkFixed };

struct Config {
  // 共用硬门控
  int xyz_threshold = 10000;        // ENU 越界阈值 m
  int stars_threshold = 10;         // 最少卫星数
  // 北云
  double sticky_grace_time = 30.0;  // 粘滞宽限 s；<=0 立即判无效
  double single_point_age_threshold = 10.0;
  double heading_length = 1.6;      // 双天线标称基线 m
  double heading_length_threshold = 0.2;
  // 华测
  double unfixed_time = 5.0;
  double combined_nav_mod_time = 30.0;
  double rtk_float_grace_time = 20.0;
};

// 一帧证据（适配器归一化后的输入）
struct Evidence {
  double now = 0.0;                 // 注入时间（回放确定性；秒，单调）
  // 归一化解状态
  PosType pos_type = PosType::None;
  bool ins_good = false;            // 北云: ins_status==INS_SOLUTION_GOOD；华测: sta低4位==2
  double diff_age = 0.0;            // 差分龄期 s（北云 GGA / 华测 GPCHC）
  // 位置（ENU，由调用方完成 BLH→ENU 换算——坐标换算不在 Judge 职责内）
  double enu_x = 0.0, enu_y = 0.0, enu_z = 0.0;
  // 卫星数（北云: GPFPD nsv1 + HEADINGA obs；华测: nsv1/nsv2）
  uint16_t sats_main = 0;
  uint16_t sats_aux = 0;
  // 北云双天线基线（仅北云用）
  bool heading_narrow_int = false;
  double heading_length = 0.0;
  bool heading_valid_frame = false; // 本帧 HEADINGA 是否收到过
  // 华测硬件告警（仅华测用）：warning & 0x1FD 的判定结果由适配器算好
  bool hw_warning = false;
  // 华测 GNSS 原始定位类型（sta 高 4 位，rtk_status 无条件透传用）
  uint8_t huace_gnss_pos_type = 0;
  // 华测专用：GPS 消息正常位（warning bit0 == 0）
  bool huace_gnss_pos_valid = true;
};

// 显式跨帧状态（粘滞/宽限计时摆上台面，可测试可回放）
struct GateState {
  // 北云粘滞
  bool in_sticky = false;
  double sticky_start = 0.0;
  // 华测固定解时间基线
  bool has_fix_time_base = false;
  double last_rtkfixed_t = 0.0;
  // 两品牌共用：上一帧判决（粘滞语义的本体）
  bool prev_valid = false;
  RtkStatus prev_rtk = RtkStatus::RtkInit;
  FaultCode prev_fault = FaultCode::None;
};

struct Verdict {
  bool is_valid = false;
  RtkStatus rtk_status = RtkStatus::RtkInit;
  FaultCode fault_code = FaultCode::None;
};

// 北云判决（对齐 beiyun_processor.cpp::CheckValidity）
Verdict judge_beiyun(const Evidence& e, GateState& s, const Config& cfg);

// 华测判决（对齐 huace_processor.cpp::CheckValidity）
Verdict judge_huace(const Evidence& e, GateState& s, const Config& cfg);

}  // namespace gnss_core::judge
