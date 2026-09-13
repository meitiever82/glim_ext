#pragma once
#include <cmath>
#include <optional>
#include <string>
#include "gnss_core/pos_io.hpp"

namespace gnss_bringup {

// pos_writer_node 自己的、跟 ROS 无关的纯校验逻辑,抽出来是因为这两条都是
// 这个节点本身的、独立可测的判断——与 rtcm_bridge_params.hpp /
// rtk_fix_mapping.hpp 里其它纯校验函数同样的处理方式。

// ---------- time_system 参数 ----------
// 只接受 "GPST"/"UTC" 两个字面量,其它值(拼写错误、大小写不对……)必须在
// 启动时报清楚,而不是被 PosWriter 悄悄当成某个默认值用错时间系统写出去
// ——.pos 文件的时间列因此整体偏移 leap_seconds 秒,而且不会有任何报错,
// 只有下游拿去跟其它时间源比对时才会发现,现场很难排查。
inline std::optional<gnss_core::PosTimeSystem> parse_pos_time_system(const std::string& s) {
  if (s == "GPST") return gnss_core::PosTimeSystem::GPST;
  if (s == "UTC") return gnss_core::PosTimeSystem::UTC;
  return std::nullopt;
}

// ---------- leap_seconds 参数 ----------
// GPST-UTC 闰秒差,当前(2026)是 18,只会随时间缓慢增加,几十年内不会到 30。
// 负数或离谱大的值(拼错成秒数、错填成端口号之类)如果放过去,只会让
// .pos 的时间列整体偏移一个不合理的量,且没有任何报错——与 time_system
// 校验同一个道理:提前挡住、报得清楚。上限取 60 是一个远超任何可预见真实
// 值、但仍能挡住"明显打错"的宽松边界。
inline bool is_sane_leap_seconds(int leap_seconds) {
  return leap_seconds >= 0 && leap_seconds <= 60;
}

// ---------- record 时间戳的"合理性"闸门 ----------
// 上一个任务(rtk_fix_mapping::to_pos_record)在 gnss_time 和 header.stamp
// 都是 0 的极端输入下会把 0 原样传下来——这是它明确的、有意的决定(见该
// 头文件里的注释),不是本任务要修的 bug。pos_rotation::pos_path_for 对此
// 同样是"合法但离谱"的处理方式:正常换算,落进肉眼可见的 19700101 目录。
//
// 但这两个决定加在一起,到了本节点这一层,会变成"每次板卡两个时间字段都
// 缺失,就悄悄在 <root>/19700101/ 下建一个目录并往里写数据"——这不是任何
// 一个下游使用者会去看的地方,等于数据静默丢失在一个没人会想起来查的目录
// 里,比"跳过并报警"更危险。本任务在这一层加一道闸门:只有落在
// [kMinSaneUtcStamp, kMaxSaneUtcStamp] 区间内的有限时间戳才会被继续处理
// (决定/写文件);区间之外的(含 0、NaN、inf、任何离今天几十年开外的值)
// 一律跳过这条记录、报警(节流,不刷屏),不落盘、不建目录。
//
// 区间选得足够宽松,不会误伤任何真实数据:下界是 2000-01-01T00:00:00Z
// (GPS/GNSS 普及之前的时间不可能是真实的解算历元或者接收时刻),上界是
// 2100-01-01T00:00:00Z。这个闸门还顺带挡住了 NaN/±inf——floor(NaN/period)
// 转 long long(PosDecimator::accept 内部做的事)是未定义行为,必须在
// 调用 accept() 之前挡住,不能指望 pos_path_for 事后兜底(accept() 比
// pos_path_for 更早被调用)。
inline constexpr double kMinSaneUtcStamp = 946684800.0;   // 2000-01-01T00:00:00Z
inline constexpr double kMaxSaneUtcStamp = 4102444800.0;  // 2100-01-01T00:00:00Z

// round 2 review 的 Minor:丢弃时的日志原来一律写"gnss_time 与 header.stamp
// 都缺失或异常",但一个落在合理区间之外、却既非 0 也非非有限数的时间戳
// (比如 1990 年的某个真实 unix 秒——两个字段都被填了,只是填的时间不对,
// 或者是某种单位/换算错误)用这句话描述是误导的:它暗示"字段缺失",但
// 实际情况是"字段都在,只是不像真实的 GNSS 解算时刻"。分开分类,让日志
// 说得准确,操作人员据此能判断该去查驱动的时间戳来源还是查换算逻辑。
enum class StampSanity {
  kOk,          // 落在合理区间内
  kNonFinite,   // NaN/±inf——两个字段都没给,或者上游算出了非数
  kNearEpoch,   // 接近 0——gnss_time 与 header.stamp 都缺失时的典型特征
  kOutOfRange,  // 有限、不接近 0,但落在 [kMin, kMax] 区间之外——字段都在,
                // 只是不像真实的 GNSS 解算时刻(单位错/换算错/钟不对之类)
};

// 0 附近的判据:两个字段都缺失时 to_pos_record 会原样传下 0.0;留一点点
// 余量(±1 秒)而不是严格等于 0.0,同样能覆盖"缺失"这个语义,又不必用
// bit-exact 比较。
inline constexpr double kNearEpochToleranceS = 1.0;

inline StampSanity classify_utc_stamp(double utc_stamp) {
  if (!std::isfinite(utc_stamp)) return StampSanity::kNonFinite;
  if (std::fabs(utc_stamp) < kNearEpochToleranceS) return StampSanity::kNearEpoch;
  if (utc_stamp < kMinSaneUtcStamp || utc_stamp > kMaxSaneUtcStamp) return StampSanity::kOutOfRange;
  return StampSanity::kOk;
}

inline bool is_sane_utc_stamp(double utc_stamp) {
  return classify_utc_stamp(utc_stamp) == StampSanity::kOk;
}

// 人类可读的分类说明,供节点拼日志用——分类逻辑只写一遍,文案也只写一遍。
inline const char* describe_stamp_sanity(StampSanity s) {
  switch (s) {
    case StampSanity::kNonFinite:
      return "不是有限数(NaN/inf)——两个时间字段大概率都没给,或者上游算出了非数";
    case StampSanity::kNearEpoch:
      return "接近 UTC 纪元零点——gnss_time 与 header.stamp 大概率都缺失";
    case StampSanity::kOutOfRange:
      return "是有限数,但不落在合理的 GNSS 时间范围内——字段本身有值,"
             "但不像真实的解算/接收时刻(检查单位换算或时钟source)";
    case StampSanity::kOk:
      return "合理";
  }
  return "未知";
}

// ---------- period_s 参数(1 Hz 抽稀周期)----------
// round 2 review 的 Minor(promoted):period_s 只校验"正数且有限"是不够的
// ——period_s=1e-12 一样能通过那道校验,但 PosDecimator::accept() 内部算
// floor(stamp / period_s) 时,一个正常量级的 stamp(~4.1e9)除以 1e-12 会
// 产生一个远超 long long 表示范围的浮点数,随后的 static_cast<long long> 是
// 未定义行为。下限取 1e-3(1 ms)——比这个节点存在的意义(1 Hz 摘要)快
// 1000 倍,足够宽松到不会误伤任何合理配置,同时远离溢出边界。
inline constexpr double kMinSanePeriodSeconds = 1e-3;

inline bool is_sane_period_seconds(double period_s) {
  return std::isfinite(period_s) && period_s >= kMinSanePeriodSeconds;
}

}  // namespace gnss_bringup
