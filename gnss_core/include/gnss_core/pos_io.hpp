#pragma once
#include <string>
#include <vector>
#include <Eigen/Core>
#include "gnss_core/types.hpp"

namespace gnss_core {

// RTKLIB .pos 一条记录(spec §5.3)。
struct PosRecord {
  double stamp = 0.0;           // UTC unix seconds(read_pos 已统一换算)
  double lat = 0.0, lon = 0.0, height = 0.0;   // WGS-84 deg/deg/m(椭球高)
  int q = 0;                    // RTKLIB Q: 1=fix 2=float 4=dgps 5=single
  int ns = 0;                   // 卫星数
  Eigen::Vector3d sdne = Eigen::Vector3d::Zero();   // sdn, sde, sdu(m)
  double age = 0.0;             // 差分龄期(s)
  double ratio = 0.0;           // AR ratio
};

// 时间系统:RTKLIB .pos 默认 GPST(比 UTC 快 leap_seconds),头部 "% (... time=GPST)" / "(... time=UTC)" 标明。
// read_pos 统一输出 UTC unix 秒:GPST 减 leap_seconds;UTC 原样;无头部标注时按 default_time_system。
enum class PosTimeSystem { GPST, UTC };

struct PosReadOptions {
  int leap_seconds = 18;
  PosTimeSystem default_time_system = PosTimeSystem::GPST;
};

// 读取 .pos:跳过 % 注释行(但扫描其中 time=GPST/UTC);
// 数据列 "YYYY/MM/DD HH:MM:SS.sss lat lon height Q ns sdn sde sdu sdne sdeu sdun age ratio"。
// 列数不足 10(到 sdu)的行跳过;age/ratio 缺省为 0。文件打不开抛 std::runtime_error。
std::vector<PosRecord> read_pos(const std::string& path, const PosReadOptions& opt = {});

// 写标准 RTKLIB .pos(头 "% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,...,time=GPST|UTC)" + 列名注释 + 14 列数据)。
// records.stamp 为 UTC unix 秒;time_system=GPST 时写出时间加 leap_seconds。
void write_pos(const std::string& path, const std::vector<PosRecord>& records,
               PosTimeSystem time_system = PosTimeSystem::GPST, int leap_seconds = 18);

// RTKLIB Q → 归一化质量:1→FIXED 2→FLOAT 4→DGPS 5→SINGLE 其它→NONE
Quality q_to_quality(int q);

// 1 Hz 抽稀(spec §5.3:rosbag2 存全量原始流,.pos 只存 1 Hz 摘要)。
// 按 floor(stamp / period) 分桶,每个桶只放行第一条 —— 桶边界对齐整秒,
// 因此输出的时间戳分布与 RTKLIB 1 Hz .pos 一致,且对抖动与时钟回跳都不会卡死
// (回跳落进更早的桶,算作新桶直接放行,而不是等到"上次 + 1 s"才恢复)。
class PosDecimator {
public:
  explicit PosDecimator(double period_s = 1.0) : period_(period_s) {}
  // 该条应当写出则返回 true
  bool accept(const PosRecord& r);
  void reset() { has_bin_ = false; }

private:
  double period_;
  long long bin_ = 0;
  bool has_bin_ = false;
};

}  // namespace gnss_core
