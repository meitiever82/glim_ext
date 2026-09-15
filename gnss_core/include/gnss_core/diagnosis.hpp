#pragma once
// 诊断规则链(spec §3 C1–C9、§8)。纯函数,无 ROS / GLIM 依赖。
// 优先级、阈值默认值与消息语义移植自 rtk-monitor diagnosis/rules.py;
// 与参考实现的差异见 docs/gnss/plans/2026-09-15-round3a-diagnosis-core.md「设计决定」。
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "gnss_core/rtkstat.hpp"
#include "gnss_core/types.hpp"

namespace gnss_core {

enum class Level { Ok, Info, Warning, Serious, Critical };
const char* level_name(Level level);
// 只有 warning / serious / critical 会开事件;ok / info 只是状态
bool level_opens_event(Level level);
const char* quality_name(Quality q);

struct Verdict {
  Level level = Level::Ok;
  std::string code;
  std::string message;
};

struct ControlPoint {
  std::string name;
  double lat = 0.0, lon = 0.0;   // WGS-84 deg;只做水平比较
};

// 一路定位解(rtkrcv 独立解或 610 融合解)在诊断里用到的字段
struct SolutionSample {
  Quality quality = Quality::NONE;
  double lat = 0.0, lon = 0.0;   // deg
  int ns = 0;                    // 参与解算卫星数
  double sdn = 0.0, sde = 0.0;   // m
  double age = 0.0;              // 差分龄期 s
  std::optional<double> ratio;   // AR ratio;源不提供时为空,ambiguity 规则不触发
  // 该解的历元时刻(UTC unix 秒,不是到达时刻);3b 从 RtkFix.gnss_time(> 0 时)填入。
  // 两路都带历元时刻时 device_divergence 按历元配对(见 divergence_epoch_max_dt_s),
  // 否则退回按到达时刻配对(divergence_pair_max_dt_s)。
  std::optional<double> epoch_t;
};

struct DiagnosisConfig {
  double corr_gap_s = 3.0;
  double age_max_s = 10.0;
  double base_shift_m = 0.1;
  int min_sats = 6;
  double resid_max_m = 2.0;
  double low_el_deg = 20.0;
  double low_snr_dbhz = 35.0;
  double min_ratio = 3.0;
  int slip_max_per_30s = 5;
  double divergence_sigma = 3.0;
  double divergence_hold_s = 5.0;
  double close_hysteresis_s = 10.0;
  double sol_stale_s = 5.0;
  double abs_ref_max_m = 0.2;
  double abs_ref_radius_m = 3.0;
  // 以下为本项目新增(rtk-monitor 没有)
  double divergence_window_s = 600.0;       // 经验 σ 的滑动窗口
  int divergence_min_samples = 60;          // 窗口样本够这么多才有经验基线(之前是预热期)
  double divergence_sigma_floor_m = 0.05;   // 阈值 σ 下限,任何时候都生效,两路几乎重合时防误报
  double divergence_pair_max_dt_s = 2.0;    // 按到达时刻配对(任一路缺历元时刻)时,到达时刻相差 >= 此值不配对
  double divergence_epoch_max_dt_s = 0.1;   // 按历元时刻配对时,最近的历元相差超过此值不配对
  double base_warmup_s = 600.0;             // 基站基线预热时长(取中位数)
  // 学到的 σ(窗口 RMS 与 held 基线)上限:阈值里来自学习的部分最多 divergence_sigma × 此值,
  // 更大的持续偏差不会被学成正常。rtkrcv 当前自报 σ 不受此限。
  double divergence_sigma_max_m = 0.10;
};

// 任何字段非法时抛 std::invalid_argument,消息以字段名开头
void validate_diagnosis_config(const DiagnosisConfig& cfg);

// 每秒一次的规则输入。时间一律是 UTC unix 秒。
struct DiagnosisInput {
  double now = 0.0;
  std::optional<double> corr_last_t;         // 最近一次收到任何差分字节的时刻
  std::optional<double> corr_age;            // 差分龄期 s
  std::optional<double> base_offset_m;       // 基站相对基线的 ECEF 位移
  std::optional<SolutionSample> sol;         // 独立解(调用方已按 sol_stale_s 过滤)
  std::vector<SatStat> sats;                 // 当前 $SAT 历元(调用方已过滤新鲜度)
  int slip_count_30s = 0;
  std::optional<double> divergence_m;        // 610 与独立解的水平偏差
  std::optional<double> divergence_since;    // 偏差首次超限时刻(DivergenceMonitor 给)
  double divergence_threshold_m = 0.0;       // 当前超限阈值(DivergenceMonitor 给)
  bool solver_enabled = true;
  std::vector<ControlPoint> control_points;
};

struct DiagnosisResult {
  // 全部命中的结论,按规则优先级排序(优先级最高的在前);状态结论(no_solution / not_fixed /
  // rtk_fixed)总是追加在最后。例外:no_data 命中时只有这一条。
  std::vector<Verdict> verdicts;
  // 第一条(优先级最高的)结论;没有任何故障命中时就是追加在最后的那条状态结论
  const Verdict& status() const { return verdicts.front(); }
};

DiagnosisResult evaluate_rules(const DiagnosisInput& in, const DiagnosisConfig& cfg);

}  // namespace gnss_core
