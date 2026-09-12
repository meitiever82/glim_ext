#pragma once
#include <string>
#include <vector>
#include "gnss_core/pos_io.hpp"

namespace gnss_core {

// rtkrcv 解状态流(.stat / -r 2 输出)里的一行 $SAT。
// RTKLIB 列序:$SAT,week,tow,sat,frq,az,el,resp,resc,vsat,snr,fix,slip,lock,outc,slipc,rejc
struct SatStat {
  double tow = 0.0;        // GPS 周内秒
  std::string sat;         // 卫星号,如 "G05"
  double az = 0.0, el = 0.0;   // deg
  double resp = 0.0;       // 伪距残差 (m) —— multipath 规则的输入
  double snr = 0.0;        // dBHz
  bool valid = false;      // vsat:该星是否参与解算
  int slipc = 0;           // 累计周跳次数 —— cycle_slip 规则的输入
  int rejc = 0;            // 累计被剔除次数
};

// 解析一行 $SAT;非 $SAT 行、列数不足或数值非法时返回 false(out 不变)。
bool parse_sat_line(const std::string& line, SatStat& out);

// 把 $SAT 行流按 tow 聚成"一个历元的各卫星状态"。
// 同一颗星只保留第一个频点。历元切换时,epoch() 仍返回上一个**完整**历元,
// 直到新历元收到第一颗星为止 —— 消费者因此不会在历元边界看到空列表闪一下。
class StatEpochAccumulator {
public:
  void feed(const std::string& line);
  const std::vector<SatStat>& epoch() const;
  double epoch_tow() const { return tow_; }
  void reset();

private:
  std::vector<SatStat> cur_, prev_;
  std::vector<std::string> seen_;
  double tow_ = 0.0;
  bool has_tow_ = false;
};

// 在滑动窗口内统计所有卫星的周跳增量之和(spec §3 C7 cycle_slip)。
class SlipWindow {
public:
  explicit SlipWindow(double window_s = 30.0) : window_(window_s) {}
  // 首次见到某颗星只记录基准值,不计增量(否则重连后会误报一大批)
  void feed(double t, const std::string& sat, int slipc);
  int count(double now);

private:
  double window_;
  std::vector<std::pair<std::string, int>> last_;
  std::vector<std::pair<double, int>> hits_;
};

// 解析 rtkrcv 的 llh 解流一行(列序与 RTKLIB .pos 数据行相同)。
// 输出 out.stamp 为 UTC unix 秒:opt.default_time_system=GPST 时减闰秒。
// 注释行(以 % 开头)、空行、列数不足返回 false。
bool parse_llh_solution(const std::string& line, PosRecord& out, const PosReadOptions& opt = {});

}  // namespace gnss_core
