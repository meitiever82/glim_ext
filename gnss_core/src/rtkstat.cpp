#include "gnss_core/rtkstat.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>

namespace gnss_core {

namespace {

std::vector<std::string> split(const std::string& s, char sep) {
  std::vector<std::string> out;
  size_t beg = 0;
  while (true) {
    const size_t pos = s.find(sep, beg);
    if (pos == std::string::npos) { out.push_back(s.substr(beg)); break; }
    out.push_back(s.substr(beg, pos - beg));
    beg = pos + 1;
  }
  return out;
}

// 严格数值解析:必须整段被消费完,否则视为非法(避免 "BADTOW" 被 strtod 静默当成 0)
bool to_double(const std::string& s, double& out) {
  if (s.empty()) return false;
  errno = 0;
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (errno == ERANGE || end == s.c_str() || *end != '\0') return false;
  out = v;
  return true;
}

bool to_int(const std::string& s, int& out) {
  double v = 0.0;
  if (!to_double(s, v)) return false;
  out = static_cast<int>(v);
  return true;
}

// $SAT,week,tow,sat,frq,az,el,resp,resc,vsat,snr,fix,slip,lock,outc,slipc,rejc
constexpr size_t kSatFields = 17;
constexpr size_t kIdxTow = 2, kIdxSat = 3, kIdxAz = 5, kIdxEl = 6, kIdxResp = 7;
constexpr size_t kIdxVsat = 9, kIdxSnr = 10, kIdxSlipc = 15, kIdxRejc = 16;

}  // namespace

bool parse_sat_line(const std::string& line, SatStat& out) {
  if (line.rfind("$SAT,", 0) != 0) return false;
  const auto f = split(line, ',');
  if (f.size() < kSatFields) return false;

  SatStat s;
  int vsat = 0;
  if (!to_double(f[kIdxTow], s.tow)) return false;
  if (!to_double(f[kIdxAz], s.az)) return false;
  if (!to_double(f[kIdxEl], s.el)) return false;
  if (!to_double(f[kIdxResp], s.resp)) return false;
  if (!to_double(f[kIdxSnr], s.snr)) return false;
  if (!to_int(f[kIdxVsat], vsat)) return false;
  if (!to_int(f[kIdxSlipc], s.slipc)) return false;
  if (!to_int(f[kIdxRejc], s.rejc)) return false;
  s.sat = f[kIdxSat];
  s.valid = vsat == 1;
  out = s;   // 只在全部字段合法后才落到 out
  return true;
}

void StatEpochAccumulator::feed(const std::string& line) {
  SatStat s;
  if (!parse_sat_line(line, s)) return;

  if (has_tow_ && s.tow != tow_) {
    // 新历元开始:把已完成的历元挪到 prev_,cur_ 清空。
    // epoch() 在 cur_ 为空时返回 prev_,因此消费者仍看到上一个完整历元。
    prev_ = std::move(cur_);
    cur_.clear();
    seen_.clear();
  }
  tow_ = s.tow;
  has_tow_ = true;

  if (std::find(seen_.begin(), seen_.end(), s.sat) != seen_.end()) return;   // 第二个频点
  seen_.push_back(s.sat);
  cur_.push_back(std::move(s));
}

const std::vector<SatStat>& StatEpochAccumulator::epoch() const {
  return cur_.empty() ? prev_ : cur_;
}

void StatEpochAccumulator::reset() {
  cur_.clear();
  prev_.clear();
  seen_.clear();
  tow_ = 0.0;
  has_tow_ = false;
}

void SlipWindow::feed(double t, const std::string& sat, int slipc) {
  auto it = std::find_if(last_.begin(), last_.end(),
                         [&](const std::pair<std::string, int>& e) { return e.first == sat; });
  if (it == last_.end()) {
    last_.emplace_back(sat, slipc);   // 首见:只立基准
    return;
  }
  if (slipc > it->second) hits_.emplace_back(t, slipc - it->second);
  it->second = slipc;
}

int SlipWindow::count(double now) {
  const double cutoff = now - window_;
  hits_.erase(std::remove_if(hits_.begin(), hits_.end(),
                             [&](const std::pair<double, int>& h) { return h.first < cutoff; }),
              hits_.end());
  int sum = 0;
  for (const auto& h : hits_) sum += h.second;
  return sum;
}

}  // namespace gnss_core
