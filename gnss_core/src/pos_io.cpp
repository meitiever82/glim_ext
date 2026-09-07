#include "gnss_core/pos_io.hpp"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace gnss_core {

namespace {

// "YYYY/MM/DD" + "HH:MM:SS.sss" → 按 UTC 日历合成 unix 秒。
// 用 timegm 而非 mktime:后者受本机 TZ 影响。
bool parse_date_time(const std::string& date, const std::string& time, double& out) {
  int Y = 0, M = 0, D = 0, h = 0, m = 0;
  double s = 0.0;
  if (std::sscanf(date.c_str(), "%d/%d/%d", &Y, &M, &D) != 3) return false;
  if (std::sscanf(time.c_str(), "%d:%d:%lf", &h, &m, &s) != 3) return false;
  std::tm tm{};
  tm.tm_year = Y - 1900;
  tm.tm_mon = M - 1;
  tm.tm_mday = D;
  tm.tm_hour = h;
  tm.tm_min = m;
  tm.tm_sec = 0;
  const std::time_t base = timegm(&tm);
  if (base == static_cast<std::time_t>(-1)) return false;
  out = static_cast<double>(base) + s;
  return true;
}

}  // namespace

Quality q_to_quality(int q) {
  switch (q) {
    case 1: return Quality::FIXED;
    case 2: return Quality::FLOAT;
    case 4: return Quality::DGPS;
    case 5: return Quality::SINGLE;
    default: return Quality::NONE;
  }
}

std::vector<PosRecord> read_pos(const std::string& path, const PosReadOptions& opt) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("read_pos: cannot open " + path);

  PosTimeSystem ts = opt.default_time_system;
  std::vector<PosRecord> out;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    if (line[0] == '%') {
      if (line.find("time=GPST") != std::string::npos) ts = PosTimeSystem::GPST;
      else if (line.find("time=UTC") != std::string::npos) ts = PosTimeSystem::UTC;
      else {
        // RTKLIB 列名行:"%  GPST  latitude(deg) ..." / "%  UTC ...",第一个 token 即时间系统
        std::istringstream hs(line.substr(1));
        std::string tok;
        if (hs >> tok) {
          if (tok == "GPST") ts = PosTimeSystem::GPST;
          else if (tok == "UTC") ts = PosTimeSystem::UTC;
        }
      }
      continue;
    }
    std::istringstream ss(line);
    std::string date, time;
    PosRecord r;
    double sdne_ = 0, sdeu_ = 0, sdun_ = 0;
    if (!(ss >> date >> time >> r.lat >> r.lon >> r.height >> r.q >> r.ns
             >> r.sdne(0) >> r.sdne(1) >> r.sdne(2))) {
      continue;   // 列数不足或非数据行,跳过
    }
    // 可选列:sdne sdeu sdun age ratio
    ss >> sdne_ >> sdeu_ >> sdun_ >> r.age >> r.ratio;
    double stamp = 0.0;
    if (!parse_date_time(date, time, stamp)) continue;
    if (ts == PosTimeSystem::GPST) stamp -= static_cast<double>(opt.leap_seconds);
    r.stamp = stamp;
    out.push_back(r);
  }
  return out;
}

void write_pos(const std::string& path, const std::vector<PosRecord>& records,
               PosTimeSystem time_system, int leap_seconds) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("write_pos: cannot open " + path);

  out << "% program   : gnss_core write_pos\n";
  out << "% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,4:dgps,5:single, time="
      << (time_system == PosTimeSystem::GPST ? "GPST" : "UTC") << ")\n";
  out << "%  " << (time_system == PosTimeSystem::GPST ? "GPST" : "UTC ")
      << "                  latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n";

  char buf[256];
  for (const auto& r : records) {
    double t = r.stamp;
    if (time_system == PosTimeSystem::GPST) t += static_cast<double>(leap_seconds);
    // 拆成整秒 + 毫秒,毫秒四舍五入并处理进位
    double whole = std::floor(t);
    int ms = static_cast<int>(std::lround((t - whole) * 1000.0));
    if (ms >= 1000) { ms -= 1000; whole += 1.0; }
    const std::time_t tt = static_cast<std::time_t>(whole);
    std::tm tm{};
    gmtime_r(&tt, &tm);
    std::snprintf(buf, sizeof(buf), "%04d/%02d/%02d %02d:%02d:%02d.%03d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ms);
    out << buf;
    std::snprintf(buf, sizeof(buf), " %14.9f %14.9f %10.4f %3d %3d %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %6.2f %6.1f\n",
                  r.lat, r.lon, r.height, r.q, r.ns, r.sdne(0), r.sdne(1), r.sdne(2), 0.0, 0.0, 0.0, r.age, r.ratio);
    out << buf;
  }
}

}  // namespace gnss_core
