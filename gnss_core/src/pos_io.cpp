#include "gnss_core/pos_io.hpp"
#include "gnss_core/rtkstat.hpp"   // parse_llh_solution 在此实现:与 .pos 数据行是同一套列解析

#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
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

bool PosDecimator::accept(const PosRecord& r) {
  const long long bin = static_cast<long long>(std::floor(r.stamp / period_));
  if (!has_bin_) {
    bin_ = bin;
    has_bin_ = true;
    return true;
  }
  if (bin > bin_) {
    // 桶号前进:正常的下一秒,无条件放行。
    bin_ = bin;
    return true;
  }
  if (bin_ - bin > kJitterToleranceBins) {
    // 桶号后退超过容忍范围——真实的时钟回跳(比如 ClockJumpBackwardsDoesNotStallOutput
    // 覆盖的 1000 秒回跳),必须继续输出,不能等追上才恢复。
    bin_ = bin;
    return true;
  }
  // 桶号没变,或者只后退了 <= kJitterToleranceBins 格:前者是同一秒内的
  // 重复,后者是抖动(在整秒边界两侧来回摆动)——两者都不是"新的一秒",丢弃。
  return false;
}

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
    // 数据行与 rtkrcv 的 llh 解流同格式,复用同一个解析器(spec §2.1 算法只写一遍)。
    // 时间系统用从头部扫出来的 ts,而不是 opt 里的默认值。
    PosReadOptions line_opt = opt;
    line_opt.default_time_system = ts;
    PosRecord r;
    if (parse_llh_solution(line, r, line_opt)) out.push_back(r);
  }
  return out;
}

std::string pos_header(PosTimeSystem time_system) {
  std::string h;
  h += "% program   : gnss_core write_pos\n";
  h += "% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,4:dgps,5:single, time=";
  h += (time_system == PosTimeSystem::GPST ? "GPST" : "UTC");
  h += ")\n";
  h += "%  ";
  h += (time_system == PosTimeSystem::GPST ? "GPST" : "UTC ");
  h += "                  latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n";
  return h;
}

std::string format_pos_record(const PosRecord& r, PosTimeSystem time_system, int leap_seconds) {
  double t = r.stamp;
  if (time_system == PosTimeSystem::GPST) t += static_cast<double>(leap_seconds);
  // 拆成整秒 + 毫秒,毫秒四舍五入并处理进位
  double whole = std::floor(t);
  int ms = static_cast<int>(std::lround((t - whole) * 1000.0));
  if (ms >= 1000) { ms -= 1000; whole += 1.0; }
  const std::time_t tt = static_cast<std::time_t>(whole);
  std::tm tm{};
  gmtime_r(&tt, &tm);
  char buf[256];
  std::string line;
  std::snprintf(buf, sizeof(buf), "%04d/%02d/%02d %02d:%02d:%02d.%03d",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ms);
  line += buf;
  std::snprintf(buf, sizeof(buf), " %14.9f %14.9f %10.4f %3d %3d %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %6.2f %6.1f\n",
                r.lat, r.lon, r.height, r.q, r.ns, r.sdne(0), r.sdne(1), r.sdne(2), 0.0, 0.0, 0.0, r.age, r.ratio);
  line += buf;
  return line;
}

void write_pos(const std::string& path, const std::vector<PosRecord>& records,
               PosTimeSystem time_system, int leap_seconds) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("write_pos: cannot open " + path);

  out << pos_header(time_system);
  for (const auto& r : records) {
    out << format_pos_record(r, time_system, leap_seconds);
  }
}

PosWriter::~PosWriter() { close(); }

bool PosWriter::open(const std::string& path) {
  close();
  std::error_code ec;
  const std::filesystem::path fp(path);
  const auto parent = fp.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, ec);   // 忽略"已存在";其余失败交给后面的 open 判定
  }
  // "新文件"指不存在或存在但长度为 0 —— 用这个判断是否需要写表头,
  // 因此进程重启接续一个已有非空文件时不会再写一遍表头。
  bool is_new = true;
  {
    std::error_code size_ec;
    const auto sz = std::filesystem::file_size(fp, size_ec);
    if (!size_ec) is_new = (sz == 0);
  }
  out_.open(path, std::ios::app);
  if (!out_.is_open()) return false;
  path_ = path;
  if (is_new) {
    out_ << pos_header(ts_);
    out_.flush();
  }
  return true;
}

bool PosWriter::write(const PosRecord& r) {
  if (!out_.is_open()) return false;
  out_ << format_pos_record(r, ts_, leap_);
  out_.flush();
  return static_cast<bool>(out_);
}

void PosWriter::close() {
  if (out_.is_open()) out_.close();
}

bool parse_llh_solution(const std::string& line, PosRecord& out, const PosReadOptions& opt) {
  // 空行 / 全空白 / 注释行不是数据
  const size_t first = line.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return false;
  if (line[first] == '%') return false;

  std::istringstream ss(line);
  std::string date, time;
  PosRecord r;
  double sdne_ = 0, sdeu_ = 0, sdun_ = 0;
  if (!(ss >> date >> time >> r.lat >> r.lon >> r.height >> r.q >> r.ns
           >> r.sdne(0) >> r.sdne(1) >> r.sdne(2))) {
    return false;   // 列数不足
  }
  // 可选列:sdne sdeu sdun age ratio(缺省保持 0)
  ss >> sdne_ >> sdeu_ >> sdun_ >> r.age >> r.ratio;

  double stamp = 0.0;
  if (!parse_date_time(date, time, stamp)) return false;
  if (opt.default_time_system == PosTimeSystem::GPST) stamp -= static_cast<double>(opt.leap_seconds);
  r.stamp = stamp;
  out = r;
  return true;
}

}  // namespace gnss_core
