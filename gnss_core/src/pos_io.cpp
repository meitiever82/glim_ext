#include "gnss_core/pos_io.hpp"
#include "gnss_core/pos_io_test_hooks.hpp"
#include "gnss_core/rtkstat.hpp"   // parse_llh_solution 在此实现:与 .pos 数据行是同一套列解析

#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace gnss_core {

namespace {

// t(UTC unix 秒,已经按需加过闰秒)→ 整秒 + 毫秒,四舍五入并处理进位。
// format_pos_record 与 PosWriter 的去重键(record_time_key_ms)共用同一份
// 取整逻辑——两处一旦出现哪怕最后一位的差异,"这一行在文件里占的那个
// 毫秒"和"判断重复用的那个毫秒"就不再是同一个东西,去重会出现系统性错位。
void split_seconds_ms(double t, std::time_t& whole_out, int& ms_out) {
  double whole = std::floor(t);
  int ms = static_cast<int>(std::lround((t - whole) * 1000.0));
  if (ms >= 1000) { ms -= 1000; whole += 1.0; }
  whole_out = static_cast<std::time_t>(whole);
  ms_out = ms;
}

// 一条记录在给定时间系统/闰秒下,渲染进 .pos 里的那个毫秒级时刻——用作
// PosWriter 的去重键(见 pos_io.hpp 类注释)。跟 format_pos_record 是
// 同一套换算(经 split_seconds_ms 共用实现)。
long long record_time_key_ms(const PosRecord& r, PosTimeSystem ts, int leap_seconds) {
  double t = r.stamp;
  if (ts == PosTimeSystem::GPST) t += static_cast<double>(leap_seconds);
  std::time_t whole;
  int ms;
  split_seconds_ms(t, whole, ms);
  return static_cast<long long>(whole) * 1000 + ms;
}

// round 2 review 的 BLOCKING 1/2(final-fix-report.md 之后又发现的问题):
// 这个函数原来只看"读到的最后一个字节是不是 '\n'",完全没检查那次读取
// 本身有没有成功——一次瞬时 I/O 错误(EIO)会让 in.get() 失败、last 保持
// 默认值 0,0 != '\n',于是被误判成"最后一行不完整",接着拿一份实际上
// 没读到的 content 去算截断点。reviewer 用 LD_PRELOAD 在真实二进制上复现:
// 一次瞬时 EIO 让 1000 行的文件被截到 1 字节,表头和全部数据行都没了。
// 另一半(BLOCKING 2):文件只有写权限、没有读权限时(比如 chmod 200),
// 旧代码里"读失败"和"不需要截断"被同一个 false 返回值混在一起,调用方
// 分不清,于是当成"不需要截断"继续往下 append,把新记录粘连到还没写完的
// 半行上——跟完全没做这个截断修复时一模一样的 bug A 又回来了。
//
// 现在返回一个三态结果:任何一步(读最后一个字节、读整份内容、真正
// resize_file)失败,都归为 kFailed——调用方必须把它当成 open() 本身失败
// (返回 false,不截断、不 append、不抛),绝不能猜"大概不需要截断"就
// 继续往下走。
enum class TrailingLineOutcome {
  kNoTruncationNeeded,  // 文件为空,或者最后一行本来就完整,不用动
  kTruncated,           // 确认最后一行不完整,已经成功原地截掉
  kFailed,              // 检查/截断过程本身失败——调用方必须让 open() 直接失败
};

// 仅供 test_pos_io.cpp 通过 pos_io_test_hooks.hpp 注入;生产环境恒为
// nullptr,open() 里只多一次判空,不影响任何逐条记录的热路径。
gnss_core::testing::TrailingLineReadFailureInjector g_trailing_line_read_failure_injector = nullptr;

bool inject_read_failure(const char* step) {
  return g_trailing_line_read_failure_injector != nullptr && g_trailing_line_read_failure_injector(step);
}

// 一次截断在“最后一行没写完整”这个前提下,理论上能去掉的字节数不会超过
// 一行的最坏长度——format_pos_record 的列宽是固定的(日期时间 24 字节 +
// 13 个定宽数值列),正常情况下远小于这个上限。用一个宽松的常数而不是
// 精确算出的行宽,一是格式将来略微调整时不用跟着改这里,二是作为纵深
// 防御:即便"读到的字节数恰好等于文件大小"这个校验也被蒙混过去(比如
// 某种读错误凑巧返回了等长但错误的数据),截断量一旦远超"一行"的量级,
// 就说明拿到的 content 根本不可信,拒绝截断远比截掉不该截的东西安全。
constexpr std::uintmax_t kMaxPlausibleIncompleteLineBytes = 4096;

TrailingLineOutcome truncate_incomplete_trailing_line(const std::filesystem::path& fp) {
  std::error_code size_ec;
  const auto file_size = std::filesystem::file_size(fp, size_ec);
  if (size_ec) return TrailingLineOutcome::kFailed;
  if (file_size == 0) return TrailingLineOutcome::kNoTruncationNeeded;

  std::ifstream in(fp, std::ios::binary);
  if (!in) return TrailingLineOutcome::kFailed;   // 比如没有读权限(BLOCKING 2 的 chmod 200 场景)

  in.seekg(static_cast<std::streamoff>(file_size) - 1);
  char last = 0;
  in.get(last);
  // 这一步失败(EIO/短读)绝不能被当成"读到了 0",必须立刻当成失败处理——
  // 这正是 BLOCKING 1 复现命中的那一行。
  if (!in || inject_read_failure("last_byte")) return TrailingLineOutcome::kFailed;
  if (last == '\n') return TrailingLineOutcome::kNoTruncationNeeded;   // 最后一行完整,不用截断

  in.clear();
  in.seekg(0);
  if (!in) return TrailingLineOutcome::kFailed;
  const std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const bool stream_had_io_error = in.bad();
  in.close();
  // 第二道防线,比单看流状态更可靠:istreambuf_iterator 读到 EOF 不会
  // 设置 failbit,一次真正的中途 I/O 错误未必会体现在流状态上——但读到的
  // 字节数一定会跟文件大小对不上。两者任一为真都不可信。
  if (stream_had_io_error || content.size() != static_cast<std::uintmax_t>(file_size) ||
      inject_read_failure("full_content")) {
    return TrailingLineOutcome::kFailed;
  }

  const auto pos_nl = content.find_last_of('\n');
  const std::uintmax_t old_size = static_cast<std::uintmax_t>(file_size);
  const std::uintmax_t new_size = (pos_nl == std::string::npos) ? 0 : static_cast<std::uintmax_t>(pos_nl + 1);
  if (old_size - new_size > kMaxPlausibleIncompleteLineBytes) {
    // 算出来的截断量比一整行还大得多——content 不可信,拒绝截断。
    return TrailingLineOutcome::kFailed;
  }

  std::error_code trunc_ec;
  std::filesystem::resize_file(fp, new_size, trunc_ec);
  if (trunc_ec) return TrailingLineOutcome::kFailed;   // 截断本身失败(比如没有写权限)
  return TrailingLineOutcome::kTruncated;
}

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

namespace testing {
void set_trailing_line_read_failure_injector(TrailingLineReadFailureInjector injector) {
  g_trailing_line_read_failure_injector = injector;
}
}  // namespace testing

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
  // 拆成整秒 + 毫秒(与 PosWriter 的去重键共用同一份取整/进位逻辑)
  std::time_t tt;
  int ms;
  split_seconds_ms(t, tt, ms);
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

  existing_keys_.clear();
  last_write_suppressed_ = false;
  suppressed_duplicate_count_ = 0;
  discarded_incomplete_line_ = false;

  // "新文件"指不存在或存在但长度为 0 —— 用这个判断是否需要写表头,
  // 因此进程重启接续一个已有非空文件时不会再写一遍表头。
  bool is_new = true;
  {
    std::error_code size_ec;
    const auto sz = std::filesystem::file_size(fp, size_ec);
    if (!size_ec) is_new = (sz == 0);
  }

  // bug A:断电导致最后一行没有换行结尾——视为"这一行从未真正写完",原地
  // 截掉(见 pos_io.hpp 类注释与 truncate_incomplete_trailing_line)。必须
  // 在下面收集去重键之前做,否则半行本身或者(旧实现里)被它污染出来的
  // 错误列会混进 existing_keys_。截完之后文件可能变空(唯一一行本来就是
  // 残缺的),因此要重新判定一次 is_new——这也是唯一一处"截断后文件变空"
  // 的地方,漏掉会在下一次 open() 里产出一个没有表头的文件。
  //
  // round 2 review 的 BLOCKING 1/2:这一步(读最后一个字节/读整份内容/
  // 真正 resize_file)只要失败,就必须让 open() 直接失败,绝不能猜"大概
  // 不需要截断"就继续往下 append——那样会把新记录粘连到还没写完的半行上,
  // 跟完全没做这个修复时一样。
  if (!is_new) {
    switch (truncate_incomplete_trailing_line(fp)) {
      case TrailingLineOutcome::kFailed:
        return false;
      case TrailingLineOutcome::kTruncated: {
        discarded_incomplete_line_ = true;
        std::error_code size_ec2;
        const auto sz2 = std::filesystem::file_size(fp, size_ec2);
        if (!size_ec2) is_new = (sz2 == 0);
        break;
      }
      case TrailingLineOutcome::kNoTruncationNeeded:
        break;
    }
  }

  // bug B/C:精确成员判定取代旧的"stamp <= 文件最后一条的 stamp"分界线
  // (见 pos_io.hpp 类注释)。复用 read_pos()(不写第二个 .pos 解析器)
  // 解析已有内容,对每一条都按当前 open() 使用的 ts_/leap_ 重新算出它在
  // .pos 里占的那个毫秒键,塞进 existing_keys_。
  if (!is_new) {
    try {
      const auto existing = read_pos(path, PosReadOptions{leap_, ts_});
      for (const auto& rec : existing) existing_keys_.insert(record_time_key_ms(rec, ts_, leap_));
    } catch (const std::exception&) {
      // 文件已经确认存在(上面 file_size 没报错),这里失败极不寻常
      // (比如竞态下被删掉)——不让它阻止 open(),只是没有去重保护。
    }
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
  if (!out_.is_open()) {
    last_write_suppressed_ = false;
    return false;
  }
  const long long key = record_time_key_ms(r, ts_, leap_);
  if (existing_keys_.count(key) != 0) {
    // 这个毫秒键已经在文件里(可能来自之前的 open() 会话,也可能是本次
    // 会话里刚写过)——精确判定为重复,静默跳过,不算失败:调用方据此
    // 报一次汇总计数。
    last_write_suppressed_ = true;
    ++suppressed_duplicate_count_;
    return true;
  }
  last_write_suppressed_ = false;
  out_ << format_pos_record(r, ts_, leap_);
  out_.flush();
  if (!out_) return false;   // 写失败:不把这个键记为"已存在",好让恢复后的重试能真正写出
  existing_keys_.insert(key);
  return true;
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
