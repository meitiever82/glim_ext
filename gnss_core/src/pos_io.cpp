#include "gnss_core/pos_io.hpp"
#ifdef GNSS_CORE_WITH_TEST_HOOKS
#include "gnss_core/pos_io_test_hooks.hpp"   // 仅在 BUILD_TESTING 时编译进来(见 CMakeLists.txt),不随生产库一起发布
#endif
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
// round 3 复盘(final-fix-report.md 之后又发现的问题):上一版虽然在读完
// 之后检查了流状态,但 libstdc++ 的 basic_filebuf::underflow() 在真实
// I/O 错误时会*无条件*抛 std::ios_base::failure——不受这个流的
// exceptions() 掩码影响,也不会等着让调用方去查 rdstate()。reviewer 用
// LD_PRELOAD 在真实二进制上复现:读整份内容那次 istreambuf_iterator 读取
// 在第 2/3/5/20 次内部缓冲区重新填充时命中一次瞬时 EIO,异常直接从这段
// 代码里抛出来,原来完全没有 try/catch,一路捅穿 open() "打开失败返回
// false、不抛"的承诺(靠 pos_writer_node.cpp 里一个本不该需要存在的
// try/catch 兜底,才没有真的崩)。现在把这几行读操作整体包进 try/catch,
// 把任何 std::exception 都当成 kFailed——不管失败是"读完之后查状态发现
// 的"还是"读到一半直接抛出来的",处理方式必须是同一条规则。
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

// 仅在 BUILD_TESTING 时存在(GNSS_CORE_WITH_TEST_HOOKS 由 CMakeLists.txt
// 按 BUILD_TESTING 条件定义)——一个测试注入点不应该出现在正式发布的
// libgnss_core.so 导出符号表里。
#ifdef GNSS_CORE_WITH_TEST_HOOKS
gnss_core::testing::TrailingLineReadFailureInjector g_trailing_line_read_failure_injector = nullptr;

// round 3 复盘的教训:上一版这里只是"读成功之后再问一句要不要假装失败",
// 测的只是"这个 if 分支存在",从来没有真正让任何一次读失败过——删掉
// try/catch(round 3 复现的那个 bug)之后,这个注入点原样通过,测不出
// 问题。现在直接在"真正要去读"之前抛出一个和 libstdc++ 同类型的异常
// (std::ios_base::failure),把"这次读失败"做成一次真的异常传播,而不是
// 一个被检查的返回值——这样下面的 try/catch 才是真正被这个测试用例覆盖
// 到的代码,删掉它测试就会因为异常没被接住而失败(进程异常终止/gtest
// 报告未捕获异常),而不是安静地继续通过。
void throw_if_injected(const char* step) {
  if (g_trailing_line_read_failure_injector != nullptr && g_trailing_line_read_failure_injector(step)) {
    throw std::ios_base::failure(std::string("gnss_core test injection: ") + step);
  }
}
// resize_file() 走 std::error_code、不走异常,所以这里不需要 throw——
// 直接问一句"这一步要不要模拟失败",跟 throw_if_injected 是同一个注入点
// 的两种表现形式(读用抛异常模拟,resize 用返回值模拟,各自贴近生产代码
// 里这一步真实的失败方式)。
bool injected_resize_failure() {
  return g_trailing_line_read_failure_injector != nullptr && g_trailing_line_read_failure_injector("resize");
}

// round 3 review 的第三次纠正:read_pos() 用 std::getline 读文件——这是
// 一个"格式化的"istream 操作,内部会经过 istream::sentry。真实 I/O 错误
// 发生时,libstdc++ 的 sentry 会*吞掉* underflow() 抛出的
// std::ios_base::failure、把流设成 badbit,并且——因为这个流的
// exceptions() 掩码默认是 goodbit——**不会重新抛出来**。这跟
// full_content 那种绕开 sentry、直接用 istreambuf_iterator 操作
// streambuf 的读法完全不是同一种失败方式,那种读法才会无条件抛异常
// (round 3 的 BLOCKING 修复正是针对那种情况)。因此这里不能像上面
// throw_if_injected 那样抛异常来模拟——那样测出来的是"read_pos 会不会
// 抛异常",而真实场景里它根本不抛,只是安静地把流设成 badbit、循环提前
// 结束、返回一份不完整的结果。这个注入点直接把流设成同样的状态,精确
// 复现"没有异常、只是从此往后读不到东西"这个真正的失败方式。
bool injected_read_pos_scan_failure() {
  return g_trailing_line_read_failure_injector != nullptr && g_trailing_line_read_failure_injector("read_pos_scan");
}
#else
inline void throw_if_injected(const char*) {}   // 非测试构建:这段代码整体不存在,调用处优化成空操作
inline bool injected_resize_failure() { return false; }
inline bool injected_read_pos_scan_failure() { return false; }
#endif

// 一次截断在“最后一行没写完整”这个前提下,理论上能去掉的字节数不会超过
// 一行的最坏长度——format_pos_record 的列宽是固定的(日期时间 24 字节 +
// 13 个定宽数值列),正常情况下远小于这个上限。用一个宽松的常数而不是
// 精确算出的行宽,一是格式将来略微调整时不用跟着改这里,二是作为纵深
// 防御:即便"读到的字节数恰好等于文件大小"这个校验也被蒙混过去(比如
// 某种读错误凑巧返回了等长但错误的数据),截断量一旦远超"一行"的量级,
// 就说明拿到的 content 根本不可信,拒绝截断远比截掉不该截的东西安全。
//
// 已知的权衡(round 3 review 明确要求本轮不修):如果一行真的超过这个
// 上限(比如未来格式改动让某一列变成变长字段),这个文件会永远打不开
// (每次 open() 都判定成"截断量可疑"而拒绝)。安全的方向是"拒绝而不是
// 冒险截错",但确实缺一条"为什么打不开"的诊断日志——留到下一轮。
constexpr std::uintmax_t kMaxPlausibleIncompleteLineBytes = 4096;

TrailingLineOutcome truncate_incomplete_trailing_line(const std::filesystem::path& fp) {
  std::error_code size_ec;
  const auto file_size = std::filesystem::file_size(fp, size_ec);
  if (size_ec) return TrailingLineOutcome::kFailed;
  if (file_size == 0) return TrailingLineOutcome::kNoTruncationNeeded;

  std::ifstream in(fp, std::ios::binary);
  if (!in) return TrailingLineOutcome::kFailed;   // 比如没有读权限(BLOCKING 2 的 chmod 200 场景)

  try {
    in.seekg(static_cast<std::streamoff>(file_size) - 1);
    char last = 0;
    // round 3 review 的第三次纠正:in.get() 是格式化的 istream 操作,会
    // 经过 sentry——真实 I/O 错误在这里发生时,sentry 会吞掉 underflow()
    // 抛出的异常、把流设成 badbit,并不会重新抛出来(这跟下面
    // full_content 那种绕开 sentry、直接操作 streambuf、无条件抛异常的
    // 读法不是同一回事)。下面的 if (!in) 检查才是这一步真正生效的失败
    // 处理;这里的 throw_if_injected 只是防御性的——万一将来因为某种
    // 实现差异这里真的抛出来,确认周围的 try/catch 也接得住,不代表在
    // 复现一个已知会发生的真实失败路径(见 pos_io_test_hooks.hpp 的
    // 类注释)。
    throw_if_injected("last_byte");
    in.get(last);
    // 失败但没有抛异常(sentry 吞掉了,或者单纯的 fail/eof)同样不能被
    // 当成"读到了 0",必须立刻当成失败处理——这是这一步真正的失败通道。
    if (!in) return TrailingLineOutcome::kFailed;
    if (last == '\n') return TrailingLineOutcome::kNoTruncationNeeded;   // 最后一行完整,不用截断

    in.clear();
    in.seekg(0);
    if (!in) return TrailingLineOutcome::kFailed;
    throw_if_injected("full_content");
    const std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const bool stream_had_io_error = in.bad();
    in.close();
    // 第二道防线,比单看流状态更可靠:istreambuf_iterator 读到 EOF 不会
    // 设置 failbit,一次真正的中途 I/O 错误未必会体现在流状态上(如果它
    // 没有像 underflow() 那样直接抛出来的话)——但读到的字节数一定会跟
    // 文件大小对不上。两者任一为真都不可信。
    if (stream_had_io_error || content.size() != static_cast<std::uintmax_t>(file_size)) {
      return TrailingLineOutcome::kFailed;
    }

    const auto pos_nl = content.find_last_of('\n');
    const std::uintmax_t old_size = static_cast<std::uintmax_t>(file_size);
    const std::uintmax_t new_size = (pos_nl == std::string::npos) ? 0 : static_cast<std::uintmax_t>(pos_nl + 1);
    if (old_size - new_size > kMaxPlausibleIncompleteLineBytes) {
      // 算出来的截断量比一整行还大得多——content 不可信,拒绝截断。
      return TrailingLineOutcome::kFailed;
    }

    // round 3 review 纠正了上一轮的一个错误判断:曾经以为 resize_file()
    // 失败这条分支是"defense-in-depth"、可有可无,理由是它跟下面
    // out_.open(path, ios::app) 共用同一个"对文件要有写权限"的判定,两者
    // 总是同时失败或同时成功。reviewer 用 chattr +a(只读文件系统的
    // append-only 属性:允许 O_APPEND 追加,拒绝 truncate())构造出一个
        // "能读、能 append,但 truncate() 会被 EPERM 拒绝"的真实场景,证明
    // 这两个权限判定并不总是绑在一起——去掉这里的检查之后,open() 会
    // 误判成功,write() 真的把新记录 append 了上去,直接粘连在还没被截掉
    // 的半行后面(复现:988 字节的文件变成 1129 字节,bug A 原样复发)。
    // 这条检查是这条路径下的第一道、也是唯一一道防线,不是冗余。
    std::error_code trunc_ec;
    if (injected_resize_failure()) {
      // 测试注入:跳过真正的 resize_file() 调用,直接模拟它失败——这样
      // 才能在没有 root/CAP_LINUX_IMMUTABLE 的沙箱里确定性地覆盖这条分支
      // (chattr +a 需要 root,这台机器上的普通用户做不到),同时保证文件
      // 在磁盘上确实一字节没被动过。
      trunc_ec = std::make_error_code(std::errc::operation_not_permitted);
    } else {
      std::filesystem::resize_file(fp, new_size, trunc_ec);
    }
    if (trunc_ec) return TrailingLineOutcome::kFailed;   // 截断本身失败(比如没有写权限,或者 chattr +a)
    return TrailingLineOutcome::kTruncated;
  } catch (const std::exception&) {
    // libstdc++ 在真实 I/O 错误时可能从上面任意一次读取里直接抛出来
    // (见本函数前面的类注释),不管流的 exceptions() 掩码——open() 的
    // "打开失败返回 false、不抛"承诺要求这里必须接住,不能让它捅穿。
    return TrailingLineOutcome::kFailed;
  }
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

#ifdef GNSS_CORE_WITH_TEST_HOOKS
namespace testing {
void set_trailing_line_read_failure_injector(TrailingLineReadFailureInjector injector) {
  g_trailing_line_read_failure_injector = injector;
}
}  // namespace testing
#endif

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
    if (injected_read_pos_scan_failure()) {
      // 测试注入(仅 BUILD_TESTING):见上面 injected_read_pos_scan_failure
      // 的注释——直接把流设成 badbit,不抛异常,精确复现真实 I/O 错误在
      // getline 这里的失败方式。
      in.setstate(std::ios::badbit);
      break;
    }
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
  // round 3 review 的第三次纠正:上面的循环只在 getline 因为正常 EOF
  // 结束(设置 eofbit,但不设置 badbit)时才应该被信任为"读完了整份
  // 文件"。真实 I/O 错误发生时,getline 内部的 istream::sentry 会把流
  // 设成 badbit、循环因为流状态不再"好"而提前结束——但**不会抛异常**
  // (exceptions() 掩码默认是 goodbit),因此不检查这里就会安静地把
  // "读到故障发生前"当成"读到了全部内容",返回一份不完整的 vector 却不
  // 报任何错。这里统一按 read_pos() 已有的"打不开文件"同一个错误约定
  // (std::runtime_error)报出来——调用方(PosWriter::open() 的
  // try/catch,以及离线工具 calibrate_sigma_scale/estimate_lever_arm 只要
  // 补上 try/catch)不用再猜"这份结果到底完不完整"。
  if (in.bad()) {
    throw std::runtime_error("read_pos: I/O error while reading " + path);
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
  //
  // round 3 review:这里原来的 catch 会静默吞掉任何异常、让 open() 照样
  // 返回 true(只是没有去重保护)——理由是"文件已经确认存在,这里失败
  // 极不寻常"。继续吞掉、照样成功,等于让"这一整轮修复的三个数据正确性
  // bug"原样复发(existing_keys_ 是空的或不完整的,新写入的记录不会跟
  // 文件里已有的内容去重)。这是跟上面截断检查完全同一条"读失败就必须让
  // open() 直接失败"的规则,不该是两套不一致的容错策略——统一改成失败
  // 直接返回 false。理论上也可能是文件在 file_size() 判过之后被并发删除
  // 这种更罕见的竞态,但同样没有办法安全区分"这是哪一种失败",按同一条
  // 规则处理。
  //
  // round 3 review 的第三次纠正:这个 catch 曾经一度靠 open() 这一侧
  // 抢先抛一个异常来"测出它存在"——但 read_pos() 内部用的是 std::getline
  // (格式化 istream 操作,经过 sentry),真实 I/O 错误在这里根本不会抛
  // 异常,只会把流设成 badbit、循环提前结束、安静地返回一份不完整的
  // vector(跟上面 truncate_incomplete_trailing_line 里 istreambuf_iterator
  // 那种绕开 sentry、无条件抛异常的失败方式不是同一回事)。真正的修复在
  // read_pos() 自己身上(见其实现里 in.bad() 之后 throw 的那一段)——
  // read_pos() 现在会对这种情况真的抛出来,这里的 catch 才有意义接住它。
  if (!is_new) {
    try {
      const auto existing = read_pos(path, PosReadOptions{leap_, ts_});
      for (const auto& rec : existing) existing_keys_.insert(record_time_key_ms(rec, ts_, leap_));
    } catch (const std::exception&) {
      return false;
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
