#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "gnss_core/diag_io.hpp"
#include "gnss_core/report_cli.hpp"
using namespace gnss_core;
namespace fs = std::filesystem;

namespace {
const double T = 1789430400.0;   // 2026-09-15 00:00:00 UTC

class TempDir {
public:
  TempDir() {
    const char* base = std::getenv("TMPDIR");
    std::string tmpl = std::string(base ? base : "/tmp") + "/report_cli_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    if (::mkdtemp(buf.data()) != nullptr) path_ = buf.data();
  }
  ~TempDir() {
    std::error_code ec;
    if (!path_.empty()) fs::remove_all(path_, ec);
  }
  const std::string& path() const { return path_; }

private:
  std::string path_;
};

// 测试里切换工作目录,析构时切回(同一进程内的其他用例依赖原工作目录)
class CwdGuard {
public:
  explicit CwdGuard(const std::string& dir) : old_(fs::current_path()) { fs::current_path(dir); }
  ~CwdGuard() {
    std::error_code ec;
    fs::current_path(old_, ec);
  }

private:
  fs::path old_;
};

struct Run {
  int code = -1;
  std::string out, err;
};

Run run(const std::vector<std::string>& args) {
  std::ostringstream out, err;
  Run r;
  r.code = run_gnss_report(args, out, err);
  r.out = out.str();
  r.err = err.str();
  return r;
}

std::string slurp(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

void make_day(const std::string& root) {
  fs::create_directories(root + "/20260915");
  std::vector<PosRecord> recs;
  for (int i = 0; i < 10; ++i) {
    PosRecord r;
    r.stamp = T + 3600.0 + i;
    r.q = i < 8 ? 1 : 2;
    r.lat = 44.5;
    r.lon = 90.28;
    r.height = 600.0;
    recs.push_back(r);
  }
  write_pos(root + "/20260915/can.pos", recs);
  EventTransition e;
  e.kind = EventKind::Open;
  e.t = e.t_open = T + 3601.0;
  e.code = "corr_outage";
  e.level = Level::Serious;
  e.message = "差分中断";
  std::ofstream(root + "/20260915/events.log") << events_log_header() << format_event_line(e) << "\n";
  std::ofstream(root + "/20260915/base.pos") << base_pos_header() << "2026/09/15 01:00:00.000  1.0\n";   // 半行
}
}  // namespace

TEST(ReportCli, HelpPrintsUsageAndExitsZero) {
  const auto r = run({"--help"});
  EXPECT_EQ(r.code, 0);
  EXPECT_NE(r.out.find("用法"), std::string::npos);
  EXPECT_NE(r.out.find("--control-point"), std::string::npos);
  EXPECT_TRUE(r.err.empty());
}

TEST(ReportCli, UsageErrorsExitOneWithAReasonAndTheUsage) {
  const std::vector<std::pair<std::vector<std::string>, std::string>> cases{
      {{}, "缺少 --root"},
      {{"--root"}, "缺少参数值"},
      {{"--root", "/x"}, "--day"},
      {{"--root", "/x", "--day", "20260915", "--from", "2026/09/15 00:00:00", "--to", "2026/09/15 01:00:00"}, "二选一"},
      {{"--root", "/x", "--day", "2026091"}, "YYYYMMDD"},
      {{"--root", "/x", "--from", "2026-09-15", "--to", "2026/09/15 00:00:00"}, "格式应为"},
      {{"--root", "/x", "--from", "2026/09/15 01:00:00", "--to", "2026/09/15 00:00:00"}, "晚于"},
      {{"--root", "/x", "--from", "2026/08/01 00:00:00", "--to", "2026/09/15 00:00:00"}, "31 天"},
      {{"--root", "/x", "--day", "20260915", "--control-point", "K1,91,90"}, "--control-point"},
      {{"--root", "/x", "--day", "20260915", "--control-point", "K1,44.5"}, "--control-point"},
      {{"--root", "/x", "--bogus"}, "不认识"},
      {{"--root", "/x", "--day", "20260915", "--abs-ref-max-m", "abc"}, "需要一个数字"},
      {{"--root", "/x", "--day", "20260915", "--pair-tol-s", "0"}, "大于 0"},
      {{"--root", "/x", "--day", "20260915", "--leap-seconds", "1.5"}, "--leap-seconds"},
  };
  for (const auto& [args, reason] : cases) {
    std::string joined;
    for (const auto& a : args) joined += a + " ";
    const auto r = run(args);
    EXPECT_EQ(r.code, 1) << joined;
    EXPECT_NE(r.err.find(reason), std::string::npos) << joined << "\n" << r.err;
    EXPECT_NE(r.err.find("用法"), std::string::npos) << joined;
    EXPECT_TRUE(r.out.empty()) << joined;
  }
}

TEST(ReportCli, WritesTheReportForADayAndPrintsASummaryAndWarnings) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  make_day(dir.path() + "/root");
  const std::string out_path = dir.path() + "/r.html";
  const auto r = run({"--root", dir.path() + "/root", "--day", "20260915", "--out", out_path, "--control-point",
                      "K1,44.5,90.28"});
  ASSERT_EQ(r.code, 0) << r.err;
  EXPECT_NE(r.out.find("已生成 " + out_path), std::string::npos) << r.out;
  EXPECT_NE(r.out.find("can: 10 历元, 固定率 80.0%"), std::string::npos) << r.out;
  EXPECT_NE(r.out.find("事件 1 条"), std::string::npos) << r.out;
  EXPECT_NE(r.err.find("警告"), std::string::npos) << "base.pos 的半行要报出来\n" << r.err;
  const std::string html = slurp(out_path);
  EXPECT_EQ(html.rfind("<!DOCTYPE html>", 0), 0u);
  EXPECT_NE(html.find("id=\"events\""), std::string::npos);
  EXPECT_NE(html.find("corr_outage"), std::string::npos);
  EXPECT_NE(html.find("K1"), std::string::npos);
  EXPECT_NE(html.find("2026/09/16 00:00:00.000"), std::string::npos) << "--day 覆盖整个 UTC 自然日";
}

// final review Critical 复现(rev4a/bmove):前一天记了基站坐标,当天挪了 0.5 m 只写了一行
TEST(ReportCli, ABaseMoveOnTheReportDayIsVisibleAgainstThePreviousDaysRow) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  const std::string root = dir.path() + "/root";
  fs::create_directories(root + "/20260914");
  fs::create_directories(root + "/20260915");
  std::ofstream(root + "/20260914/base.pos")
      << base_pos_header() << format_base_history_line(T - 57600.0, Ecef{-2148744.1, 4426641.2, 4044655.9}) << "\n";
  std::ofstream(root + "/20260915/base.pos")
      << base_pos_header() << format_base_history_line(T + 36000.0, Ecef{-2148744.6, 4426641.2, 4044655.9}) << "\n";
  const std::string out_path = dir.path() + "/r.html";
  const auto r = run({"--root", root, "--day", "20260915", "--out", out_path});
  ASSERT_EQ(r.code, 0) << r.err;
  const std::string html = slurp(out_path);
  const size_t a = html.find("id=\"base\""), b = html.find("id=\"events\"");
  ASSERT_NE(a, std::string::npos);
  ASSERT_NE(b, std::string::npos);
  const std::string base = html.substr(a, b - a);
  EXPECT_NE(base.find("<b>0.500 m</b>"), std::string::npos) << base;
  EXPECT_NE(base.find("基站坐标可能变动"), std::string::npos) << base;
  EXPECT_NE(base.find("2026/09/14 08:00:00.000"), std::string::npos) << "说明基准取自哪条记录\n" << base;
}

TEST(ReportCli, DefaultOutputNameEncodesTheWindowInTheCurrentDirectory) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  make_day(dir.path() + "/root");
  {
    CwdGuard cwd(dir.path());
    const auto r = run({"--root", dir.path() + "/root", "--from", "2026/09/15 01:00:00", "--to", "2026/09/15 02:30:00"});
    ASSERT_EQ(r.code, 0) << r.err;
  }
  EXPECT_TRUE(fs::exists(dir.path() + "/gnss_report_20260915T010000_20260915T023000.html"));
}

TEST(ReportCli, UnreadableRootOrUnwritableOutputExitsTwo) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  auto r = run({"--root", dir.path() + "/absent", "--day", "20260915", "--out", dir.path() + "/r.html"});
  EXPECT_EQ(r.code, 2);
  EXPECT_NE(r.err.find("absent"), std::string::npos) << r.err;
  EXPECT_FALSE(fs::exists(dir.path() + "/r.html"));

  make_day(dir.path() + "/root");
  r = run({"--root", dir.path() + "/root", "--day", "20260915", "--out", dir.path() + "/no/such/dir/r.html"});
  EXPECT_EQ(r.code, 2);
  EXPECT_NE(r.err.find("无法写出"), std::string::npos) << r.err;
}
