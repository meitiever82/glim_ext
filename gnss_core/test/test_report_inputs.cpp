#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "gnss_core/diag_io.hpp"
#include "gnss_core/report_inputs.hpp"
using namespace gnss_core;
namespace fs = std::filesystem;

namespace {
const double T = 1789430400.0;   // 2026-09-15 00:00:00 UTC
const ReportWindow kDay{T, T + 86400.0};

class TempDir {
public:
  TempDir() {
    const char* base = std::getenv("TMPDIR");
    std::string tmpl = std::string(base ? base : "/tmp") + "/report_inputs_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    if (::mkdtemp(buf.data()) != nullptr) path_ = buf.data();
  }
  ~TempDir() {
    if (path_.empty()) return;
    std::error_code ec;
    fs::permissions(path_, fs::perms::owner_all, fs::perm_options::add, ec);
    for (auto it = fs::recursive_directory_iterator(path_, ec); !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
      fs::permissions(it->path(), fs::perms::owner_all, fs::perm_options::add, ec);
    }
    fs::remove_all(path_, ec);
  }
  const std::string& path() const { return path_; }

private:
  std::string path_;
};

PosRecord rec(double t, int q = 1, double lat = 44.5, double lon = 90.28) {
  PosRecord r;
  r.stamp = t;
  r.q = q;
  r.lat = lat;
  r.lon = lon;
  r.height = 600.0;
  r.ns = 20;
  return r;
}

void write_text(const std::string& path, const std::string& text) {
  fs::create_directories(fs::path(path).parent_path());
  std::ofstream(path, std::ios::binary) << text;
}

EventTransition ev(EventKind kind, double t, double t_open, const std::string& code,
                   std::optional<LatLon> pos = std::nullopt) {
  EventTransition e;
  e.kind = kind;
  e.t = t;
  e.t_open = t_open;
  e.code = code;
  e.level = Level::Serious;
  e.message = code + " 的结论";
  e.pos = pos;
  e.reason = CloseReason::Recovered;
  return e;
}

std::string events_file(const std::vector<EventTransition>& es) {
  std::string s = events_log_header();
  for (const auto& e : es) s += format_event_line(e) + "\n";
  return s;
}
}  // namespace

TEST(ReportInputs, LoadsPosSourcesInsideTheWindowSortedAndDropsEmptySources) {
  TempDir root;
  ASSERT_FALSE(root.path().empty());
  fs::create_directories(root.path() + "/20260915");
  write_pos(root.path() + "/20260915/can.pos", {rec(T + 10.0), rec(T + 5.0, 2)});   // 乱序写入,GPST
  write_pos(root.path() + "/20260915/rtkrcv.pos", {rec(T + 90000.0)});               // 次日,在窗口外
  write_pos(root.path() + "/20260914/gpchc.pos", {rec(T - 10.0)});                   // 前一天目录不读 .pos
  write_text(root.path() + "/20260915/base.pos", base_pos_header());                 // base.pos 不是数据源

  const auto in = load_report_inputs(root.path(), kDay);
  ASSERT_EQ(in.sources.size(), 1u) << "窗口内没有记录的源不出现";
  const auto& can = in.sources.at("can");
  ASSERT_EQ(can.size(), 2u);
  EXPECT_NEAR(can[0].stamp, T + 5.0, 1e-3) << "按时间升序";
  EXPECT_EQ(can[0].q, 2);
  EXPECT_NEAR(can[1].stamp, T + 10.0, 1e-3);
  EXPECT_TRUE(in.warnings.empty());
}

TEST(ReportInputs, PairsOpenAndCloseAcrossDaysAndKeepsTheOpenPosition) {
  TempDir root;
  ASSERT_FALSE(root.path().empty());
  const LatLon at_open{44.5, 90.28}, at_close{44.6, 90.29};
  write_text(root.path() + "/20260914/events.log",
             events_file({ev(EventKind::Open, T - 50.0, T - 50.0, "corr_outage", at_open)}));
  EventTransition close = ev(EventKind::Close, T + 30.0, T - 50.0, "corr_outage", at_close);
  close.peak = {{"corr_gap_s", 80.0}};
  write_text(root.path() + "/20260915/events.log",
             events_file({close, ev(EventKind::Open, T + 100.0, T + 100.0, "low_sats", at_close)}));

  const auto in = load_report_inputs(root.path(), kDay);
  ASSERT_EQ(in.events.size(), 2u);
  const auto& a = in.events[0];
  EXPECT_EQ(a.code, "corr_outage");
  EXPECT_NEAR(a.t_open, T - 50.0, 1e-3);
  ASSERT_TRUE(a.t_close.has_value());
  EXPECT_NEAR(*a.t_close, T + 30.0, 1e-3);
  EXPECT_EQ(a.close_reason, "recovered");
  ASSERT_TRUE(a.pos.has_value());
  EXPECT_NEAR(a.pos->lat, 44.5, 1e-9) << "问题路段取开启位置";
  EXPECT_NEAR(a.peak.at("corr_gap_s"), 80.0, 1e-9);
  EXPECT_EQ(a.level, "serious");
  const auto& b = in.events[1];
  EXPECT_EQ(b.code, "low_sats");
  EXPECT_FALSE(b.t_close.has_value()) << "没有关闭行:仍在进行或进程中断";
  EXPECT_TRUE(b.close_reason.empty());
}

TEST(ReportInputs, UnmatchedCloseStandsAloneAndASecondOpenLeavesTheFirstUnclosed) {
  TempDir root;
  ASSERT_FALSE(root.path().empty());
  const LatLon p{44.5, 90.28};
  const ReportWindow w{T + 100.0, T + 200.0};
  write_text(root.path() + "/20260915/events.log",
             events_file({
                 ev(EventKind::Open, T + 10.0, T + 10.0, "x", p),
                 ev(EventKind::Close, T + 20.0, T + 10.0, "x", p),             // 窗口前就关了
                 ev(EventKind::Open, T + 110.0, T + 110.0, "y", p),
                 ev(EventKind::Open, T + 150.0, T + 150.0, "y", p),            // 进程重启后又开
                 ev(EventKind::Close, T + 160.0, T + 5.0, "z", std::nullopt),  // OPEN 行找不到
                 ev(EventKind::Open, T + 300.0, T + 300.0, "w", p),            // 窗口后才开
             }));
  const auto in = load_report_inputs(root.path(), w);
  ASSERT_EQ(in.events.size(), 3u);
  EXPECT_EQ(in.events[0].code, "z");
  EXPECT_NEAR(in.events[0].t_open, T + 5.0, 1e-3);
  ASSERT_TRUE(in.events[0].t_close.has_value());
  EXPECT_FALSE(in.events[0].pos.has_value());
  EXPECT_EQ(in.events[1].code, "y");
  EXPECT_NEAR(in.events[1].t_open, T + 110.0, 1e-3);
  EXPECT_FALSE(in.events[1].t_close.has_value()) << "被第二条 OPEN 顶替,视为未关闭";
  EXPECT_EQ(in.events[2].code, "y");
  EXPECT_NEAR(in.events[2].t_open, T + 150.0, 1e-3);
}

TEST(ReportInputs, BaseHistoryIsWindowedAndBrokenInputsBecomeWarnings) {
  TempDir root;
  ASSERT_FALSE(root.path().empty());
  write_text(root.path() + "/20260915/base.pos",
             base_pos_header() + format_base_history_line(T + 1.0, Ecef{1.0, 2.0, 3.0}) + "\n" +
                 format_base_history_line(T + 90000.0, Ecef{4.0, 5.0, 6.0}) + "\n" + "2026/09/15 00:00:02.000  1.0\n");
  write_text(root.path() + "/20260915/events.log", events_log_header() + "hello world\n");
  const auto in = load_report_inputs(root.path(), kDay);
  ASSERT_EQ(in.base_history.size(), 1u);
  EXPECT_NEAR(in.base_history[0].t, T + 1.0, 1e-3);
  EXPECT_DOUBLE_EQ(in.base_history[0].p.z, 3.0);
  bool events_warned = false, base_warned = false;
  for (const auto& w : in.warnings) {
    if (w.find("events.log") != std::string::npos && w.find("1 行") != std::string::npos) events_warned = true;
    if (w.find("base.pos") != std::string::npos && w.find("1 行") != std::string::npos) base_warned = true;
  }
  EXPECT_TRUE(events_warned);
  EXPECT_TRUE(base_warned);
}

TEST(ReportInputs, UnreadablePosFileIsAWarningNotAFailure) {
  if (::geteuid() == 0) GTEST_SKIP() << "root 无视文件权限,无法构造读失败";
  TempDir root;
  ASSERT_FALSE(root.path().empty());
  write_pos(root.path() + "/20260915/can.pos", {rec(T + 1.0)});
  write_pos(root.path() + "/20260915/rtkrcv.pos", {rec(T + 1.0)});
  ASSERT_EQ(::chmod((root.path() + "/20260915/can.pos").c_str(), 0), 0);
  const auto in = load_report_inputs(root.path(), kDay);
  EXPECT_EQ(in.sources.count("can"), 0u);
  EXPECT_EQ(in.sources.count("rtkrcv"), 1u);
  ASSERT_EQ(in.warnings.size(), 1u);
  EXPECT_NE(in.warnings[0].find("can.pos"), std::string::npos) << in.warnings[0];
}

TEST(ReportInputs, MissingRootThrowsButAnEmptyRootIsFine) {
  TempDir root;
  ASSERT_FALSE(root.path().empty());
  EXPECT_THROW(load_report_inputs(root.path() + "/absent", kDay), std::runtime_error);
  const auto in = load_report_inputs(root.path(), kDay);
  EXPECT_TRUE(in.sources.empty());
  EXPECT_TRUE(in.events.empty());
  EXPECT_TRUE(in.base_history.empty());
  EXPECT_TRUE(in.warnings.empty());
}
