#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "gnss_core/diag_io.hpp"
using namespace gnss_core;

namespace {
class TempDir {
public:
  TempDir() {
    const char* base = std::getenv("TMPDIR");
    std::string tmpl = std::string(base ? base : "/tmp") + "/diag_io_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    if (::mkdtemp(buf.data()) != nullptr) path_ = buf.data();
  }
  ~TempDir() {
    if (!path_.empty()) std::filesystem::remove_all(path_);
  }
  const std::string& path() const { return path_; }

private:
  std::string path_;
};

std::string slurp(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

EventTransition open_event() {
  EventTransition e;
  e.kind = EventKind::Open;
  e.t = 1789372800.5;
  e.t_open = e.t;
  e.code = "corr_outage";
  e.level = Level::Serious;
  e.message = "差分中断 10s——5G 链路或平台转发问题";
  e.pos = LatLon{44.5, 90.28};
  return e;
}
}  // namespace

TEST(DiagIo, UtcTimestampRoundsToMillisecondsAndCarries) {
  EXPECT_EQ(format_utc_timestamp(1789372800.5), "2026/09/14 08:00:00.500");
  EXPECT_EQ(format_utc_timestamp(1789372817.25), "2026/09/14 08:00:17.250");
  EXPECT_EQ(format_utc_timestamp(1789459199.9996), "2026/09/15 08:00:00.000") << "四舍五入进位跨秒";
}

TEST(DiagIo, OpenEventLine) {
  EXPECT_EQ(format_event_line(open_event()),
            "2026/09/14 08:00:00.500 OPEN serious corr_outage lat=44.500000000 lon=90.280000000 "
            "差分中断 10s——5G 链路或平台转发问题");
}

TEST(DiagIo, CloseEventLineCarriesDurationReasonAndSortedPeak) {
  EventTransition e = open_event();
  e.kind = EventKind::Close;
  e.t = 1789372817.25;
  e.pos.reset();
  e.reason = CloseReason::Shutdown;
  e.peak = {{"sats_min", 4.0}, {"corr_gap_s", 12.0}};
  EXPECT_EQ(format_event_line(e),
            "2026/09/14 08:00:17.250 CLOSE serious corr_outage lat=- lon=- "
            "opened=2026/09/14 08:00:00.500 duration_s=16.8 reason=shutdown "
            "peak=corr_gap_s=12.000;sats_min=4.000 差分中断 10s——5G 链路或平台转发问题");
}

TEST(DiagIo, EmptyPeakAndNewlinesInMessage) {
  EventTransition e = open_event();
  e.kind = EventKind::Close;
  e.message = "a\nb\rc";
  const std::string line = format_event_line(e);
  EXPECT_NE(line.find(" peak=- "), std::string::npos) << line;
  EXPECT_NE(line.find("a b c"), std::string::npos) << line;
  EXPECT_EQ(line.find('\n'), std::string::npos);
}

TEST(DiagIo, HeadersAndBaseHistoryLine) {
  EXPECT_EQ(events_log_header().rfind("% gnss_core events.log (time=UTC)\n", 0), 0u);
  EXPECT_EQ(base_pos_header(),
            "% program : gnss_core base history\n"
            "% time=UTC\n"
            "%  UTC                    x-ecef(m)        y-ecef(m)        z-ecef(m)\n");
  EXPECT_EQ(format_base_history_line(1789372800.5, Ecef{-2148744.1, 4426641.2, 4044655.9}),
            "2026/09/14 08:00:00.500  -2148744.1000 4426641.2000 4044655.9000");
}

namespace {
EventTransition close_event_with_peak() {
  EventTransition e = open_event();
  e.kind = EventKind::Close;
  e.t = 1789372810.0;
  e.pos = LatLon{44.501, 90.281};
  e.reason = CloseReason::Recovered;
  e.peak = {{"corr_gap_s", 12.345}, {"sats_min", 4.0}};
  return e;
}
}  // namespace

// 轮 4a:报告工具要把 events.log 读回来,解析必须与 format_event_line 互逆
TEST(EventLineParsing, OpenLineRoundTrips) {
  const auto p = parse_event_line(format_event_line(open_event()));
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->kind, EventKind::Open);
  EXPECT_NEAR(p->t, 1789372800.5, 1e-6);
  EXPECT_NEAR(p->t_open, 1789372800.5, 1e-6) << "OPEN 行的开启时刻就是行时刻";
  EXPECT_EQ(p->level, "serious");
  EXPECT_EQ(p->code, "corr_outage");
  ASSERT_TRUE(p->pos.has_value());
  EXPECT_NEAR(p->pos->lat, 44.5, 1e-9);
  EXPECT_NEAR(p->pos->lon, 90.28, 1e-9);
  EXPECT_EQ(p->message, "差分中断 10s——5G 链路或平台转发问题");
}

TEST(EventLineParsing, CloseLineRoundTripsWithOpenedDurationReasonAndPeak) {
  const auto p = parse_event_line(format_event_line(close_event_with_peak()));
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->kind, EventKind::Close);
  EXPECT_NEAR(p->t, 1789372810.0, 1e-6);
  EXPECT_NEAR(p->t_open, 1789372800.5, 1e-6);
  EXPECT_NEAR(p->duration_s, 9.5, 1e-9);
  EXPECT_EQ(p->reason, "recovered");
  ASSERT_EQ(p->peak.size(), 2u);
  EXPECT_NEAR(p->peak.at("corr_gap_s"), 12.345, 1e-9);
  EXPECT_NEAR(p->peak.at("sats_min"), 4.0, 1e-9);
  ASSERT_TRUE(p->pos.has_value());
  EXPECT_NEAR(p->pos->lat, 44.501, 1e-9);
  EXPECT_EQ(p->message, "差分中断 10s——5G 链路或平台转发问题");
}

TEST(EventLineParsing, NoPositionEmptyPeakAndEmptyMessage) {
  EventTransition e = close_event_with_peak();
  e.pos.reset();
  e.peak.clear();
  e.message.clear();
  e.reason = CloseReason::Shutdown;
  const auto p = parse_event_line(format_event_line(e));
  ASSERT_TRUE(p.has_value()) << format_event_line(e);
  EXPECT_FALSE(p->pos.has_value());
  EXPECT_TRUE(p->peak.empty());
  EXPECT_EQ(p->message, "");
  EXPECT_EQ(p->reason, "shutdown");
}

TEST(EventLineParsing, RejectsCommentsHalfLinesAndUnknownWords) {
  std::istringstream header(events_log_header());
  for (std::string line; std::getline(header, line);) {
    EXPECT_FALSE(parse_event_line(line).has_value()) << line;
  }
  EXPECT_FALSE(parse_event_line("").has_value());
  const std::string close = format_event_line(close_event_with_peak());
  EXPECT_FALSE(parse_event_line(close.substr(0, 60)).has_value()) << "掉电留下的半行";
  std::string bad_kind = format_event_line(open_event());
  bad_kind.replace(bad_kind.find(" OPEN "), 6, " MAYBE ");
  EXPECT_FALSE(parse_event_line(bad_kind).has_value());
  std::string bad_level = format_event_line(open_event());
  bad_level.replace(bad_level.find(" serious "), 9, " fatal ");
  EXPECT_FALSE(parse_event_line(bad_level).has_value());
  std::string bad_reason = close;
  bad_reason.replace(bad_reason.find("reason=recovered"), 16, "reason=whatever");
  EXPECT_FALSE(parse_event_line(bad_reason).has_value());
}

TEST(BaseHistoryParsing, RoundTripsAndRejectsBrokenLines) {
  const Ecef p{-2148744.1, 4426641.2, 4044655.9};
  const auto r = parse_base_history_line(format_base_history_line(1789372800.25, p));
  ASSERT_TRUE(r.has_value());
  EXPECT_NEAR(r->first, 1789372800.25, 1e-6);
  EXPECT_NEAR(r->second.x, p.x, 1e-4);
  EXPECT_NEAR(r->second.y, p.y, 1e-4);
  EXPECT_NEAR(r->second.z, p.z, 1e-4);
  std::istringstream header(base_pos_header());
  for (std::string line; std::getline(header, line);) {
    EXPECT_FALSE(parse_base_history_line(line).has_value()) << line;
  }
  EXPECT_FALSE(parse_base_history_line("2026/09/14 08:00:00.500  -2148744.1").has_value()) << "半行";
  EXPECT_FALSE(parse_base_history_line("2026/09/14 08:00:00.500  1 2 3 4").has_value()) << "多余字段视为损坏";
  EXPECT_FALSE(parse_base_history_line("not-a-date 08:00:00.500  1 2 3").has_value());
  EXPECT_FALSE(parse_base_history_line("2026/09/14 08:00:00.500  nan 2 3").has_value());
}

// final review Important:在 Windows 上拷过、或经 CRLF 转换的 events.log / base.pos,行尾带 '\r'。
// gnss_bringup 的 read_last_base_history 直接 getline 后交给 parse_base_history_line,不会先剥 '\r'。
TEST(DiagLineParsing, CrlfLineEndingsParseForBothFormats) {
  for (const char* eol : {"\r", "\r\n"}) {
    const auto open = parse_event_line(format_event_line(open_event()) + eol);
    ASSERT_TRUE(open.has_value());
    EXPECT_EQ(open->message, "差分中断 10s——5G 链路或平台转发问题") << "结论里不能带着 \\r";

    const auto close = parse_event_line(format_event_line(close_event_with_peak()) + eol);
    ASSERT_TRUE(close.has_value());
    EXPECT_EQ(close->message, "差分中断 10s——5G 链路或平台转发问题");
    EXPECT_NEAR(close->peak.at("sats_min"), 4.0, 1e-9);

    EventTransition quiet = close_event_with_peak();
    quiet.message.clear();
    const auto empty_msg = parse_event_line(format_event_line(quiet) + eol);
    ASSERT_TRUE(empty_msg.has_value());
    EXPECT_EQ(empty_msg->message, "");

    const Ecef p{-2148744.1, 4426641.2, 4044655.9};
    const auto base = parse_base_history_line(format_base_history_line(1789372800.25, p) + eol);
    ASSERT_TRUE(base.has_value());
    EXPECT_NEAR(base->second.z, p.z, 1e-4);
    EXPECT_FALSE(parse_base_history_line(std::string("2026/09/14 08:00:00.500  1 2 3 4") + eol).has_value())
        << "多余字段仍然拒绝";
  }
}

TEST(LineAppender, WritesHeaderOnceAndAppendsAcrossReopen) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  const std::string path = dir.path() + "/20260914/events.log";
  {
    LineAppender a;
    ASSERT_TRUE(a.open(path, "% H\n"));
    ASSERT_TRUE(a.append("one"));
  }
  {
    LineAppender a;
    ASSERT_TRUE(a.open(path, "% H\n"));
    ASSERT_TRUE(a.append("two"));
  }
  EXPECT_EQ(slurp(path), "% H\none\ntwo\n");
}

TEST(LineAppender, RepairsAHalfWrittenLastLineWithoutTruncating) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  const std::string path = dir.path() + "/events.log";
  std::ofstream(path, std::ios::binary) << "% H\nhalf";
  LineAppender a;
  ASSERT_TRUE(a.open(path, "% H\n"));
  ASSERT_TRUE(a.append("next"));
  EXPECT_EQ(slurp(path), "% H\nhalf\nnext\n");
}

TEST(LineAppender, EachLineIsFlushed) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  const std::string path = dir.path() + "/events.log";
  LineAppender a;
  ASSERT_TRUE(a.open(path, ""));
  ASSERT_TRUE(a.append("x"));
  EXPECT_EQ(slurp(path), "x\n") << "appender 仍打开时内容已经落盘";
}

TEST(LineAppender, RejectsEmbeddedNewlinesAndReportsFailures) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  LineAppender closed;
  EXPECT_FALSE(closed.append("x"));

  const std::string path = dir.path() + "/events.log";
  LineAppender a;
  ASSERT_TRUE(a.open(path, ""));
  EXPECT_FALSE(a.append("a\nb"));
  EXPECT_EQ(slurp(path), "");

  const std::string blocker = dir.path() + "/file";
  std::ofstream(blocker) << "x";
  LineAppender b;
  EXPECT_FALSE(b.open(blocker + "/events.log", ""));
  EXPECT_FALSE(b.is_open());
}
