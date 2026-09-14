#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
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
