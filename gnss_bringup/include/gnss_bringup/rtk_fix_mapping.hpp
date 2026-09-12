#pragma once
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <gnss_msgs/msg/rtk_fix.hpp>
#include "gnss_core/pos_io.hpp"

namespace gnss_bringup {

// PosRecord(rtkrcv llh 解)→ RtkFix。
// 注意 σ 顺序:PosRecord.sdne 是 RTKLIB 的 (sdn, sde, sdu) = N/E/U,
// 而 RtkFix.sigma_enu 是 E/N/U —— 前两项必须交换(spec §4.1 v2)。
inline gnss_msgs::msg::RtkFix to_rtk_fix(const gnss_core::PosRecord& r) {
  gnss_msgs::msg::RtkFix m;
  m.gnss_time = r.stamp;                 // parse_llh_solution 已换算为 UTC unix 秒
  m.quality = static_cast<uint8_t>(gnss_core::q_to_quality(r.q));
  m.raw_status = static_cast<uint8_t>(r.q);   // 保留 RTKLIB 原始 Q 供追溯
  m.latitude = r.lat;
  m.longitude = r.lon;
  m.altitude = r.height;
  m.sigma_enu[0] = r.sdne(1);            // sde → E
  m.sigma_enu[1] = r.sdne(0);            // sdn → N
  m.sigma_enu[2] = r.sdne(2);            // sdu → U
  m.diff_age = static_cast<float>(r.age);
  m.sats_used = static_cast<uint8_t>(r.ns);
  m.sats_main = 0;
  m.sats_aux = 0;
  m.heading = 0.0f;
  m.heading_sigma = 0.0f;
  m.heading_valid = false;               // rtkrcv 单天线解无双天线航向
  return m;
}

// 增量按行切分器:TcpStream 交付的是任意切分的字节块,不是行。
// feed() 攒内部缓冲,吐出所有已经凑齐的完整行(不含末尾 \n,也会剥掉紧邻的
// \r 以兼容 CRLF);半行留在缓冲里等下一次 feed()。与 gnss_core::RtcmFramer
// 是同一个道理(见其头文件注释)。
//
// review round 1 的 Important 2:原来的实现没有上限——对端如果不按行协议
// 发送(outstr1-format 配错成二进制格式,或者 sol_port 接到了一个完全无关
// 的、不带 '\n' 的字节流上),buf_ 会无界增长,实测 1MB 无换行数据能把 RSS
// 从 57MB 顶到 88MB 且还在涨。max_line_bytes 给单行设一个上限(默认 64KiB,
// 一行 llh 解绰绰有余);超限就整段丢弃、计数、进入"丢弃直到下一个 \n"模式
// 重新同步,不把丢弃前后的字节拼成一条脏行。
class LineSplitter {
public:
  explicit LineSplitter(size_t max_line_bytes = 64 * 1024) : max_line_bytes_(max_line_bytes) {}

  std::vector<std::string> feed(const uint8_t* data, size_t len) {
    buf_.append(reinterpret_cast<const char*>(data), len);
    std::vector<std::string> lines;
    size_t start = 0;
    for (;;) {
      const size_t nl = buf_.find('\n', start);
      if (nl == std::string::npos) break;
      if (discarding_) {
        // 之前因为超限被丢弃的半行,这里终于等到了它的结尾——只是把这段
        // 残留吃掉、恢复正常模式,不把它当成一条(已经被截断、内容不完整
        // 的)行发出去。
        discarding_ = false;
      } else {
        size_t end = nl;
        if (end > start && buf_[end - 1] == '\r') --end;  // 兼容 CRLF
        lines.emplace_back(buf_, start, end - start);
      }
      start = nl + 1;
    }
    buf_.erase(0, start);

    if (buf_.size() > max_line_bytes_) {
      // 单行(到目前为止还没见到 \n 的这一坨)超过上限:对端没有按行协议
      // 发送,或者根本接错了流。丢弃已攒的内容、计数 +1、进入丢弃模式——
      // 后续字节会被继续吞掉,直到遇到下一个 \n 才重新开始正常切分,避免
      // 无界增长,也避免把丢弃前后拼起来的内容误判成一条完整行。
      buf_.clear();
      discarding_ = true;
      ++overflow_count_;
    }
    return lines;
  }

  // 尚未凑成一整行的残留字节数(供诊断/测试用)
  size_t buffered() const { return buf_.size(); }
  // 因为单行超过 max_line_bytes 而被丢弃的次数(供节点侧限流打日志用)
  size_t overflow_count() const { return overflow_count_; }
  // 是否处于"丢弃直到下一个 \n"模式(供测试验证 reset() 真的把这个状态也
  // 清掉了——round 2 review 指出的测试缺口:只测 buffered()==0 的话,一个
  // 清空了缓冲区、却忘记把 discarding_ 也复位成 false 的 reset() 实现照样
  // 能通过测试,但会把重连后第一条真正的新行也当成"还在丢弃中的残留"吞掉,
  // 是与 finding 4 同一类的静默丢数据)。
  bool discarding() const { return discarding_; }
  void reset() {
    buf_.clear();
    discarding_ = false;
  }

private:
  std::string buf_;
  size_t max_line_bytes_;
  bool discarding_ = false;
  size_t overflow_count_ = 0;
};

// 校验一个"必须是正数的秒数"参数(轮询间隔、重启退避、空闲超时……)。
// <=0 或非有限值(NaN/inf)如果放过去,会在各自的使用点造成热循环或者钉死
// 的行为——例如 stat_poll_interval_s<=0 会让 tail 线程的 condition_variable
// wait_for 立刻超时返回,变成一个忙轮询扫 run_dir、跑满一个核的循环。与
// rtcm_bridge_params::is_valid_port 是同一个道理:提前挡住、报得清楚,而
// 不是让它在运行时变成一个隐蔽的性能坑。
inline bool is_positive_finite_seconds(double v) {
  return std::isfinite(v) && v > 0.0;
}

// ---------- rtkrcv *.stat tail 的纯决策逻辑 ----------
// review round 1 的 Important 3:抽成纯函数,好单测覆盖三类坑——
//   1) 进程刚起来时选中的是已经存在的旧文件(比如上一轮遗留的
//      rtkrcv_20200101000000.stat),不能把它的全部历史内容当成"新增"
//      一次性吐给下游诊断(会把上一轮的历元冒充成实时数据)。
//   2) 运行期间才出现的新文件(rtkrcv 重启后开的),必须从头开始 tail,
//      不能漏内容。
//   3) 同名文件被截断或者重新创建(size 比记录的 offset 还小),必须把
//      offset 归零重新开始,而不是永远卡在"没有新内容"的判断上再也读不到
//      东西。

// 一个候选 .stat 文件的最小信息。mtime 用一个任意的、只保证"值越大越新"的
// 整数计数表示(调用方从 std::filesystem::file_time_type::time_since_epoch()
// 转换而来),不直接依赖 std::filesystem 类型,方便跨平台单测。
struct StatFileInfo {
  std::string path;
  uint64_t size = 0;
  int64_t mtime = 0;
};

struct StatTailDecision {
  bool has_target = false;   // candidates 是否非空(目录里有没有候选文件)
  std::string file;          // 要 tail 的文件(等于按 mtime 选出的最新文件)
  uint64_t read_from = 0;    // 从这个偏移开始读
  uint64_t read_to = 0;      // 读到这个偏移为止(即该文件当前 size);
                             // read_to <= read_from 表示这次没有新内容
};

// candidates:目录下所有候选 .stat 文件(调用方已经过滤好扩展名)。
// current_file/current_offset:上一次 tail 到的文件与偏移("" / 0 表示
// tail 线程从未真正处理过任何文件)。
// is_first_poll:这是不是 tail 线程启动以来的第一轮轮询——不是"第一次看到
// 某个文件",而是字面意义的"循环第 0 次"。这一轮看到的任何文件,不管它是
// 不是刚被创建,都只能是"这一刻已经存在"的文件,视为上一轮遗留的旧文件,
// 跳到文件末尾开始 tail;之后任何一轮里"当前最新文件"发生变化,那个新文件
// 必然是运行期间才出现/才变成最新的,从 0 开始 tail。
inline StatTailDecision plan_stat_tail(const std::vector<StatFileInfo>& candidates,
                                        const std::string& current_file,
                                        uint64_t current_offset,
                                        bool is_first_poll) {
  StatTailDecision d;
  if (candidates.empty()) return d;

  const StatFileInfo* latest = &candidates.front();
  for (const auto& c : candidates) {
    if (c.mtime > latest->mtime) latest = &c;
  }

  d.has_target = true;
  d.file = latest->path;

  uint64_t offset;
  if (latest->path != current_file) {
    offset = is_first_poll ? latest->size : 0;
  } else {
    offset = current_offset;
    if (latest->size < offset) {
      offset = 0;   // 被截断或者同名重开:归零重新开始,不然永远读不到新内容
    }
  }

  d.read_from = offset;
  d.read_to = latest->size;
  return d;
}

}  // namespace gnss_bringup
