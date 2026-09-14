#pragma once
#include <cstddef>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include "gnss_core/types.hpp"

namespace gnss_core {

// RTKLIB .pos 一条记录(spec §5.3)。
struct PosRecord {
  double stamp = 0.0;           // UTC unix seconds(read_pos 已统一换算)
  double lat = 0.0, lon = 0.0, height = 0.0;   // WGS-84 deg/deg/m(椭球高)
  int q = 0;                    // RTKLIB Q: 1=fix 2=float 4=dgps 5=single
  int ns = 0;                   // 卫星数
  Eigen::Vector3d sdne = Eigen::Vector3d::Zero();   // sdn, sde, sdu(m)
  double age = 0.0;             // 差分龄期(s)
  double ratio = 0.0;           // AR ratio
};

// 时间系统:RTKLIB .pos 默认 GPST(比 UTC 快 leap_seconds),头部 "% (... time=GPST)" / "(... time=UTC)" 标明。
// read_pos 统一输出 UTC unix 秒:GPST 减 leap_seconds;UTC 原样;无头部标注时按 default_time_system。
enum class PosTimeSystem { GPST, UTC };

struct PosReadOptions {
  int leap_seconds = 18;
  PosTimeSystem default_time_system = PosTimeSystem::GPST;
};

// 读取 .pos:跳过 % 注释行(但扫描其中 time=GPST/UTC);
// 数据列 "YYYY/MM/DD HH:MM:SS.sss lat lon height Q ns sdn sde sdu sdne sdeu sdun age ratio"。
// 列数不足 10(到 sdu)的行跳过;age/ratio 缺省为 0。文件打不开抛 std::runtime_error。
std::vector<PosRecord> read_pos(const std::string& path, const PosReadOptions& opt = {});

// 写标准 RTKLIB .pos(头 "% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,...,time=GPST|UTC)" + 列名注释 + 14 列数据)。
// records.stamp 为 UTC unix 秒;time_system=GPST 时写出时间加 leap_seconds。
void write_pos(const std::string& path, const std::vector<PosRecord>& records,
               PosTimeSystem time_system = PosTimeSystem::GPST, int leap_seconds = 18);

// .pos 表头三行(program 行 + 说明行 + 列名行),write_pos 与 PosWriter 共用,格式只写一遍。
std::string pos_header(PosTimeSystem time_system);

// 单条记录格式化为 .pos 数据行(含末尾换行),write_pos 与 PosWriter 共用。
// r.stamp 为 UTC unix 秒;time_system=GPST 时输出时间加 leap_seconds。
std::string format_pos_record(const PosRecord& r, PosTimeSystem time_system, int leap_seconds);

// 追加式 .pos 写出器(spec §5.3:追加写、崩溃安全)。
// 与 write_pos 共用同一套表头与单行格式化,格式只写一遍。
// 打开已存在且非空的文件时不再重写表头,直接续写——因此进程重启不会破坏文件。
//
// final-fix-wave 第 1 项(重放/重启接续同一个文件会重复写行):open() 一个
// 已存在且非空的文件时,复用 read_pos()(不写第二个 .pos 解析器)取文件里
// 最后一条能被成功解析的记录的 stamp,作为这次续写会话的"分界线"。此后
// write() 里任何 stamp <= 分界线的记录都被判定为与文件里已有内容重叠
// (bag 重放、进程重启后重新收到同一段数据、或误起了第二个实例都会产生这种
// 输入),悄悄跳过、不落盘——但不当成错误(返回 true),只计数,供调用方
// 报一次"跳过了多少条"。
//
// 分界线只在 open() 时确定一次,后续同一个会话里的 write() 不会更新它——
// 因此同一次运行内(比如两次调用传入完全相同的 stamp)不受这条去重逻辑
// 影响,这是刻意的:1 Hz 抽稀之类的"同一次运行内不应该出现重复"由更上层的
// PosDecimator 负责,PosWriter 自己只管"重新打开一个已经有内容的文件时,
// 不要把已经在磁盘上的东西再写一遍"这一件事,职责边界清楚。
//
// 断电导致文件最后一行只写了一半(列数不够 10 列)的情况:parse_llh_solution
// 本来就会跳过列数不足的行,分界线因此自然落在它之前最后一条完整记录上;
// 残缺的半行本身既不会被信任为分界线,也不会被就地修复——write() 只管在
// 它后面继续追加。
class PosWriter {
public:
  PosWriter() = default;
  explicit PosWriter(PosTimeSystem ts, int leap_seconds = 18) : ts_(ts), leap_(leap_seconds) {}
  ~PosWriter();
  PosWriter(const PosWriter&) = delete;
  PosWriter& operator=(const PosWriter&) = delete;

  // 打开(追加模式)。父目录不存在时创建。失败返回 false,不抛。
  bool open(const std::string& path);
  // 追加一条并 flush —— 崩溃安全的代价是每条一次 flush,1 Hz 下可忽略。
  // stamp <= 续写分界线的记录会被静默去重(不落盘)但仍然返回 true——
  // 这不是 I/O 失败,调用方用 last_write_was_suppressed()/
  // suppressed_duplicate_count() 判断要不要报一次去重日志。
  bool write(const PosRecord& r);
  void close();
  bool is_open() const { return out_.is_open(); }
  const std::string& path() const { return path_; }

  // 最近一次 write() 是不是因为与续写分界线重叠而被跳过(没有真正落盘)。
  bool last_write_was_suppressed() const { return last_write_suppressed_; }
  // 自本次 open() 以来,因为与文件里已有内容重叠而被跳过的记录累计数——
  // 每次 open() 重新计数为 0。调用方据此报一次"跳过了多少条",而不是为
  // 区间里的每一条都打一行日志。
  std::size_t suppressed_duplicate_count() const { return suppressed_duplicate_count_; }

private:
  std::ofstream out_;
  std::string path_;
  PosTimeSystem ts_ = PosTimeSystem::GPST;
  int leap_ = 18;

  bool has_resume_cutoff_ = false;
  double resume_cutoff_ = 0.0;
  bool last_write_suppressed_ = false;
  std::size_t suppressed_duplicate_count_ = 0;
};

// RTKLIB Q → 归一化质量:1→FIXED 2→FLOAT 4→DGPS 5→SINGLE 其它→NONE
Quality q_to_quality(int q);

// 1 Hz 抽稀(spec §5.3:rosbag2 存全量原始流,.pos 只存 1 Hz 摘要)。
// 按 floor(stamp / period) 分桶,每个桶只放行第一条 —— 桶边界对齐整秒,
// 因此输出的时间戳分布与 RTKLIB 1 Hz .pos 一致。
//
// round 2 review 的 Important 2(经 gnss_bringup 侧复测证实):最初的规则是
// "桶号只要变化就放行",本意是让时钟回跳后不会卡在"永远追不上上次+1s"——
// 但这条规则对着抖动同样成立,而抖动是真实存在的场景:一个按接收时刻(而非
// 板卡历元)打时间戳的源,相邻两条记录完全可能落在整秒边界两侧来回摆动
// (…625.995→626.003→625.998→626.001…),"桶号变化就放行"会让这种摆动的
// 每一次跳变都被当成新桶放行——10 Hz 输入实测被写出 30 行而不是 3 行,直接
// 污染 .pos 这个"1 Hz 摘要"文件的意义(它是下游标定量测权重系数的输入)。
//
// 修复:只有桶号真正前进(bin > bin_)才无条件放行;桶号后退时,后退超过
// kJitterToleranceBins 个桶才算"真的跳变"(时钟被拨回去了,必须继续输出,
// 不能等到追上才恢复),否则按抖动处理、丢弃。kJitterToleranceBins=1 恰好
// 挡住"在同一条整秒边界两侧来回摆动"这种最常见的抖动模式(相邻桶号只差 1),
// 同时对 ClockJumpBackwardsDoesNotStallOutput 覆盖的真实回跳场景(相差
// 1000 个桶,远超过 1)完全不受影响——它就是为区分"抖动"与"真跳变"存在的
// 阈值,不是随手挑的数字。
class PosDecimator {
public:
  explicit PosDecimator(double period_s = 1.0) : period_(period_s) {}
  // 该条应当写出则返回 true
  bool accept(const PosRecord& r);
  void reset() { has_bin_ = false; }

private:
  // 桶号后退不超过这么多格,按抖动处理(丢弃);超过则按真实时钟回跳处理
  // (放行,避免永远追不上)。见上面类注释。
  static constexpr long long kJitterToleranceBins = 1;

  double period_;
  long long bin_ = 0;
  bool has_bin_ = false;
};

}  // namespace gnss_core
