#pragma once
#include <cstddef>
#include <fstream>
#include <set>
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
// round 2 复盘(final-fix-report.md 之后又发现的三个数据正确性 bug)的
// 去重设计,替换了最初那版"stamp <= 文件最后一条的 stamp"的一维分界线:
// 分界线规则有三个问题——(a) 文件只存到毫秒,裸浮点 stamp 与它做 "<="
// 比较时,亚毫秒级的边界差异大约一半概率被错误地放过/挡住;(b) 任何早于
// 文件末尾的合法补录(重放一段填补更早时段空洞的 bag 段)会被整体判定为
// "重叠",连同它本该落盘的新数据一起悄悄丢弃,日志却还说"不是数据丢失";
// (c) 分界线只在 open() 时定一次,同一次运行内两次传入完全相同的 stamp
// 完全不受保护。
//
// 现在的规则是精确成员判定,粒度就是文件自己的分辨率(毫秒):
//   - open() 时,对已有内容的每一行,复用 read_pos()(不写第二个 .pos
//     解析器)解出的 PosRecord,按当前 open() 使用的 ts_/leap_ 重新算出
//     format_pos_record() 会渲染出的那个整数毫秒键(pos_io.cpp 内部的
//     record_time_key_ms,与 format_pos_record 共用同一套取整/进位逻辑),
//     塞进 existing_keys_——"文件里已经有这一行"和"这个毫秒键在集合里"
//     是同一件事。
//   - write() 只在这个毫秒键已经在集合里时才判定为重复、跳过(返回 true,
//     不是 I/O 失败,计数供调用方报一次);否则写盘并立刻把键加入集合。
//   - 不变量:同一个毫秒键在整份文件的生命周期里(跨越多次 open() 会话,
//     也包括同一次运行内)至多出现一次——因此不再需要依赖上层
//     PosDecimator 替它兜底同一次运行内的重复。
//   - 副作用(刻意允许):这允许乱序追加。.pos 文件本身从不需要"整体按
//     时间有序"这个不变量——它是逐行数据,RTKLIB 的读者和这里的 read_pos
//     都是逐行解析,不关心顺序;gnss_core 内唯一因此需要留意的地方是
//     lever_arm_estimator 的命令行工具 estimate_lever_arm.cpp,它用
//     fixes.front()/back() 的 stamp 打印时间范围、计算与轨迹的重叠—— 那
//     两行诊断信息假定了输入有序,乱序追加后可能打印出误导性的范围/误判
//     "无重叠"(不影响 estimate_lever_arm() 本身的最小二乘解,build_pairs
//     只是线性扫描 fixes,不要求有序);trajectory_compare::compare_by_quality
//     已经自己对 ref 排序、线性扫描 test,不受影响。
//
// 另一个独立的崩溃场景(与上面的去重规则正交):断电导致文件最后一行只
// 写了一半、没有换行结尾。open() 一个已存在且非空的文件时,如果最后一个
// 字节不是 '\n',视为"这一行从未真正写完"——原地把文件截断到最后一个
// '\n' 之后(找不到则截到空文件),并记一次"丢弃了一条不完整的行"
// (discarded_incomplete_line()),供调用方报一次——这是真正的数据丢失
// (丢了半条记录),必须与"重复,不是丢失"的 suppressed_duplicate_count
// 分开报告,不能混着算。
// 理由:一条记录存在与否,以"它的完整换行行是否存在"为准,而不是"凑巧能
// 解析出某些列"——半行残留时被截断的数值列完全可能凑巧解析出一个合理但
// 错误的值(比如 ratio 列只写了半个数字),留着不比丢掉更安全;而且旧实现
// 里 write() 会直接接着这半行继续追加,导致下一条记录被物理粘连、连同
// 这半行一起在 read_pos 里读成一整行错误数据。
// 注意:两个写者同时打开同一个文件本来就是不支持的误配置——这里的截断
// 可能会切掉另一个写者尚未写完的那一行,不额外做互斥保护。
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
  // 这条记录渲染出的毫秒键如果已经在文件里(见类注释),会被静默去重
  // (不落盘)但仍然返回 true——这不是 I/O 失败,调用方用
  // last_write_was_suppressed()/suppressed_duplicate_count() 判断要不要
  // 报一次去重日志。
  bool write(const PosRecord& r);
  void close();
  bool is_open() const { return out_.is_open(); }
  const std::string& path() const { return path_; }

  // 最近一次 write() 是不是因为这个毫秒键已经在文件里而被跳过(没有真正
  // 落盘)。现在是精确成员判定,凡是被跳过的都是文件里已经原样存在的同一
  // 条记录——"不是数据丢失"这句话因此是准确的(不像旧的"<=分界线"规则
  // 会连带丢掉合法的补录数据)。
  bool last_write_was_suppressed() const { return last_write_suppressed_; }
  // 自本次 open() 以来,因为毫秒键已存在而被跳过的记录累计数——每次
  // open() 重新计数为 0。调用方据此报一次"跳过了多少条",而不是为区间里
  // 的每一条都打一行日志。
  std::size_t suppressed_duplicate_count() const { return suppressed_duplicate_count_; }
  // 本次 open() 是否在打开一个已有文件时,发现最后一行没有换行结尾
  // (断电/崩溃留下的半行)并把它原地截掉了。这是真正丢了数据(半条
  // 记录),必须与"重复,不是丢失"的 suppressed_duplicate_count 分开
  // 报告——每次 open() 至多为 true 一次,不会跨记录重复触发。
  bool discarded_incomplete_line() const { return discarded_incomplete_line_; }

private:
  std::ofstream out_;
  std::string path_;
  PosTimeSystem ts_ = PosTimeSystem::GPST;
  int leap_ = 18;

  // 已经在文件里(含本次会话已经成功写过)的记录的毫秒键——见类注释。
  std::set<long long> existing_keys_;
  bool last_write_suppressed_ = false;
  std::size_t suppressed_duplicate_count_ = 0;
  bool discarded_incomplete_line_ = false;
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
