#pragma once
#include <cmath>
#include <cstdio>
#include <ctime>
#include <limits>
#include <string>

namespace gnss_bringup {

// ---------- .pos 按天轮转的纯决策逻辑 ----------
// spec §5.3:目录布局是 <root>/YYYYMMDD/<source>.pos,按天轮转。把"这条记录
// 该写哪个文件"和"要不要因此换文件"做成纯函数,节点只负责在 should_rotate()
// 为真时关旧开新——与上一轮 rtk_fix_mapping.hpp 里的 plan_stat_tail 同样的
// 处理方式,理由也一样:时间与路径的决策是最容易出错、又最容易测的部分。

namespace detail {

// source 必须能安全地拼进 "<root>/YYYYMMDD/<source>.pos" 这一段文件名里:
// 不能为空,也不能带路径分隔符——否则会逃出预期目录(比如
// source="../../etc/cron.d/x" 这种路径穿越)。
inline bool is_valid_pos_source(const std::string& source) {
  return !source.empty() && source.find('/') == std::string::npos;
}

}  // namespace detail

// <root>/YYYYMMDD/<filename>,YYYYMMDD 按 UTC。拒绝规则与下面 pos_path_for 的说明相同
// (空 root、filename 为空/带 '/'/是 "." 或 ".."、时间非有限或超出 time_t),失败返回空串。
// .pos、events.log、base.pos 共用这一套日期目录规则(spec §5.3)。
inline std::string day_file_path(const std::string& root, const std::string& filename, double utc_stamp) {
  if (root.empty()) return "";
  if (!detail::is_valid_pos_source(filename) || filename == "." || filename == "..") return "";

  // NaN/±inf,或者超出 time_t 能表示范围的值:double→time_t 的窄化转换在
  // 这种输入下是未定义行为(标准没有规定结果),必须在转换前挡住,而不是
  // 让它变成一个随机、不可复现的路径。
  if (!std::isfinite(utc_stamp) ||
      utc_stamp < static_cast<double>(std::numeric_limits<std::time_t>::min()) ||
      utc_stamp > static_cast<double>(std::numeric_limits<std::time_t>::max())) {
    return "";
  }

  const std::time_t tt = static_cast<std::time_t>(std::floor(utc_stamp));
  std::tm tm{};
  if (gmtime_r(&tt, &tm) == nullptr) return "";  // 双重保险:libc 自己判定转换无效

  char datebuf[16];
  std::snprintf(datebuf, sizeof(datebuf), "%04d%02d%02d",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);

  // 去掉 root 末尾多余的 '/'(不管有几个),再统一补一个,这样
  // "/data/gnss" 和 "/data/gnss/" 产出同一个路径。root 本身就是纯斜杠
  // (比如就是文件系统根 "/")时,去重之后不能变成空串——那样会绕过上面
  // 的空 root 拒绝逻辑,也会拼出缺了前导 '/' 的相对路径。
  std::string r = root;
  while (r.size() > 1 && r.back() == '/') r.pop_back();
  if (r == "/") {
    return r + datebuf + "/" + filename;
  }
  return r + "/" + datebuf + "/" + filename;
}

// 计算记录应当落盘的路径:<root>/YYYYMMDD/<source>.pos,YYYYMMDD 按 UTC 取
// (必须用 gmtime_r,不能用 localtime_r——原因与 gnss_core::pos_io.cpp 里的
// 时间处理相同:同一份数据在不同时区的机器上重放,必须落进同一个日期目录)。
//
// 失败(返回空字符串)的三类输入,选择"可见地失败"而不是悄悄拼出一个能用
// 但没意义的路径:
//   1) root 为空——拼出来会变成 "/YYYYMMDD/source.pos" 这种指向文件系统根
//      目录的绝对路径,写穿了比报错更危险。
//   2) source 为空,或者带路径分隔符——同样是路径穿越/落错位置的风险。
//   3) utc_stamp 不是有限数(NaN/±inf),或者超出 time_t 能表示的范围——
//      转换成 time_t 是未定义行为,不能让它悄悄溜过去变成一个随机路径。
// 这条函数保持纯粹、不抛异常(调用方在下一个任务里是 ROS 订阅回调,一次
// 格式错误的输入不该打断整条订阅链路),调用方看到空字符串就应该跳过这条
// 记录并计数/打日志,而不是把它当成合法路径去开文件。
//
// 0 和其他"合法但离谱"的 utc_stamp(比如负数、代表 1970 年之前的时间)不在
// 上面三类拒绝范围内——上一个任务在 gnss_time 和 header.stamp 都是 0 的
// 极端情况下确实会把 0 传下来,这是真实可能发生的输入,不是需要挡住的坏
// 输入。此时函数按 UTC 纪元零点正常换算,落进 "19700101" 目录:这本身就是
// "可见地失败"的一种形式——产出的路径肉眼一看就不正常,比悄悄映射到"今天"
// 之类的默认值更容易被发现和排查。
inline std::string pos_path_for(const std::string& root, const std::string& source, double utc_stamp) {
  if (!detail::is_valid_pos_source(source)) return "";
  return day_file_path(root, source + ".pos", utc_stamp);
}

// current_path 是节点当前打开着的文件路径("" 表示还没打开过任何文件);
// next_path 是 pos_path_for() 刚算出来的、这条新记录该去的路径。两者不同
// 就该轮转(含首次打开——current_path=="" 时视为"换文件",这样调用方不用
// 再单独判断"是不是第一条记录")。
inline bool should_rotate(const std::string& current_path, const std::string& next_path) {
  return current_path != next_path;
}

}  // namespace gnss_bringup
