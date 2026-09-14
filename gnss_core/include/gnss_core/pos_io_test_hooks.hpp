#pragma once
// 仅供 test_pos_io.cpp 使用的注入点——round 2 review(final-fix-report.md
// 之后又发现的 BLOCKING 1)要求证明"读取 .pos 文件末尾不完整行时发生 I/O
// 错误"这个场景被正确处理成 open() 失败、文件原封不动,而不是被误判成
// "最后一行不完整"去做一次基于错误数据的截断(reviewer 当时是用
// LD_PRELOAD 在真实二进制上打进程级的 EIO 复现的;在 gtest 里更干净、
// 确定性更强的做法是给这一步单独开一个可注入的钩子)。
//
// 这个钩子只影响 gnss_core/src/pos_io.cpp 里判断"最后一行是否完整"这一小
// 段代码,不出现在任何逐条记录的热路径上(open() 每次打开文件最多调用几
// 次,不是 write() 的一部分),生产代码里这个函数指针恒为 nullptr,多出的
// 开销只是 open() 内部一次"指针是否为空"的判断。不引入任何 ROS 依赖。
namespace gnss_core::testing {

// step 标识读取过程走到了哪一步:
//   "last_byte"    —— 读文件最后一个字节,判断它是不是 '\n'
//   "full_content" —— 把整份文件读进内存,定位最后一个 '\n' 以确定截断点
// 返回 true 表示"在这一步注入一次失败",调用方(truncate_incomplete_
// trailing_line)必须把它当成真实的读失败处理:不截断、不继续、把失败
// 原样报给 PosWriter::open() 的调用方(返回 false)。
using TrailingLineReadFailureInjector = bool (*)(const char* step);

// 设置/清除注入器(传 nullptr 清除)。全局、非线程安全——单测里用完必须
// 立刻恢复成 nullptr,不能跨测试用例残留,否则会污染后面无关的用例。
void set_trailing_line_read_failure_injector(TrailingLineReadFailureInjector injector);

}  // namespace gnss_core::testing
