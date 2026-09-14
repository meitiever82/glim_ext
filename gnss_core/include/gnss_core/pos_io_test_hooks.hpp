#pragma once
// 仅供 test_pos_io.cpp 使用的注入点——round 2 review(final-fix-report.md
// 之后又发现的 BLOCKING 1)要求证明"读取 .pos 文件末尾不完整行时发生 I/O
// 错误"这个场景被正确处理成 open() 失败、文件原封不动,而不是被误判成
// "最后一行不完整"去做一次基于错误数据的截断(reviewer 当时是用
// LD_PRELOAD 在真实二进制上打进程级的 EIO 复现的;在 gtest 里更干净、
// 确定性更强的做法是给这一步单独开一个可注入的钩子)。
//
// round 3 review 纠正了这个钩子最初的用法:一开始只是"读成功之后再问
// 一句要不要假装失败",这测不出"读操作本身被 try/catch 保护住"这件事——
// libstdc++ 在真实 I/O 错误时会从读操作内部直接抛异常,不是读完之后才能
// 查到的一个状态位。因此"last_byte"/"full_content" 这两步现在会在真正
// 读取之前直接抛出一个 std::ios_base::failure,模拟同一种失败方式(见
// pos_io.cpp 的 throw_if_injected);"resize" 这一步走的是 std::error_code
// 而不是异常,所以是直接让 gnss_core::PosWriter::open() 认为 resize_file()
// 失败了,同时确保真正的 resize_file() 系统调用被跳过、文件在磁盘上一字
// 节都不会被动过。
//
// 只在 BUILD_TESTING 时存在:这个头文件不会被安装到非测试构建的 include/
// 里(见 CMakeLists.txt 的 install() 排除规则),pos_io.cpp 里对应的实现也
// 整段包在同一个 GNSS_CORE_WITH_TEST_HOOKS 宏后面——一个测试专用的注入点
// 不应该出现在正式发布的 libgnss_core.so 导出符号表里。生产构建里,
// open() 每次打开文件最多调用这个钩子对应的判空逻辑几次(不在 write() 的
// 热路径上),而在 BUILD_TESTING=OFF 时这段代码根本不存在,连"判空"这一
// 点点开销都没有。不引入任何 ROS 依赖。
namespace gnss_core::testing {

// step 标识读取/截断/去重扫描过程走到了哪一步:
//   "last_byte"    —— 读文件最后一个字节,判断它是不是 '\n'
//   "full_content" —— 把整份文件读进内存,定位最后一个 '\n' 以确定截断点
//   "resize"       —— 真正执行 resize_file() 把不完整的末行截掉
//   "dedup_scan"   —— PosWriter::open() 调用 read_pos() 收集去重键这一步
//                     (round 3 review:这里原来的 catch 会静默吞掉异常、
//                     照样返回 true——跟上面三步同一条"读失败就必须让
//                     open() 直接失败"的规则)
// 返回 true 表示"在这一步注入一次失败",调用方必须把它当成真实的失败
// 处理:不截断、不继续、把失败原样报给 PosWriter::open() 的调用方(返回
// false),且磁盘上的文件不能有任何变化。
using TrailingLineReadFailureInjector = bool (*)(const char* step);

// 设置/清除注入器(传 nullptr 清除)。全局、非线程安全——单测里用完必须
// 立刻恢复成 nullptr,不能跨测试用例残留,否则会污染后面无关的用例。
void set_trailing_line_read_failure_injector(TrailingLineReadFailureInjector injector);

}  // namespace gnss_core::testing
