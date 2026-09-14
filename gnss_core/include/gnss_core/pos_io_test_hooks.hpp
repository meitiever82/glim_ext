#pragma once
// 仅供 test_pos_io.cpp 使用的注入点——round 2 review(final-fix-report.md
// 之后又发现的 BLOCKING 1)要求证明"读取 .pos 文件末尾不完整行时发生 I/O
// 错误"这个场景被正确处理成 open() 失败、文件原封不动,而不是被误判成
// "最后一行不完整"去做一次基于错误数据的截断(reviewer 当时是用
// LD_PRELOAD 在真实二进制上打进程级的 EIO 复现的;在 gtest 里更干净、
// 确定性更强的做法是给这一步单独开一个可注入的钩子)。
//
// round 3 review 前后一共纠正过两次这个钩子的用法,教训是同一个:
// **必须让"这一步"本身真的失败,而不是检查一个事后补上的判定值**——
// 一个测出来只是"这个 if 分支存在"的注入点,删掉它要防的那段代码,测试
// 也不会失败。
//
// 但"真的失败"具体长什么样,取决于这一步在 libstdc++ 里到底是怎么读的,
// 两种失败方式并不一样:
//   - "full_content" 用 istreambuf_iterator 直接操作 streambuf,绕开了
//     istream::sentry——真实 I/O 错误发生时,basic_filebuf::underflow()
//     会*无条件*抛出 std::ios_base::failure,不受这个流的 exceptions()
//     掩码影响。这一步的注入因此也用抛异常模拟(见 pos_io.cpp 的
//     throw_if_injected),让 pos_io.cpp 里包着这段读取的 try/catch 是
//     真的被测试覆盖到的代码——删掉那个 try/catch,注入的异常就会一路
//     捅穿,测试会因为未捕获异常而失败,而不是安静地继续通过。
//   - "last_byte"(in.get())和 "read_pos_scan"(read_pos() 内部的
//     std::getline)都是**格式化的** istream 操作,会经过 sentry:真实
//     I/O 错误发生时,sentry 会吞掉 underflow() 抛出的异常、把流设成
//     badbit,并且因为默认的 exceptions() 掩码是 goodbit,**不会重新
//     抛出来**——这两处的真实失败方式根本不是"抛异常"。"read_pos_scan"
//     的注入因此直接把流设成 badbit(不抛),精确复现这个真实失败方式,
//     测的是 read_pos() 循环结束后有没有检查 in.bad()。"last_byte" 仍然
//     用抛异常模拟——这不是在复现一个观察到的真实失败方式(get() 真的
//     遇到 I/O 错误时不会抛,`if (!in) return kFailed` 这一句已经足够接住
//     它),而是防御性地确认"就算这里将来因为某种实现差异真的抛出来,
//     周围的 try/catch 也接得住"。两处的最终行为都是同一个 kFailed,只是
//     "last_byte" 这一支没有对应的、真实会发生的失败路径需要单独证明。
//   - "resize" 走的是 std::error_code 而不是异常(std::filesystem::
//     resize_file 的这个重载不抛),所以是直接让 gnss_core::PosWriter::
//     open() 认为 resize_file() 失败了,同时确保真正的 resize_file()
//     系统调用被跳过、文件在磁盘上一字节都不会被动过。
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
//   "last_byte"     —— 读文件最后一个字节,判断它是不是 '\n'(防御性注入,
//                      见上面的类注释——真实 I/O 错误在这里不会抛异常)
//   "full_content"  —— 把整份文件读进内存,定位最后一个 '\n' 以确定截断点
//                      (真实会无条件抛异常的那一步)
//   "resize"        —— 真正执行 resize_file() 把不完整的末行截掉
//   "read_pos_scan" —— read_pos() 内部 std::getline 循环的每一次迭代
//                      (真实失败方式是安静地把流设成 badbit,不抛异常;
//                      round 3 review 第三次纠正的对象)
//   "read_pos_premature_eof" —— 模拟"read() 提前返回 0"(FUSE/NFS 或文件
//                      被并发截断):循环正常结束、流上不设 badbit,只能靠
//                      "扫描消费的字节数 != 文件大小"识别(round 2 hardening
//                      Task 1)
// 返回 true 表示"在这一步注入一次失败"。调用方必须把它当成真实的失败
// 处理:不截断、不继续、把失败原样报给 PosWriter::open() 的调用方(返回
// false)或者从 read_pos() 抛出来,且磁盘上的文件不能有任何变化。
using TrailingLineReadFailureInjector = bool (*)(const char* step);

// 设置/清除注入器(传 nullptr 清除)。全局、非线程安全——单测里用完必须
// 立刻恢复成 nullptr,不能跨测试用例残留,否则会污染后面无关的用例。
void set_trailing_line_read_failure_injector(TrailingLineReadFailureInjector injector);

}  // namespace gnss_core::testing
