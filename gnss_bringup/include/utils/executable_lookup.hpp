#pragma once
#include <string>

namespace gnss_bringup {

// 把 ProcessSupervisor 的 binary 解析成可以直接交给 execv() 的路径;找不到返回空串。
//   - name 含 '/':不查 PATH,只检查它是不是可执行的普通文件;相对路径转成绝对路径
//     (子进程 exec 之前会 chdir 到 run_dir,相对路径到那里就失效了)。
//   - 否则按 path_env(冒号分隔)依次查找第一个可执行的普通文件。空段(POSIX 里表示
//     当前目录)被忽略:子进程的 cwd 是 run_dir,那里不该被当作查找目录。
std::string resolve_executable(const std::string& name, const char* path_env);

}  // namespace gnss_bringup
