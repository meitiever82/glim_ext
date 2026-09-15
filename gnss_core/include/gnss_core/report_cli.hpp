#pragma once
// gnss_report 命令行(spec §3 F2/F3,轮 4a)。参数解析与执行放在库里以便单测,tools/gnss_report.cpp 只是 main。
#include <ostream>
#include <string>
#include <vector>

namespace gnss_core {

// args 不含程序名。返回退出码:0 成功;1 参数错误(err 里是原因与用法);2 根目录读不了或报告写不出。
int run_gnss_report(const std::vector<std::string>& args, std::ostream& out, std::ostream& err);

}  // namespace gnss_core
