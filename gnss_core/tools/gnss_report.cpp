// gnss_report:按 UTC 自然日或时间窗,从 <root>/YYYYMMDD/ 的 .pos、events.log、base.pos 生成
// GNSS 定位报告(自包含 HTML,浏览器打印为 PDF)。用法见 gnss_report --help。spec §3 F2/F3。
#include <iostream>
#include <string>
#include <vector>

#include "gnss_core/report_cli.hpp"

int main(int argc, char** argv) {
  const std::vector<std::string> args(argv + 1, argv + argc);
  return gnss_core::run_gnss_report(args, std::cout, std::cerr);
}
