#pragma once
// GNSS 报告的 HTML 渲染(spec §3 F2/F3,轮 4a):单个自包含文件——内联 CSS 与 SVG,不含脚本与外部资源;
// 浏览器打开后"打印 → 另存为 PDF"。所有来自文件或命令行的文本都经 html_escape。
#include <string>

#include "gnss_core/report_stats.hpp"

namespace gnss_core {

struct ReportMeta {
  std::string root;            // 数据根目录(显示用)
  double generated_at = 0.0;   // 生成时刻,UTC unix 秒
};

// RTKLIB Q → 轨迹颜色,与 rtk-monitor mapview.js 一致:固定绿、浮点黄、DGPS/单点红、其他灰
std::string quality_color(int q);

std::string render_report_html(const ReportStats& stats, const ReportMeta& meta);

}  // namespace gnss_core
