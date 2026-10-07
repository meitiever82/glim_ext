// pos_to_enu —— RTKLIB .pos → 局部 ENU 轨迹文本(轮 5 Task 2;spec §12.4 的 gnss_global 输入)
//
// usage: pos_to_enu <in.pos> <out.enu> [--origin lat lon alt] [--min-quality 3]
//                   [--pos-time GPST|UTC] [--leap 18]
//
// - --origin 缺省取 .pos 首个达标历元;§12.4 要求与 gt/*.tum 同原点,所以实际使用时
//   务必显式传 <seg>/gt/enu_origin.txt 里的三个数。
// - --min-quality 用归一化枚举:4=FIXED 3=FLOAT 2=DGPS 1=SINGLE 0=不过滤。
// - --pos-time 只在 .pos 没有 "% (... time=GPST|UTC)" 头时生效(RTKLIB 约定默认 GPST)。
// - 输出格式见 gnss_core/enu_track.hpp;时间列是 UTC unix 秒。
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include "gnss_core/enu_track.hpp"
#include "gnss_core/pos_io.hpp"

namespace {

void usage() {
  std::fprintf(stderr,
               "usage: pos_to_enu <in.pos> <out.enu> [--origin lat lon alt] [--min-quality 3]\n"
               "                  [--pos-time GPST|UTC] [--leap 18]\n");
}

bool need(int argc, int i, int n) {
  if (i + n >= argc) {
    usage();
    std::exit(2);
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    usage();
    return 2;
  }
  const std::string in_path = argv[1], out_path = argv[2];

  gnss_core::EnuTrackOptions opt;
  gnss_core::PosReadOptions ropt;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--origin" && need(argc, i, 3)) {
      opt.has_origin = true;
      opt.origin_lla = Eigen::Vector3d(std::atof(argv[i + 1]), std::atof(argv[i + 2]), std::atof(argv[i + 3]));
      i += 3;
    } else if (a == "--min-quality" && need(argc, i, 1)) {
      opt.min_quality = static_cast<gnss_core::Quality>(std::atoi(argv[++i]));
    } else if (a == "--pos-time" && need(argc, i, 1)) {
      const std::string v = argv[++i];
      ropt.default_time_system = (v == "UTC") ? gnss_core::PosTimeSystem::UTC : gnss_core::PosTimeSystem::GPST;
    } else if (a == "--leap" && need(argc, i, 1)) {
      ropt.leap_seconds = std::atoi(argv[++i]);
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
      usage();
      return 2;
    }
  }

  std::vector<gnss_core::PosRecord> recs;
  try {
    recs = gnss_core::read_pos(in_path, ropt);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  if (recs.empty()) {
    std::fprintf(stderr, "error: no epochs in %s (check the time column / header time system)\n", in_path.c_str());
    return 1;
  }

  const gnss_core::EnuTrack track = gnss_core::make_enu_track(recs, opt);
  if (track.samples.empty()) {
    std::fprintf(stderr, "error: no epoch reaches min_quality=%d (read %zu epochs)\n",
                 static_cast<int>(opt.min_quality), recs.size());
    return 1;
  }

  std::ofstream ofs(out_path);
  if (!ofs) {
    std::fprintf(stderr, "error: cannot write %s\n", out_path.c_str());
    return 1;
  }
  gnss_core::write_enu_track(ofs, track);
  ofs.close();
  if (!ofs) {
    std::fprintf(stderr, "error: write failed on %s\n", out_path.c_str());
    return 1;
  }

  std::printf("read %zu epochs, wrote %zu samples (min_quality=%d)\n", recs.size(),
              track.samples.size(), static_cast<int>(opt.min_quality));
  std::printf("origin: lat=%.8f lon=%.8f alt=%.3f\n", track.origin_lla.x(), track.origin_lla.y(),
              track.origin_lla.z());
  return 0;
}
