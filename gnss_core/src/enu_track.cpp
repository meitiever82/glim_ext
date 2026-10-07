#include "gnss_core/enu_track.hpp"

#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>

#include "gnss_core/geodetic.hpp"
#include "gnss_core/pos_io.hpp"

namespace gnss_core {

EnuTrack make_enu_track(const std::vector<PosRecord>& recs, const EnuTrackOptions& opt) {
  EnuTrack track;

  // 原点:显式给定优先;否则取第一条达到 min_quality 的记录。两者都没有 → 空轨迹。
  if (opt.has_origin) {
    track.origin_lla = opt.origin_lla;
  } else {
    bool found = false;
    for (const auto& r : recs) {
      if (q_to_quality(r.q) >= opt.min_quality) {
        track.origin_lla = Eigen::Vector3d(r.lat, r.lon, r.height);
        found = true;
        break;
      }
    }
    if (!found) {
      return track;
    }
  }

  const LlaToEnu conv(track.origin_lla.x(), track.origin_lla.y(), track.origin_lla.z());
  track.samples.reserve(recs.size());
  for (const auto& r : recs) {
    const Quality q = q_to_quality(r.q);
    if (q < opt.min_quality) {
      continue;
    }
    EnuSample s;
    s.stamp = r.stamp;
    s.enu = conv.forward(r.lat, r.lon, r.height);
    // PosRecord::sdne 的列序是 (sdn, sde, sdu);输出按 ENU 换序
    s.sigma_enu = Eigen::Vector3d(r.sdne.y(), r.sdne.x(), r.sdne.z());
    s.quality = q;
    track.samples.push_back(s);
  }
  return track;
}

void write_enu_track(std::ostream& out, const EnuTrack& track) {
  char buf[256];
  std::snprintf(buf, sizeof(buf), "# origin_lla %.8f %.8f %.3f\n", track.origin_lla.x(),
                track.origin_lla.y(), track.origin_lla.z());
  out << buf;
  out << "# t e n u sde sdn sdu q\n";
  for (const auto& s : track.samples) {
    std::snprintf(buf, sizeof(buf), "%.3f %.3f %.3f %.3f %.4f %.4f %.4f %d\n", s.stamp, s.enu.x(),
                  s.enu.y(), s.enu.z(), s.sigma_enu.x(), s.sigma_enu.y(), s.sigma_enu.z(),
                  static_cast<int>(s.quality));
    out << buf;
  }
}

EnuTrack read_enu_track(std::istream& in) {
  EnuTrack track;
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();   // CRLF 容错(与 report_inputs 一致)
    }
    if (line.empty()) {
      continue;
    }
    if (line[0] == '#') {
      std::istringstream hs(line.substr(1));
      std::string key;
      double a = 0.0, b = 0.0, c = 0.0;
      if ((hs >> key >> a >> b >> c) && key == "origin_lla") {
        track.origin_lla = Eigen::Vector3d(a, b, c);
      }
      continue;
    }
    std::istringstream ls(line);
    double t = 0.0, e = 0.0, n = 0.0, u = 0.0, sde = 0.0, sdn = 0.0, sdu = 0.0;
    int q = 0;
    if (!(ls >> t >> e >> n >> u >> sde >> sdn >> sdu >> q)) {
      continue;   // 列数不足或非数值:跳过,不中断
    }
    if (q < 0 || q > static_cast<int>(Quality::FIXED)) {
      continue;
    }
    EnuSample s;
    s.stamp = t;
    s.enu = Eigen::Vector3d(e, n, u);
    s.sigma_enu = Eigen::Vector3d(sde, sdn, sdu);
    s.quality = static_cast<Quality>(q);
    track.samples.push_back(s);
  }
  return track;
}

}  // namespace gnss_core
