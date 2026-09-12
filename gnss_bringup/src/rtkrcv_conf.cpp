#include "gnss_bringup/rtkrcv_conf.hpp"

#include <cstdio>
#include <sstream>

namespace gnss_bringup {

std::string render_rtkrcv_conf(const RtkrcvConfParams& p) {
  std::ostringstream oss;

  // Input stream 1 (observations)
  oss << "inpstr1-type =" << "tcpcli" << "\n";
  oss << "inpstr1-path =127.0.0.1:" << p.obs_port << "\n";
  oss << "inpstr1-format =" << p.obs_format << "\n";

  // Input stream 2 (corrections)
  oss << "inpstr2-type =" << "tcpcli" << "\n";
  oss << "inpstr2-path =127.0.0.1:" << p.corr_port << "\n";
  oss << "inpstr2-format =" << p.corr_format << "\n";

  // Output stream 1 (solution)
  oss << "outstr1-type =" << "tcpsvr" << "\n";
  oss << "outstr1-path =:" << p.sol_port << "\n";
  oss << "outstr1-format =" << "llh" << "\n";

  // Solution format options
  oss << "out-solformat =" << "llh" << "\n";
  oss << "out-outhead =" << "off" << "\n";
  oss << "out-timesys =" << "gpst" << "\n";

  // Positioning options
  oss << "pos1-posmode =" << p.pos_mode << "\n";

  // Format elmask without trailing zeros
  char elmask_buf[32];
  snprintf(elmask_buf, sizeof(elmask_buf), "%g", p.elmask);
  oss << "pos1-elmask =" << elmask_buf << "\n";

  oss << "pos2-armode =" << p.ar_mode << "\n";
  oss << "pos1-navsys =" << p.navsys << "\n";

  return oss.str();
}

}  // namespace gnss_bringup
