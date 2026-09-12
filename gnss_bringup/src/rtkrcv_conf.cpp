#include "gnss_bringup/rtkrcv_conf.hpp"

#include <cstdio>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace gnss_bringup {

namespace {

// Validate that a string field doesn't contain problematic characters
void validate_string_field(const std::string& value, const char* field_name) {
  if (value.find('\n') != std::string::npos) {
    throw std::invalid_argument(
        std::string(field_name) + " contains newline: " + value);
  }
  if (value.find('\r') != std::string::npos) {
    throw std::invalid_argument(
        std::string(field_name) + " contains carriage return: " + value);
  }
  if (value.find('=') != std::string::npos) {
    throw std::invalid_argument(
        std::string(field_name) + " contains equals sign: " + value);
  }
}

// Format a double without trailing zeros, avoiding scientific notation
// Uses locale-independent formatting (C locale decimal point)
std::string format_double_without_trailing_zeros(double value) {
  // Use %f with enough precision, then strip trailing zeros
  char buf[32];
  snprintf(buf, sizeof(buf), "%.6f", value);
  std::string str(buf);

  // Remove trailing zeros after decimal point
  size_t decimal_pos = str.find('.');
  if (decimal_pos != std::string::npos) {
    // Strip trailing zeros after decimal point
    size_t last = str.length() - 1;
    while (last > decimal_pos && str[last] == '0') {
      last--;
    }
    // Remove trailing decimal point if all fractional digits were zero
    if (last == decimal_pos) {
      str = str.substr(0, decimal_pos);
    } else {
      str = str.substr(0, last + 1);
    }
  }

  return str;
}

}  // namespace

std::string render_rtkrcv_conf(const RtkrcvConfParams& p) {
  // Validate all string fields
  validate_string_field(p.obs_format, "obs_format");
  validate_string_field(p.corr_format, "corr_format");
  validate_string_field(p.pos_mode, "pos_mode");
  validate_string_field(p.ar_mode, "ar_mode");

  // Validate elmask: must be in range [0, 90] and finite
  if (!std::isfinite(p.elmask)) {
    throw std::invalid_argument("elmask is not finite: " +
                                std::to_string(p.elmask));
  }
  if (p.elmask < 0.0 || p.elmask > 90.0) {
    throw std::invalid_argument("elmask out of range [0, 90]: " +
                                std::to_string(p.elmask));
  }

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
  oss << "pos1-elmask =" << format_double_without_trailing_zeros(p.elmask)
      << "\n";
  oss << "pos2-armode =" << p.ar_mode << "\n";
  oss << "pos1-navsys =" << p.navsys << "\n";

  return oss.str();
}

}  // namespace gnss_bringup
