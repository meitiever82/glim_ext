#include "gnss_bringup/executable_lookup.hpp"

#include <sys/stat.h>
#include <unistd.h>

#include <filesystem>

namespace gnss_bringup {

namespace {
bool is_executable_file(const std::string& p) {
  struct stat st {};
  return ::stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode) && ::access(p.c_str(), X_OK) == 0;
}
}  // namespace

std::string resolve_executable(const std::string& name, const char* path_env) {
  if (name.empty()) return {};
  if (name.find('/') != std::string::npos) {
    if (!is_executable_file(name)) return {};
    std::error_code ec;
    const auto abs = std::filesystem::absolute(name, ec);
    return ec ? std::string{} : abs.string();
  }
  if (path_env == nullptr) return {};
  const std::string path(path_env);
  std::size_t begin = 0;
  while (begin <= path.size()) {
    std::size_t end = path.find(':', begin);
    if (end == std::string::npos) end = path.size();
    if (end > begin) {
      std::string cand = path.substr(begin, end - begin);
      if (cand.back() != '/') cand += '/';
      cand += name;
      if (is_executable_file(cand)) return cand;
    }
    begin = end + 1;
  }
  return {};
}

}  // namespace gnss_bringup
