#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include "gnss_bringup/executable_lookup.hpp"
using gnss_bringup::resolve_executable;

namespace {
class ExecutableLookup : public ::testing::Test {
protected:
  void SetUp() override {
    const char* base = std::getenv("TMPDIR");
    std::string tmpl = std::string(base ? base : "/tmp") + "/exe_lookup_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    ASSERT_NE(::mkdtemp(buf.data()), nullptr);
    dir_ = buf.data();
    std::filesystem::create_directories(dir_ + "/a");
    std::filesystem::create_directories(dir_ + "/b");
  }
  void TearDown() override { std::filesystem::remove_all(dir_); }

  std::string make_file(const std::string& rel, mode_t mode) {
    const std::string p = dir_ + "/" + rel;
    std::ofstream(p) << "#!/bin/sh\n";
    ::chmod(p.c_str(), mode);
    return p;
  }
  std::string dir_;
};
}  // namespace

TEST_F(ExecutableLookup, FindsTheFirstExecutableMatchInPathOrder) {
  make_file("a/tool", 0755);
  make_file("b/tool", 0755);
  const std::string path = dir_ + "/a:" + dir_ + "/b";
  EXPECT_EQ(resolve_executable("tool", path.c_str()), dir_ + "/a/tool");
}

TEST_F(ExecutableLookup, SkipsNonExecutableFilesAndDirectories) {
  make_file("a/tool", 0644);                               // 不可执行
  std::filesystem::create_directories(dir_ + "/b/tool");   // 同名目录
  const std::string path = dir_ + "/a:" + dir_ + "/b";
  EXPECT_EQ(resolve_executable("tool", path.c_str()), "");
}

TEST_F(ExecutableLookup, NameWithSlashIsCheckedDirectlyNotSearched) {
  const std::string exe = make_file("a/tool", 0755);
  EXPECT_EQ(resolve_executable(exe, "/nonexistent"), exe);
  EXPECT_EQ(resolve_executable(dir_ + "/b/tool", (dir_ + "/a").c_str()), "")
      << "带斜杠的名字不能再去 PATH 里找同名文件";
}

TEST_F(ExecutableLookup, RelativeNameWithSlashIsReturnedAsAbsolutePath) {
  // 子进程在 exec 之前会 chdir(run_dir),相对路径必须在父进程里就转成绝对路径
  make_file("a/tool", 0755);
  const auto old = std::filesystem::current_path();
  std::filesystem::current_path(dir_);
  const std::string got = resolve_executable("./a/tool", nullptr);
  std::filesystem::current_path(old);
  ASSERT_FALSE(got.empty());
  EXPECT_EQ(got.front(), '/');
  EXPECT_TRUE(std::filesystem::equivalent(got, dir_ + "/a/tool"));
}

TEST_F(ExecutableLookup, EmptyPathSegmentsDoNotMeanCurrentDirectory) {
  make_file("tool", 0755);
  const auto old = std::filesystem::current_path();
  std::filesystem::current_path(dir_);
  const std::string got = resolve_executable("tool", ":/nonexistent:");
  std::filesystem::current_path(old);
  EXPECT_EQ(got, "") << "空段在 POSIX 里表示当前目录;子进程的 cwd 是 run_dir,不能被当成查找目录";
}

TEST(ExecutableLookupInputs, NullOrEmptyInputsResolveToNothing) {
  EXPECT_EQ(resolve_executable("", "/usr/bin"), "");
  EXPECT_EQ(resolve_executable("sh", nullptr), "");
  EXPECT_EQ(resolve_executable("sh", ""), "");
}
