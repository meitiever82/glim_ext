#pragma once
// 测试共用的注入式 streambuf(Task 1,round 2 hardening)——test_pos_io.cpp
// 和 test_synth.cpp 都需要它来验证 read_pos(std::istream&) /
// read_glim_traj(std::istream&) 在真实 I/O 错误下的行为,原来在两个文件里
// 各自复制一份,评审要求收敛成一份共用实现。
#include <algorithm>
#include <cstddef>
#include <ios>
#include <streambuf>
#include <string>
#include <utility>

namespace gnss_core::test_fixtures {

// 前 fail_after 个字节正常供给(每次最多 16 字节,确保失败发生在扫描中途),
// 之后 underflow() 抛 std::ios_base::failure——这正是 basic_filebuf 在真实
// EIO 时的行为。std::getline 的 sentry 会吞掉这个异常、只置 badbit,不会
// 重新抛出;被测函数必须自己检查 badbit。不经过任何代码内注入点。
class FailingStreambuf : public std::streambuf {
public:
  FailingStreambuf(std::string data, std::size_t fail_after)
      : data_(std::move(data)), fail_after_(std::min(fail_after, data_.size())) {}

protected:
  int_type underflow() override {
    if (pos_ >= fail_after_) throw std::ios_base::failure("injected EIO");
    const std::size_t n = std::min<std::size_t>(sizeof(buf_), fail_after_ - pos_);
    std::copy(data_.data() + pos_, data_.data() + pos_ + n, buf_);
    pos_ += n;
    setg(buf_, buf_, buf_ + n);
    return traits_type::to_int_type(buf_[0]);
  }

private:
  std::string data_;
  std::size_t fail_after_;
  std::size_t pos_ = 0;
  char buf_[16];
};

}  // namespace gnss_core::test_fixtures
