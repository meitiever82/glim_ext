/**
 * @file golden_replay.cpp
 * @brief Golden parity 回放器：用例 + golden 输入，驱动 gnss_core::judge（时间注入），
 *        逐帧对比输出。keep 类断言与 golden 一致；changed 类断言与 expect_new 一致。
 *
 * 用法: golden_replay <case.yaml> <golden.yaml>
 * 退出码: 0=parity 通过, 1=存在分歧
 *
 * 配对与容差规则见 golden/README.md §4。
 */
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "gnss_core/judge.hpp"

namespace j = gnss_core::judge;

// --- case fields -> Evidence（对齐录制器的合成与 processor 的消费）----------
static j::PosType MapBeiyunPosType(const std::string& s) {
    if (s == "INS_PSRSP")    return j::PosType::Single;
    if (s == "INS_PSRDIFF")  return j::PosType::Psrdiff;
    if (s == "INS_RTKFLOAT") return j::PosType::RtkFloat;
    if (s == "INS_RTKFIXED") return j::PosType::RtkFixed;
    return j::PosType::None;
}

struct ReplayResult {
    int compared = 0;
    int mismatch = 0;
    std::vector<std::string> diffs;
};

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: golden_replay <case.yaml> <golden.yaml>\n");
        return 2;
    }
    YAML::Node kase = YAML::LoadFile(argv[1]);
    YAML::Node golden = YAML::LoadFile(argv[2]);
    const std::string brand = kase["brand"].as<std::string>();
    const std::string klass = kase["class"].as<std::string>();

    j::Config cfg;
    if (auto c = kase["config"]) {
        if (c["xyz_threshold"]) cfg.xyz_threshold = c["xyz_threshold"].as<int>();
        if (c["stars_threshold"]) cfg.stars_threshold = c["stars_threshold"].as<int>();
        if (c["sticky_grace_time"]) cfg.sticky_grace_time = c["sticky_grace_time"].as<double>();
        if (c["single_point_age_threshold"]) cfg.single_point_age_threshold = c["single_point_age_threshold"].as<double>();
        if (c["heading_length"]) cfg.heading_length = c["heading_length"].as<double>();
        if (c["heading_length_threshold"]) cfg.heading_length_threshold = c["heading_length_threshold"].as<double>();
        if (c["unfixed_time"]) cfg.unfixed_time = c["unfixed_time"].as<double>();
        if (c["combined_nav_mod_time"]) cfg.combined_nav_mod_time = c["combined_nav_mod_time"].as<double>();
        if (c["rtk_float_grace_time"]) cfg.rtk_float_grace_time = c["rtk_float_grace_time"].as<double>();
    }

    // golden 的主帧输出（按序）——既是对比基准，也是 ENU 输入来源
    // （ENU 越界门控的输入是旧代码坐标换算结果，golden 里录制的就是它；
    //   坐标换算单元有独立测试，回放器不复刻）
    std::vector<YAML::Node> golden_main;
    for (auto o : golden["outputs"]) golden_main.push_back(o);

    double gga_age = 0.0;
    uint16_t gpfpd_nsv1 = 0;
    bool has_heading = false, heading_narrow = false;
    double heading_len = 0.0;
    uint16_t heading_obs = 0;
    j::GateState state;
    ReplayResult res;
    size_t mi = 0;  // 主帧序号

    auto check = [&](const j::Verdict& v, const YAML::Node& exp, double t, const std::string& src) {
        ++res.compared;
        auto diff_field = [&](const char* name, bool a, bool b) {
            if (a != b) {
                res.mismatch++;
                res.diffs.push_back(std::string("t=") + std::to_string(t) + " " + name +
                    ": actual=" + (a ? "true" : "false") + " expect=" + (b ? "true" : "false") + " [" + src + "]");
            }
        };
        auto diff_int = [&](const char* name, int a, int b) {
            if (a != b) {
                res.mismatch++;
                res.diffs.push_back(std::string("t=") + std::to_string(t) + " " + name +
                    ": actual=" + std::to_string(a) + " expect=" + std::to_string(b) + " [" + src + "]");
            }
        };
        diff_field("is_valid", v.is_valid, exp["is_valid"].as<bool>());
        diff_int("rtk_status", static_cast<int>(v.rtk_status), exp["rtk_status"].as<int>());
        diff_int("fault_code", static_cast<int>(v.fault_code), exp["fault_code"].as<int>());
    };

    // changed 类：断言 expect_new；keep 类：断言 golden
    YAML::Node expect_new = kase["expect_new"];

    for (auto fr : kase["frames"]) {
        const double t = fr["t"].as<double>();
        const std::string type = fr["type"].as<std::string>();
        const YAML::Node f = fr["fields"] ? fr["fields"] : YAML::Node();
        auto gs = [&](const char* k, const char* d) { return f[k] ? f[k].as<std::string>() : std::string(d); };

        if (type == "gpgga") { gga_age = std::stod(gs("age", "0")); continue; }
        if (type == "gpfpd") { gpfpd_nsv1 = static_cast<uint16_t>(std::stoi(gs("nsv1", "0"))); continue; }
        if (type == "headinga") {
            has_heading = true;
            heading_narrow = (gs("heading_type", "") == "NARROW_INT");
            heading_len = std::stod(gs("length", "0"));
            heading_obs = static_cast<uint16_t>(std::stoi(gs("obs", "0")));
            continue;
        }
        if (type != "inspvaxa" && type != "gpchc") continue;
        if (mi >= golden_main.size()) { res.mismatch++; res.diffs.push_back("主帧数超过 golden"); break; }

        j::Evidence e;
        e.now = t;
        // ENU 取自 golden 该主帧的输出（旧代码坐标换算结果，作为门控输入）
        e.enu_x = golden_main[mi]["x"].as<double>();
        e.enu_y = golden_main[mi]["y"].as<double>();
        e.enu_z = golden_main[mi]["z"].as<double>();

        j::Verdict v;
        if (type == "inspvaxa") {
            e.pos_type = MapBeiyunPosType(gs("pos_type", ""));
            e.ins_good = (gs("ins_status", "") == "INS_SOLUTION_GOOD");
            e.diff_age = gga_age;
            e.sats_main = gpfpd_nsv1;
            e.sats_aux = heading_obs;
            e.heading_narrow_int = heading_narrow;
            e.heading_length = heading_len;
            e.heading_valid_frame = has_heading;
            v = j::judge_beiyun(e, state, cfg);
        } else {
            const auto sta = static_cast<uint8_t>(std::stoul(gs("sta", "0"), nullptr, 16));
            const auto warning = static_cast<uint16_t>(std::stoul(gs("warning", "0"), nullptr, 16));
            e.huace_gnss_pos_type = (sta & 0xF0) >> 4;
            e.ins_good = (sta & 0x0F) == 2;
            e.hw_warning = (warning & 0x1FD) != 0;
            e.huace_gnss_pos_valid = (warning & 0x01) == 0;
            e.sats_main = static_cast<uint16_t>(std::stoi(gs("solnsvs", "0")));
            e.sats_aux = static_cast<uint16_t>(std::stoi(gs("solnsvs2", "0")));
            v = j::judge_huace(e, state, cfg);
        }

        if (klass == "keep") {
            check(v, golden_main[mi], t, "golden");
        } else if (klass == "changed" && expect_new) {
            // expect_new 按 at_frame（帧序号）匹配
            bool asserted = false;
            for (auto en : expect_new) {
                if (en["at_frame"].as<int>() == static_cast<int>(mi)) {
                    check(v, en, t, "expect_new");
                    asserted = true;
                }
            }
            if (!asserted) check(v, golden_main[mi], t, "golden(unchanged-part)");
        }
        ++mi;
    }

    std::printf("case=%s class=%s compared=%d mismatch=%d\n",
        kase["case"].as<std::string>().c_str(), klass.c_str(), res.compared, res.mismatch);
    for (auto& d : res.diffs) std::printf("  DIFF %s\n", d.c_str());
    return res.mismatch == 0 ? 0 : 1;
}
