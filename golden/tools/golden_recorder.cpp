/**
 * @file golden_recorder.cpp
 * @brief Golden 用例录制器：链接 loc_parser_core（线上旧代码），按用例时间轴
 *        喂协议帧，录制处理器逐帧输出为 golden.yaml。
 *
 * 用法: golden_recorder <case.yaml> <golden_out.yaml>
 *
 * 关键设计：
 * - 帧由结构化字段合成为协议字节（含 CRC32/NMEA 校验和），用例可读可评审
 * - 虚拟 ITransport 按真实时间轴喂帧（旧实现写死 steady_clock，无法注入时间，
 *   30s 粘滞用例录制时真实等待）
 * - 录制器只记录，不修正：输出即线上行为
 */
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "localization_parser/core/common/config.hpp"
#include "localization_parser/core/processor/beiyun_processor.hpp"
#include "localization_parser/core/processor/huace_processor.hpp"
#include "localization_parser/core/processor/localization_data.hpp"

namespace lp = localization_parser::core;

// ---------------------------------------------------------------------------
// 协议帧合成
// ---------------------------------------------------------------------------

/** NovAtel CRC32（多项式 0xEDB88320，与 frame_extractor 的校验对应） */
static uint32_t NovAtelCrc32(const std::string& s) {
    uint32_t crc = 0;
    for (unsigned char c : s) {
        crc ^= c;
        for (int i = 0; i < 8; ++i)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
    }
    return crc;
}

static std::string NmeaChecksum(const std::string& body) {
    uint8_t cs = 0;
    for (char c : body) cs ^= static_cast<uint8_t>(c);
    char buf[4];
    std::snprintf(buf, sizeof(buf), "%02X", cs);
    return buf;
}

static std::string SynthesizeInspvaxa(const YAML::Node& f) {
    // header: INSPVAXA,COM1,0,66.0,FINESTEERING,week,seconds,...;
    // data: ins_status,pos_type,lat,lon,alt,undulation,nv,ev,uv,roll,pitch,yaw,
    //       lat_std,lon_std,hgt_std,nv_std,ev_std,uv_std,roll_std,pitch_std,az_std,...
    auto get = [&](const char* k, const char* dflt) {
        return f[k] ? f[k].as<std::string>() : std::string(dflt);
    };
    char body[512];
    std::snprintf(body, sizeof(body),
        "INSPVAXA,COM1,0,66.0,FINESTEERING,%s,%s,00000000,0000,785;"
        "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,"
        "%s,%s,%s,%s,%s,%s,%s,%s,%s,00000000,0000",
        get("gps_week", "2400").c_str(), get("gps_seconds", "100000.0").c_str(),
        get("ins_status", "INS_SOLUTION_GOOD").c_str(),
        get("pos_type", "INS_RTKFIXED").c_str(),
        get("lat", "44.50323451").c_str(), get("lon", "92.91759315").c_str(),
        get("alt", "753.8173").c_str(), get("undulation", "0.0").c_str(),
        get("north_vel", "0.0").c_str(), get("east_vel", "0.0").c_str(),
        get("up_vel", "0.0").c_str(),
        get("roll", "0.0").c_str(), get("pitch", "0.0").c_str(),
        get("yaw", "90.0").c_str(),
        get("lat_std", "0.01").c_str(), get("lon_std", "0.01").c_str(),
        get("hgt_std", "0.02").c_str(), get("nv_std", "0.01").c_str(),
        get("ev_std", "0.01").c_str(), get("uv_std", "0.02").c_str(),
        get("roll_std", "0.1").c_str(), get("pitch_std", "0.1").c_str(),
        get("az_std", "0.1").c_str());
    char out[600];
    std::snprintf(out, sizeof(out), "#%s*%08X\r\n", body, NovAtelCrc32(body));
    return out;
}

static std::string SynthesizeGga(const YAML::Node& f, bool beiyun) {
    (void)beiyun;
    auto get = [&](const char* k, const char* dflt) {
        return f[k] ? f[k].as<std::string>() : std::string(dflt);
    };
    // $GPGGA,hhmmss,lat,N,lon,E,quality,sats,hdop,alt,M,und,M,age,stn*cs
    char body[256];
    std::snprintf(body, sizeof(body), "GPGGA,000000,%s,%s,%s,%s,%s,%s,0.8,%s,M,0.0,M,%s,0001",
        get("lat", "").c_str(), get("lat_ns", "N").c_str(),
        get("lon", "").c_str(), get("lon_ew", "E").c_str(),
        get("quality", "4").c_str(), get("sats", "18").c_str(),
        get("alt", "753.8").c_str(), get("age", "1.0").c_str());
    return "$" + std::string(body) + "*" + NmeaChecksum(body) + "\r\n";
}

static std::string SynthesizeHeadinga(const YAML::Node& f) {
    auto get = [&](const char* k, const char* dflt) {
        return f[k] ? f[k].as<std::string>() : std::string(dflt);
    };
    // beiyun_parser.cpp: df[1]=heading_type, df[2]=length, df[3]=heading,
    // df[10]=solnsvs, df[11]=obs；df[0] 为数据段首字段
    char body[256];
    std::snprintf(body, sizeof(body),
        "HEADINGA,COM1,0,66.0,FINESTEERING,%s,%s,00000000,0000,785;"
        "0.0,%s,%s,%s,0.0,0.0,0.0,0.0,0.0,0.0,%s,%s,0,0,0,0",
        get("gps_week", "2400").c_str(), get("gps_seconds", "100000.0").c_str(),
        get("heading_type", "NARROW_INT").c_str(),
        get("length", "1.276").c_str(),
        get("heading", "0.0").c_str(),
        get("solnsvs", "18").c_str(), get("obs", "18").c_str());
    char out[400];
    std::snprintf(out, sizeof(out), "#%s*%08X\r\n", body, NovAtelCrc32(body));
    return out;
}

static std::string SynthesizeGpfpd(const YAML::Node& f) {
    auto get = [&](const char* k, const char* dflt) {
        return f[k] ? f[k].as<std::string>() : std::string(dflt);
    };
    // nmea_parser.hpp: nsv1 = fields[13]（去 $ 后按逗号分）
    // $GPFPD,week,seconds,hdg,pitch,roll,lat,lon,alt,ve,vn,vu,baseline,nsv1,nsv2,sig
    char body[256];
    std::snprintf(body, sizeof(body),
        "GPFPD,%s,%s,0.00,0.00,0.00,44.50323451,92.91759315,753.82,"
        "0.00,0.00,0.00,0.00,%s,0,00",
        get("gps_week", "2400").c_str(), get("gps_seconds", "100000.0").c_str(),
        get("nsv1", "18").c_str());
    return "$" + std::string(body) + "*" + NmeaChecksum(body) + "\r\n";
}

static std::string SynthesizeGpchc(const YAML::Node& f) {
    auto get = [&](const char* k, const char* dflt) {
        return f[k] ? f[k].as<std::string>() : std::string(dflt);
    };
    // huace_parser.cpp: [1]=week [2]=sec [3]=heading [4]=pitch [5]=roll
    // [6-8]=gyro [9-11]=acc [12]=lat [13]=lon [14]=alt [15]=ve [16]=vn [17]=vu
    // [18]=speed [19]=nsv1 [20]=nsv2 [21]=sta(十六进制!) [22]=age [23]=warning(十六进制)
    char body[512];
    std::snprintf(body, sizeof(body),
        "GPCHC,%s,%s,%s,%s,%s,0.0,0.0,0.0,0.0,0.0,0.0,"
        "%s,%s,%s,0.0,0.0,0.0,%s,%s,%s,%s,%s,%s",
        get("gps_week", "2400").c_str(), get("gps_seconds", "100000.0").c_str(),
        get("heading", "0.0").c_str(), get("pitch", "0.0").c_str(),
        get("roll", "0.0").c_str(),
        get("lat", "44.50323451").c_str(), get("lon", "92.91759315").c_str(),
        get("alt", "753.8173").c_str(),
        get("speed", "0.0").c_str(), get("solnsvs", "18").c_str(),
        get("solnsvs2", "18").c_str(),
        get("sta", "42").c_str(),           // 十六进制字符串，"42"=0x42
        get("age", "1").c_str(),            // 十进制整数（ParseInt）
        get("warning", "0000").c_str());    // 十六进制字符串
    return "$" + std::string(body) + "*" + NmeaChecksum(body) + "\r\n";
}

// ---------------------------------------------------------------------------
// 虚拟传输：按时间轴喂帧
// ---------------------------------------------------------------------------
class ScriptedTransport : public lp::transport::ITransport {
public:
    void Push(double t, std::string bytes) { queue_.emplace(t, std::move(bytes)); }

    bool Open() override { return true; }
    void Close() override {}
    bool IsOpen() override { return true; }
    bool IsClientConnected() const override { return true; }

    std::size_t Read(uint8_t* buf, std::size_t len) override {
        if (queue_.empty()) return 0;
        auto& [t, bytes] = queue_.front();
        // 时间轴由调用方（录制循环）控制：只有到点的帧才入 pending_
        if (pending_.empty() && t > Now()) return 0;
        if (pending_.empty()) { pending_ = std::move(bytes); queue_.pop(); }
        std::size_t n = std::min(len, pending_.size());
        std::memcpy(buf, pending_.data(), n);
        pending_.erase(0, n);
        return n;
    }

    static double Now() { return now_s_; }
    static void SetNow(double t) { now_s_ = t; }
    bool Empty() const { return queue_.empty() && pending_.empty(); }

private:
    std::queue<std::pair<double, std::string>> queue_;
    std::string pending_;
    static inline double now_s_ = 0.0;
};

// ---------------------------------------------------------------------------
// 子类暴露 protected 接口（不修改线上代码）
// ---------------------------------------------------------------------------
class BeiYunProbe : public lp::processor::BeiYunProcessor {
public:
    using lp::processor::BeiYunProcessor::BeiYunProcessor;
    using lp::processor::ProcessorBase::ProcessCycle;
    using lp::processor::ProcessorBase::GetLocalizationData;
    using lp::processor::ProcessorBase::GetAuxData;
};
class HuaCeProbe : public lp::processor::HuaCeProcessor {
public:
    using lp::processor::HuaCeProcessor::HuaCeProcessor;
    using lp::processor::ProcessorBase::ProcessCycle;
    using lp::processor::ProcessorBase::GetLocalizationData;
    using lp::processor::ProcessorBase::GetAuxData;
};

// ---------------------------------------------------------------------------
// 输出快照
// ---------------------------------------------------------------------------
static YAML::Node Snapshot(const lp::processor::LocalizationData& d) {
    YAML::Node n;
    n["is_valid"]   = d.is_valid;
    n["rtk_status"] = static_cast<int>(d.rtk_status);
    n["fault_code"] = static_cast<int>(d.fault_code);
    n["latitude"]  = d.latitude;
    n["longitude"] = d.longitude;
    n["altitude"]  = d.altitude;
    n["x"] = d.x; n["y"] = d.y; n["z"] = d.z;
    n["yaw"] = d.yaw; n["pitch"] = d.pitch; n["roll"] = d.roll;
    n["speed"] = d.speed;
    n["solnsvs"] = d.solnsvs; n["solnsvs2"] = d.solnsvs2;
    n["age"] = d.age;
    return n;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: golden_recorder <case.yaml> <golden_out.yaml>\n");
        return 2;
    }
    YAML::Node kase = YAML::LoadFile(argv[1]);
    const std::string brand = kase["brand"].as<std::string>();

    // 配置快照 → Config
    lp::common::Config cfg;
    if (auto c = kase["config"]) {
        if (c["origin_lat"]) cfg.origin_lat = c["origin_lat"].as<double>();
        if (c["origin_lon"]) cfg.origin_lon = c["origin_lon"].as<double>();
        if (c["origin_height"]) cfg.origin_height = c["origin_height"].as<double>();
        if (c["xyz_threshold"]) cfg.xyz_threshold = c["xyz_threshold"].as<int>();
        if (c["stars_threshold"]) cfg.stars_threshold = c["stars_threshold"].as<int>();
        if (c["unfixed_time"]) cfg.unfixed_time = c["unfixed_time"].as<double>();
        if (c["combined_nav_mod_time"]) cfg.combined_nav_mod_time = c["combined_nav_mod_time"].as<double>();
        if (c["rtk_float_grace_time"]) cfg.rtk_float_grace_time = c["rtk_float_grace_time"].as<double>();
        if (c["sticky_grace_time"]) cfg.sticky_grace_time = c["sticky_grace_time"].as<double>();
        if (c["single_point_age_threshold"]) cfg.single_point_age_threshold = c["single_point_age_threshold"].as<double>();
        if (c["heading_length"]) cfg.heading_length = c["heading_length"].as<double>();
        if (c["heading_length_threshold"]) cfg.heading_length_threshold = c["heading_length_threshold"].as<double>();
        if (c["data_timeout_sec"]) cfg.data_timeout_sec = c["data_timeout_sec"].as<double>();
        if (c["parse_fail_threshold"]) cfg.parse_fail_threshold = c["parse_fail_threshold"].as<int>();
    }

    ScriptedTransport transport;
    // 合成帧入队
    double t_max = 0.0;
    for (auto fr : kase["frames"]) {
        double t = fr["t"].as<double>();
        std::string type = fr["type"].as<std::string>();
        YAML::Node fields = fr["fields"] ? fr["fields"] : YAML::Node();
        std::string bytes;
        if (type == "inspvaxa") bytes = SynthesizeInspvaxa(fields);
        else if (type == "gpgga") bytes = SynthesizeGga(fields, brand == "beiyun");
        else if (type == "headinga") bytes = SynthesizeHeadinga(fields);
        else if (type == "gpfpd") bytes = SynthesizeGpfpd(fields);
        else if (type == "gpchc") bytes = SynthesizeGpchc(fields);
        else { std::fprintf(stderr, "unknown frame type: %s\n", type.c_str()); return 2; }
        transport.Push(t, std::move(bytes));
        t_max = std::max(t_max, t);
    }
    // 用例可选：尾帧后追加观察窗（粘滞超时用例需要）
    if (kase["observe_until"]) t_max = kase["observe_until"].as<double>();

    std::unique_ptr<BeiYunProbe> by;
    std::unique_ptr<HuaCeProbe> hc;
    if (brand == "beiyun") by = std::make_unique<BeiYunProbe>(transport, cfg);
    else if (brand == "huace") hc = std::make_unique<HuaCeProbe>(transport, cfg);
    else { std::fprintf(stderr, "unknown brand: %s\n", brand.c_str()); return 2; }
    // protected 接口经由探针子类调用
    auto cycle = [&]() { return by ? by->ProcessCycle() : hc->ProcessCycle(); };
    auto get_data = [&]() { return by ? by->GetLocalizationData() : hc->GetLocalizationData(); };

    // 真实时间播放（旧代码用 steady_clock/system_clock，时间轴必须真实流逝）
    // 帧配对：记录每个输出快照时"已按时间轴喂入的最后一帧"的序号
    std::vector<double> frame_times;
    for (auto fr : kase["frames"]) frame_times.push_back(fr["t"].as<double>());
    std::sort(frame_times.begin(), frame_times.end());

    YAML::Node outputs(YAML::NodeType::Sequence);
    const auto wall0 = std::chrono::steady_clock::now();
    while (true) {
        double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
        ScriptedTransport::SetNow(t);
        bool produced = cycle();
        if (produced) {
            auto d = get_data();
            if (d) {
                YAML::Node snap = Snapshot(*d);
                snap["t"] = t;
                int idx = -1;
                for (size_t i = 0; i < frame_times.size(); ++i)
                    if (frame_times[i] <= t) idx = static_cast<int>(i);
                snap["frame"] = idx;
                outputs.push_back(snap);
            }
        }
        if (transport.Empty() && t > t_max) break;
        if (t > t_max + 5.0) break;  // 兜底
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    YAML::Node golden;
    golden["case"] = kase["case"].as<std::string>();
    golden["class"] = kase["class"].as<std::string>();
    YAML::Node prov;
    prov["recorder_version"] = 1;
    prov["note"] = "source_commit/config_sha256 由录制脚本填充";
    golden["provenance"] = prov;
    golden["outputs"] = outputs;

    std::ofstream fout(argv[2]);
    fout << golden;
    std::printf("recorded %zu output snapshots -> %s\n", outputs.size(), argv[2]);
    return 0;
}
