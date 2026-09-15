// gnss_diag_node:把 gnss_core::DiagnosisEngine 接到车上(spec §8,轮 3b)。
// 订阅差分裸流、rtkrcv 独立解、610 融合解、rtkrcv $SAT 流;每秒 tick 一次,事件写
// <root>/YYYYMMDD/events.log,基站坐标史写同目录 base.pos,基线持久化到 <root>/base_baseline,
// 状态发布到 /gnss/diagnostics。判定逻辑全部在 gnss_core,这里只做参数、订阅、定时与落盘接线。
//
// 线程:只用 rclcpp::spin(单线程 executor),订阅回调、定时器、服务串行执行,引擎不加锁。
// 时间:引擎与落盘都用 node->now()(回放 bag 时配 use_sim_time),经 MonotonicClockGuard 保证单调;
//   回退超过 clock_jump_tolerance_s 时以 shutdown 关闭已开事件、重建引擎(基线从文件重读),
//   重新开始启动宽限期。use_sim_time 下还没收到 /clock(now()==0)时整拍跳过。
// 启动宽限期:前 startup_grace_s 秒只接收数据、不 tick,避免 rtkrcv 收敛前每次开机记一条 no_solution。
// 停机:spin 返回后 shutdown(now) → 写关闭行 → 关文件。
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <gnss_msgs/msg/raw_stream.hpp>
#include <gnss_msgs/msg/rtk_fix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "gnss_bringup/diag_node_support.hpp"
#include "gnss_bringup/rtk_fix_mapping.hpp"   // LineSplitter、is_positive_finite_seconds

namespace gnss_bringup {

class GnssDiagNode {
public:
  explicit GnssDiagNode(rclcpp::Node* node) : node_(node) {
    read_params();
    std::error_code ec;
    std::filesystem::create_directories(root_, ec);
    if (ec) throw std::invalid_argument("root 目录建不出来: " + root_ + ": " + ec.message());
    baseline_path_ = (std::filesystem::path(root_) / "base_baseline").string();
    events_ = std::make_unique<DayFileAppender>(root_, "events.log", gnss_core::events_log_header());
    base_history_ = std::make_unique<DayFileAppender>(root_, "base.pos", gnss_core::base_pos_header());
    build_engine();

    diag_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/gnss/diagnostics", 10);
    const auto qos = rclcpp::QoS(100).reliable();   // 与 rtcm_bridge / rtkrcv_node / 610 驱动的 reliable 发布者匹配
    corr_sub_ = node_->create_subscription<gnss_msgs::msg::RawStream>(
        corr_topic_, qos, [this](gnss_msgs::msg::RawStream::ConstSharedPtr m) { on_corrections(*m); });
    sol_sub_ = node_->create_subscription<gnss_msgs::msg::RtkFix>(
        sol_topic_, qos, [this](gnss_msgs::msg::RtkFix::ConstSharedPtr m) { on_solution(*m); });
    dev_sub_ = node_->create_subscription<gnss_msgs::msg::RtkFix>(
        dev_topic_, qos, [this](gnss_msgs::msg::RtkFix::ConstSharedPtr m) { on_device_solution(*m); });
    stat_sub_ = node_->create_subscription<gnss_msgs::msg::RawStream>(
        stat_topic_, qos, [this](gnss_msgs::msg::RawStream::ConstSharedPtr m) { on_stat(*m); });
    reset_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/reset_base_baseline",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> resp) { on_reset_base_baseline(*resp); });
    // 墙钟定时器:回放暂停(sim time 不走)时也照常检查;判定用的时间仍取 node->now()
    timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this] { on_tick(); });

    RCLCPP_INFO(node_->get_logger(),
                "gnss_diag 已启动: root=%s corr=%s sol=%s dev=%s stat=%s solver_enabled=%s 宽限期 %.0f s 控制点 %zu 个",
                root_.c_str(), corr_topic_.c_str(), sol_topic_.c_str(), dev_topic_.c_str(), stat_topic_.c_str(),
                solver_enabled_ ? "true" : "false", startup_grace_s_, control_points_.size());
  }

  void shutdown() {
    if (const auto last = clock_.last()) {
      write_transitions(engine_->shutdown(std::max(*last, node_->now().seconds())));
    }
    events_->close();
    base_history_->close();
  }

private:
  void read_params() {
    root_ = node_->declare_parameter<std::string>("root", "");
    if (root_.empty()) throw std::invalid_argument("root 不能为空");
    corr_topic_ = node_->declare_parameter<std::string>("corrections_topic", "/gnss/rtcm_corrections");
    sol_topic_ = node_->declare_parameter<std::string>("solution_topic", "/rtkrcv_node/rtk_fix");
    dev_topic_ = node_->declare_parameter<std::string>("device_topic", "/gnss_cgi610/rtk_fix");
    stat_topic_ = node_->declare_parameter<std::string>("stat_topic", "/rtkrcv_node/stat");
    solver_enabled_ = node_->declare_parameter<bool>("solver_enabled", true);

    startup_grace_s_ = node_->declare_parameter<double>("startup_grace_s", 60.0);
    if (!std::isfinite(startup_grace_s_) || startup_grace_s_ < 0.0) {
      throw std::invalid_argument("startup_grace_s 必须是 >= 0 的有限秒数");
    }
    const double tolerance_s = node_->declare_parameter<double>("clock_jump_tolerance_s", 1.0);
    if (!is_positive_finite_seconds(tolerance_s)) throw std::invalid_argument("clock_jump_tolerance_s 必须 > 0");
    clock_ = MonotonicClockGuard(tolerance_s);
    const double unpaired_warn_s = node_->declare_parameter<double>("unpaired_warn_s", 60.0);
    if (!std::isfinite(unpaired_warn_s) || unpaired_warn_s < 1.0 || unpaired_warn_s > 86400.0) {
      throw std::invalid_argument("unpaired_warn_s 必须在 [1, 86400]");
    }
    unpaired_ticks_ = static_cast<int>(unpaired_warn_s);   // 每秒一拍

    const auto names = node_->declare_parameter<std::vector<std::string>>("control_points.names", std::vector<std::string>{});
    const auto lat = node_->declare_parameter<std::vector<double>>("control_points.lat", std::vector<double>{});
    const auto lon = node_->declare_parameter<std::vector<double>>("control_points.lon", std::vector<double>{});
    control_points_ = control_points_from_params(names, lat, lon);

    cfg_ = read_diagnosis_config();
  }

  gnss_core::DiagnosisConfig read_diagnosis_config() {
    gnss_core::DiagnosisConfig c;
    const auto d = [this](const char* name, double& field) {
      field = node_->declare_parameter<double>(std::string("diagnosis.") + name, field);
    };
    const auto i = [this](const char* name, int& field) {
      // declare_parameter<int> 实际按 int64_t 取值(与 rtkrcv_node.cpp 同样的注意事项)
      const int64_t v = node_->declare_parameter<int>(std::string("diagnosis.") + name, field);
      if (v < INT_MIN || v > INT_MAX) throw std::invalid_argument(std::string(name) + ": 超出 int 范围");
      field = static_cast<int>(v);
    };
    d("corr_gap_s", c.corr_gap_s);
    d("age_max_s", c.age_max_s);
    d("base_shift_m", c.base_shift_m);
    i("min_sats", c.min_sats);
    d("resid_max_m", c.resid_max_m);
    d("low_el_deg", c.low_el_deg);
    d("low_snr_dbhz", c.low_snr_dbhz);
    d("min_ratio", c.min_ratio);
    i("slip_max_per_30s", c.slip_max_per_30s);
    d("divergence_sigma", c.divergence_sigma);
    d("divergence_hold_s", c.divergence_hold_s);
    d("close_hysteresis_s", c.close_hysteresis_s);
    d("sol_stale_s", c.sol_stale_s);
    d("abs_ref_max_m", c.abs_ref_max_m);
    d("abs_ref_radius_m", c.abs_ref_radius_m);
    d("divergence_window_s", c.divergence_window_s);
    i("divergence_min_samples", c.divergence_min_samples);
    d("divergence_sigma_floor_m", c.divergence_sigma_floor_m);
    d("divergence_sigma_max_m", c.divergence_sigma_max_m);
    d("divergence_pair_max_dt_s", c.divergence_pair_max_dt_s);
    d("divergence_epoch_max_dt_s", c.divergence_epoch_max_dt_s);
    d("base_warmup_s", c.base_warmup_s);
    gnss_core::validate_diagnosis_config(c);   // 非法时抛,消息以字段名开头
    return c;
  }

  // 新建(或回跳后重建)引擎:基线从文件读,base.pos 末行跨日期目录读(设计决定 6)
  void build_engine() {
    engine_ = std::make_unique<gnss_core::DiagnosisEngine>(cfg_, control_points_, solver_enabled_,
                                                           gnss_core::read_base_baseline(baseline_path_),
                                                           read_last_base_history(root_));
    grace_start_.reset();
    last_sol_t_.reset();
    last_dev_t_.reset();
    unpaired_ = UnpairedWatch(unpaired_ticks_);
    stat_splitter_.reset();
  }

  // 每次调用引擎前取时间。返回空:时间尚不可用。
  std::optional<double> engine_time() {
    const double now = node_->now().seconds();
    if (!(now > 0.0)) return std::nullopt;
    const auto prev = clock_.last();
    const auto step = clock_.step(now);
    if (step.jumped) {
      RCLCPP_WARN(node_->get_logger(),
                  "时钟回跳 %.3f s(%.3f → %.3f):以 shutdown 关闭已开事件,重建诊断引擎,重新开始 %.0f s 启动宽限期",
                  *prev - now, *prev, now, startup_grace_s_);
      write_transitions(engine_->shutdown(*prev));
      events_->close();
      base_history_->close();
      build_engine();
    }
    if (!grace_start_) grace_start_ = step.t;
    return step.t;
  }

  void on_corrections(const gnss_msgs::msg::RawStream& m) {
    const auto t = engine_time();
    if (!t) return;
    for (const auto& u : engine_->on_corrections(*t, m.data.data(), m.data.size())) handle_base_update(u);
  }

  void on_solution(const gnss_msgs::msg::RtkFix& m) {
    const auto t = engine_time();
    if (!t) return;
    engine_->on_solution(*t, to_solution_sample(m));
    last_sol_t_ = *t;
  }

  void on_device_solution(const gnss_msgs::msg::RtkFix& m) {
    const auto t = engine_time();
    if (!t) return;
    engine_->on_device_solution(*t, to_solution_sample(m));
    last_dev_t_ = *t;
  }

  // rtkrcv_node 按文件块转发 .stat,块边界不是行边界
  void on_stat(const gnss_msgs::msg::RawStream& m) {
    const auto t = engine_time();
    if (!t) return;
    for (const auto& line : stat_splitter_.feed(m.data.data(), m.data.size())) {
      if (line.rfind("$SAT", 0) == 0) engine_->on_stat_line(*t, line);
    }
  }

  void handle_base_update(const gnss_core::BaseUpdate& u) {
    if (u.feed.baseline_learned) {
      const auto b = engine_->baseline();
      if (b && gnss_core::write_base_baseline(baseline_path_, *b)) {
        RCLCPP_INFO(node_->get_logger(), "基站基线已持久化: %.4f %.4f %.4f -> %s", b->x, b->y, b->z,
                    baseline_path_.c_str());
      } else {
        RCLCPP_ERROR(node_->get_logger(), "基站基线写入失败: %s(重启后会重新预热)", baseline_path_.c_str());
      }
    }
    if (u.feed.history_changed &&
        !base_history_->append(u.t, gnss_core::format_base_history_line(u.t, gnss_core::Ecef{u.coords.x, u.coords.y, u.coords.z}))) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), steady_clock_, 10000, "base.pos 写入失败(root=%s)", root_.c_str());
    }
  }

  void write_transitions(const std::vector<gnss_core::EventTransition>& transitions) {
    for (const auto& e : transitions) {
      const std::string line = gnss_core::format_event_line(e);
      if (e.kind == gnss_core::EventKind::Open) {
        RCLCPP_WARN(node_->get_logger(), "诊断事件 %s", line.c_str());
      } else {
        RCLCPP_INFO(node_->get_logger(), "诊断事件 %s", line.c_str());
      }
      if (!events_->append(e.t, line)) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), steady_clock_, 10000, "events.log 写入失败(root=%s)", root_.c_str());
      }
    }
  }

  void on_tick() {
    const auto t = engine_time();
    if (!t) return;
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = node_->now();
    const double elapsed = *t - *grace_start_;
    if (elapsed < startup_grace_s_) {
      arr.status.push_back(make_startup_grace_status(startup_grace_s_ - elapsed));
      diag_pub_->publish(arr);
      return;
    }
    const auto r = engine_->tick(*t);
    write_transitions(r.transitions);
    arr.status.push_back(make_diagnostic_status(r, engine_->open_event_codes()));
    diag_pub_->publish(arr);

    const auto live = [&](const std::optional<double>& last) { return last && *t - *last < cfg_.sol_stale_s; };
    if (unpaired_.update(live(last_sol_t_) && live(last_dev_t_), r.divergence.divergence_m.has_value())) {
      RCLCPP_WARN(node_->get_logger(),
                  "独立解与 610 解都在到达,但已连续 %d s 没配上对,device_divergence 无法判定——"
                  "检查两路 gnss_time 是否都是 UTC(历元相差须 <= %.2f s)",
                  unpaired_ticks_, cfg_.divergence_epoch_max_dt_s);
    }
  }

  void on_reset_base_baseline(std_srvs::srv::Trigger::Response& resp) {
    const auto t = engine_time();
    if (!t) {
      resp.success = false;
      resp.message = "ROS 时间尚不可用(use_sim_time 下还没收到 /clock)";
      return;
    }
    const auto u = engine_->reset_base_baseline(*t);
    if (!u) {
      resp.success = false;
      resp.message = "还没收到过 RTCM 1005/1006,无法重置基站基线";
      return;
    }
    handle_base_update(*u);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "基站基线已重置为 %.4f %.4f %.4f", u->coords.x, u->coords.y, u->coords.z);
    resp.success = true;
    resp.message = buf;
    RCLCPP_WARN(node_->get_logger(), "运维重置: %s", buf);
  }

  rclcpp::Node* node_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};   // 日志节流用,不受 sim time 回跳影响

  std::string root_, baseline_path_;
  std::string corr_topic_, sol_topic_, dev_topic_, stat_topic_;
  bool solver_enabled_ = true;
  double startup_grace_s_ = 60.0;
  int unpaired_ticks_ = 60;
  gnss_core::DiagnosisConfig cfg_;
  std::vector<gnss_core::ControlPoint> control_points_;

  std::unique_ptr<gnss_core::DiagnosisEngine> engine_;
  std::unique_ptr<DayFileAppender> events_, base_history_;
  MonotonicClockGuard clock_{1.0};
  UnpairedWatch unpaired_{60};
  std::optional<double> grace_start_, last_sol_t_, last_dev_t_;
  LineSplitter stat_splitter_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<gnss_msgs::msg::RawStream>::SharedPtr corr_sub_, stat_sub_;
  rclcpp::Subscription<gnss_msgs::msg::RtkFix>::SharedPtr sol_sub_, dev_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace gnss_bringup

int main(int argc, char** argv) {
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<gnss_bringup::GnssDiagNode> diag;
  // 与 pos_writer_node.cpp 相同:rclcpp::init / 节点构造 / 参数声明的异常统一收口成一行错误 + 退出码 1
  try {
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("gnss_diag");
    diag = std::make_unique<gnss_bringup::GnssDiagNode>(node.get());
  } catch (const std::exception& e) {
    if (node) {
      RCLCPP_ERROR(node->get_logger(), "启动失败,配置有误: %s", e.what());
    } else {
      std::fprintf(stderr, "gnss_diag: 启动失败,配置有误: %s\n", e.what());
    }
    diag.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  diag->shutdown();   // node 仍存活:now()、日志都可用
  diag.reset();
  rclcpp::shutdown();
  return 0;
}
