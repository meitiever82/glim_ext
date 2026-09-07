#pragma once
#include <gtsam/config.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Core>

namespace gnss_core {

// GTSAM 4.2 与 4.3 的 evaluateError 可选 Jacobian 形参类型不同:
//   4.3: gtsam::OptionalMatrixType (= gtsam::Matrix*)
//   4.2: boost::optional<gtsam::Matrix&>(由 NonlinearFactor.h 自身引入 boost,这里不直接 include boost)
// 两种类型都支持 `if (H) *H = ...`,函数体通用,只有签名按版本切换。
#if (GTSAM_VERSION_MAJOR > 4) || (GTSAM_VERSION_MAJOR == 4 && GTSAM_VERSION_MINOR >= 3)
using OptionalMatrixArg = gtsam::OptionalMatrixType;
#else
using OptionalMatrixArg = boost::optional<gtsam::Matrix&>;
#endif

// 带杆臂的 GNSS 位置先验因子(spec §7.3 v2)。
//   h(X) = X.transformFrom(body_point);  残差 = h(X) - measured_world
// body_point:天线在被约束 node 局部系中的位置。
//   rtk_global 在 submap 原点帧时刻取样,T_origin_frame = I,故 body_point = lever_imu;
//   rtk_odometry(轮 1.5)直接传 lever_imu。
// body_point = 0 时退化为纯平移先验(旋转不影响残差)。
class AntennaPriorFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
  gtsam::Point3 measured_;
  gtsam::Point3 body_point_;

public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

  AntennaPriorFactor(gtsam::Key key, const Eigen::Vector3d& measured_world,
                     const Eigen::Vector3d& body_point, const gtsam::SharedNoiseModel& model)
    : Base(model, key), measured_(measured_world), body_point_(body_point) {}

  // 工厂:返回基类 shared_ptr(4.2 为 boost::shared_ptr,4.3 为 std::shared_ptr,
  // 用 shared_ptr(new ...) 构造两版通用),可直接 graph.add()/push_back()。
  static gtsam::NonlinearFactor::shared_ptr create(gtsam::Key key, const Eigen::Vector3d& measured_world,
                                                   const Eigen::Vector3d& body_point,
                                                   const gtsam::SharedNoiseModel& model) {
    return gtsam::NonlinearFactor::shared_ptr(
        new AntennaPriorFactor(key, measured_world, body_point, model));
  }

  using Base::evaluateError;   // 保留基类的无 Jacobian 便捷重载 evaluateError(X)

  gtsam::Vector evaluateError(const gtsam::Pose3& X, OptionalMatrixArg H) const override {
    gtsam::Matrix36 Hpred;   // d(predicted)/d(pose),GTSAM 内建解析 Jacobian
    const gtsam::Point3 predicted = X.transformFrom(body_point_, H ? &Hpred : nullptr);
    if (H) *H = Hpred;
    return predicted - measured_;
  }

  const gtsam::Point3& measured() const { return measured_; }
  const gtsam::Point3& body_point() const { return body_point_; }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new AntennaPriorFactor(*this));
  }
};

}  // namespace gnss_core
