#pragma once
// Stage-2 RTK absolute-heading factor (spec 2.1 §5.4).
//
// Unary factor on a pose that aligns its yaw with the RTK dual-antenna heading. The shell
// converts the board heading (deg, north-zero clockwise) into a world-frame yaw using the
// frozen T_world_enu yaw offset before constructing the factor:
//
//   yaw_world = yaw(T_world_enu) + (pi/2 - heading_rad)
//
// Only constructed when heading_valid == true, so low-speed drift of the dual-antenna
// heading is gated out and attitude stays on IMU/VGICP.

#include <cmath>

#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "gnss_core/factor_compat.hpp"

namespace gnss_core {

class HeadingPriorFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
  double measured_yaw_;

  static double wrap(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

  HeadingPriorFactor(gtsam::Key key, double measured_yaw, const gtsam::SharedNoiseModel& model)
    : Base(model, key), measured_yaw_(wrap(measured_yaw)) {}

  static gtsam::NonlinearFactor::shared_ptr create(gtsam::Key key, double measured_yaw,
                                                   const gtsam::SharedNoiseModel& model) {
    return gtsam::NonlinearFactor::shared_ptr(new HeadingPriorFactor(key, measured_yaw, model));
  }

  using Base::evaluateError;

  gtsam::Vector evaluateError(const gtsam::Pose3& X, OptionalMatrixArg H) const override {
    const double e = wrap(X.rotation().yaw() - measured_yaw_);
    if (H) {
      // d(yaw)/d(tangent): the translation block is zero. With a right perturbation
      // R <- R Exp(d), yaw changes only through the rotation block, and the body-z axis
      // is not aligned with world-z when roll/pitch is non-zero, so use a central finite
      // difference instead of the flat-ground approximation H = [0,0,0,0,0,1].
      gtsam::Matrix J = gtsam::Matrix::Zero(1, 6);
      const double h = 1e-6;
      for (int j = 0; j < 6; ++j) {
        gtsam::Vector6 d = gtsam::Vector6::Zero();
        d(j) = h;
        const double ep = wrap(X.retract(d).rotation().yaw() - measured_yaw_);
        const double em = wrap(X.retract(-d).rotation().yaw() - measured_yaw_);
        J(0, j) = (ep - em) / (2.0 * h);
      }
      *H = J;
    }
    return gtsam::Vector1(e);
  }

  double measured_yaw() const { return measured_yaw_; }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new HeadingPriorFactor(*this));
  }
};

}  // namespace gnss_core
