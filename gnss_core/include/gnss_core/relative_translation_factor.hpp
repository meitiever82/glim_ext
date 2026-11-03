#pragma once
// Stage-2 RTK relative-translation factor (spec 2.1 §5.4).
//
// Binary factor on X(t1), X(t2) that constrains only the horizontal translation difference:
//
//   r = (t2 - t1)_xy - R_ENU_odom * dp_ENU_xy
//
// The measured displacement is already rotated into the world frame by the caller (using the
// independently bootstrapped yaw); rotation of the poses is left entirely to IMU/VGICP, and
// the vertical component is ignored because RTK height differences are noisy.

#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "gnss_core/factor_compat.hpp"

namespace gnss_core {

class RelativeTranslationFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3> {
  Eigen::Vector2d measured_world_;   // R_ENU_odom * dp_ENU_xy

public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>;

  RelativeTranslationFactor(gtsam::Key key1, gtsam::Key key2, const Eigen::Vector2d& measured_world_xy,
                            const gtsam::SharedNoiseModel& model)
    : Base(model, key1, key2), measured_world_(measured_world_xy) {}

  static gtsam::NonlinearFactor::shared_ptr create(gtsam::Key key1, gtsam::Key key2,
                                                   const Eigen::Vector2d& measured_world_xy,
                                                   const gtsam::SharedNoiseModel& model) {
    return gtsam::NonlinearFactor::shared_ptr(
        new RelativeTranslationFactor(key1, key2, measured_world_xy, model));
  }

  using Base::evaluateError;

  gtsam::Vector evaluateError(const gtsam::Pose3& X1, const gtsam::Pose3& X2,
                              OptionalMatrixArg H1, OptionalMatrixArg H2) const override {
    const Eigen::Vector2d predicted = (X2.translation() - X1.translation()).head<2>();
    // Pose3's tangent is (omega, v) and retract perturbs the translation in the local
    // frame (t <- t + R v), so d(t)/dv = R (columns 3..5). The measurement is in the
    // world frame, hence R rather than a plain ±I; X1 and X2 may have different yaw.
    if (H1) {
      gtsam::Matrix J = gtsam::Matrix::Zero(2, 6);
      J.block<2, 3>(0, 3) = -X1.rotation().matrix().topRows<2>();
      *H1 = J;
    }
    if (H2) {
      gtsam::Matrix J = gtsam::Matrix::Zero(2, 6);
      J.block<2, 3>(0, 3) = X2.rotation().matrix().topRows<2>();
      *H2 = J;
    }
    return predicted - measured_world_;
  }

  const Eigen::Vector2d& measured() const { return measured_world_; }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new RelativeTranslationFactor(*this));
  }
};

}  // namespace gnss_core
