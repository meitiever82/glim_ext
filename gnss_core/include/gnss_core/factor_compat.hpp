// GTSAM 4.2 / 4.3 compatibility for the optional-Jacobian argument of
// NoiseModelFactorN::evaluateError().
//
//   4.3: gtsam::OptionalMatrixType          (= gtsam::Matrix*)
//   4.2: boost::optional<gtsam::Matrix&>    (boost brought in by NonlinearFactor.h)
//
// Both support `if (H) *H = ...`, so the factor bodies are shared.

#pragma once

#include <gtsam/config.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gnss_core {

#if (GTSAM_VERSION_MAJOR > 4) || (GTSAM_VERSION_MAJOR == 4 && GTSAM_VERSION_MINOR >= 3)
using OptionalMatrixArg = gtsam::OptionalMatrixType;
#else
using OptionalMatrixArg = boost::optional<gtsam::Matrix&>;
#endif

}  // namespace gnss_core
