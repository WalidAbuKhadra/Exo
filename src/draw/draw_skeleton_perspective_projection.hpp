#pragma once
#include "engines/nvidia/nvar_config.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/Ref.h>
#include <vector>

namespace draw {
class DrawSkeletonPerspectiveProjection {
public:
  static void Draw3D(const Eigen::Ref<const Eigen::Matrix3Xf> &kpts3D, const std::vector<float> &confidence, const engines::nvidia::NvidiaConfig &nvConfig, const char *canvasName, double focalLength);

  static void Draw3D(const Eigen::Ref<const Eigen::Matrix3Xf> &kpts3D, const std::vector<float> &confidence, const engines::nvidia::NvidiaConfig &nvConfig, const char *canvasName, double focalLength, const Eigen::Ref<const Eigen::Matrix3Xf> &tagKpts3D);
};
} // namespace draw