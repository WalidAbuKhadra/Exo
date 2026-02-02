#pragma once
#include <Eigen/src/Core/Map.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/Ref.h>
#include <Eigen/src/Core/util/Meta.h>
#include <Eigen/src/Geometry/Transform.h>
#include <nvAR_defs.h>
#include <vector>

namespace core {

inline void ApplyPoseToEigen(const Eigen::Ref<const Eigen::Matrix3Xf> &inputPoints, const Eigen::Isometry3f &pose, std::vector<Eigen::Vector3f> &outputBuffer) {
  if (inputPoints.cols() == 0)
    return;

  if (outputBuffer.size() != inputPoints.cols()) {
    outputBuffer.resize(inputPoints.cols());
  }

  Eigen::Map<Eigen::Matrix3Xf> outputMap((float *)outputBuffer.data(), 3, inputPoints.cols());

  outputMap = pose * inputPoints;
}

inline Eigen::Map<const Eigen::Matrix3Xf> AsEigenMap(const std::vector<NvAR_Point3f> &points) {
  if (points.empty()) {
    return {nullptr, 3, 0};
  }
  return {&points[0].x, 3, (Eigen::Index)points.size()};
}

inline Eigen::Map<const Eigen::Matrix3Xf> AsEigenMap(const std::vector<Eigen::Vector3f> &points) {
  {
    if (points.empty())
      return {nullptr, 3, 0};
  }
  return {(const float *)points.data(), 3, (Eigen::Index)points.size()};
}

}; // namespace core