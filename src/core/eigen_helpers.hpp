#pragma once
#include <Eigen/src/Core/Map.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/Ref.h>
#include <Eigen/src/Core/util/Meta.h>
#include <Eigen/src/Geometry/Transform.h>
#include <nvAR_defs.h>
#include <vector>

namespace core {

inline std::vector<Eigen::Vector3f> ApplyPoseToEigen(const Eigen::Ref<const Eigen::Matrix3Xf> &keypoints, const Eigen::Isometry3f &pose) {
  if (keypoints.cols() == 0)
    return {};

  std::vector<Eigen::Vector3f> transformed(keypoints.cols());
  Eigen::Map<Eigen::Matrix3Xf>((float *)transformed.data(), 3, keypoints.cols()) = pose * keypoints;

  return transformed;
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