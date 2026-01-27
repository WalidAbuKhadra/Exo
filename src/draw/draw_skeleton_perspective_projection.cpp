#include "draw_skeleton_perspective_projection.hpp"

#include "skeleton.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/Ref.h>
#include <Eigen/src/Core/util/Meta.h>
#include <engines/nvidia/nvar_config.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace draw {

void DrawSkeletonPerspectiveProjection::Draw3DInversed(const Eigen::Ref<const Eigen::Matrix3Xf> &kpts3D, const std::vector<float> &confidence, const engines::nvidia::NvidiaConfig &nvConfig, const char *canvasName, double focalLength) {
  Eigen::Matrix3Xf emptyMatrix(3, 0);
  Draw3DInversed(kpts3D, confidence, nvConfig, canvasName, focalLength, emptyMatrix);
}

void DrawSkeletonPerspectiveProjection::Draw3DInversed(const Eigen::Ref<const Eigen::Matrix3Xf> &kpts3D, const std::vector<float> &confidence, const engines::nvidia::NvidiaConfig &nvConfig, const char *canvasName, double focalLength, const Eigen::Ref<const Eigen::Matrix3Xf> &tagKpts3D) {

  if (kpts3D.cols() == 0) {
    return;
  }

  int w = nvConfig.res.width;
  int h = nvConfig.res.height;

  cv::Mat canvas3D = cv::Mat::zeros(h, w, CV_8UC3);
  float cx = w / 2.0f;
  float cy = h / 2.0f;

  const float CONF_THRESH = 0.3f;

  auto projectInversed = [&](const Eigen::Vector3f &p) {
    return cv::Point2f(cx + (p.x() * focalLength / p.z()),
                       cy + (p.y() * focalLength / p.z()));
  };

  for (const auto &bone : BONES) {
    if (confidence[bone.first] < CONF_THRESH || confidence[bone.second] < CONF_THRESH)
      continue;

    auto p1 = kpts3D.col(bone.first);
    auto p2 = kpts3D.col(bone.second);

    if (p1.z() == 0 || p2.z() == 0)
      continue;

    cv::line(canvas3D, projectInversed(p1), projectInversed(p2), GetColor(bone.second), 2, cv::LINE_AA);
  }

  for (Eigen::Index i = 0; i < kpts3D.cols(); ++i) {
    if (confidence[i] < CONF_THRESH)
      continue;

    auto p = kpts3D.col(i);
    if (p.y() == 0)
      continue;

    cv::circle(canvas3D, projectInversed(p), 4, GetColor(static_cast<int>(i)), -1);
  }

  bool hasTags = tagKpts3D.cols() > 0;

  if (hasTags) {
    if (tagKpts3D.cols() > 0) {
      for (Eigen::Index i = 0; i < tagKpts3D.cols(); ++i) {
        auto p = tagKpts3D.col(i);
        if (p.z() == 0)
          continue;
        cv::circle(canvas3D, projectInversed(p), 2, cv::Scalar(0, 255, 255), 2);
      }
    }
  }

  cv::imshow(canvasName, canvas3D);
}

} // namespace draw