#include "apriltag_estimate_tag_pose.hpp"

#include "core/types.hpp"

#include <apriltag.h>
#include <apriltag_pose.h>
#include <atomic>
#include <common/zarray.h>
#include <memory>
#include <vector>

namespace calibration::apriltag {

ApriltagEstimateTagPose::ApriltagEstimateTagPose(double tagSize) : m_tagSize(tagSize) {}

ApriltagEstimateTagPose::~ApriltagEstimateTagPose() {}

std::shared_ptr<std::vector<apriltag_pose_t>> ApriltagEstimateTagPose::estimateTagPose(zarray_t *detections, core::Frame *frameHybridBufferPeekSlot) const {

  if (!detections || zarray_size(detections) == 0) {
    return {m_poses.load(std::memory_order_acquire)};
  }

  if (!frameHybridBufferPeekSlot || frameHybridBufferPeekSlot->image.empty()) {
    return {m_poses.load(std::memory_order_acquire)};
  }

  double width = static_cast<double>(frameHybridBufferPeekSlot->image.cols);
  double height = static_cast<double>(frameHybridBufferPeekSlot->image.rows);

  auto newPoselist = std::make_shared<std::vector<apriltag_pose_t>>();

  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);

    apriltag_detection_info_t info{};
    info.det = det;
    info.tagsize = m_tagSize;
    info.fx = frameHybridBufferPeekSlot->lensFocalLengthMM * width / frameHybridBufferPeekSlot->sensorWidthMM;
    info.fy = frameHybridBufferPeekSlot->lensFocalLengthMM * width / frameHybridBufferPeekSlot->sensorWidthMM;
    info.cx = width / 2.0;
    info.cy = height / 2.0;

    apriltag_pose_t pose;
    estimate_tag_pose(&info, &pose);

    newPoselist->push_back(pose);
  }

  m_poses.store(newPoselist, std::memory_order_release);

  return {m_poses.load(std::memory_order_acquire)};
}
} // namespace calibration::apriltag