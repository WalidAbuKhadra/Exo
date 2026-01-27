#pragma once

#include "core/types.hpp"

#include <apriltag_pose.h>
#include <common/zarray.h>
#include <memory>
#include <vector>

namespace calibration::apriltag {

class ApriltagEstimateTagPose {
public:
  ApriltagEstimateTagPose(double tagSize);
  ~ApriltagEstimateTagPose();

  std::shared_ptr<std::vector<apriltag_pose_t>> estimateTagPose(zarray_t *detections = nullptr, core::Frame *frameHybridBufferPeekSlot = nullptr) const;

private:
  double m_tagSize;
  mutable std::atomic<std::shared_ptr<std::vector<apriltag_pose_t>>> m_poses;
};
} // namespace calibration::apriltag