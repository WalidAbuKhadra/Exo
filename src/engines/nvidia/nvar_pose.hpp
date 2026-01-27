#pragma once

#include <cstdint>
#include <nvAR_defs.h>
#include <vector>

namespace engines::nvidia {

struct NvARPose {

  std::vector<NvAR_Point2f> keypoints;
  std::vector<NvAR_Point3f> keypoints3D;
  std::vector<NvAR_Quaternion> jointAngles;
  std::vector<float> keypointsConfidence;

  NvAR_TrackingBBoxes trackingBoundingBoxes{};
  std::vector<NvAR_TrackingBBox> trackingBoundingBox;

  NvAR_BBoxes boundingBoxes{};
  std::vector<NvAR_Rect> boundingBox;

  std::vector<float> boundingBoxConfidence;

  uint64_t frameID{0};
  double timestamp{0.0};

  NvARPose() = default;
  NvARPose(const NvARPose &) = delete;
  NvARPose &operator=(const NvARPose &) = delete;
};

} // namespace engines::nvidia