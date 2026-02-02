#pragma once

#include "calibration/apriltag/apriltag_estimate_tag_pose.hpp"
#include "calibration/solvepnp/solvepnp_ippe_square.hpp"
#include "calibration_config.hpp"
#include "core/types.hpp"

#include <apriltag.h>
#include <common/zarray.h>
#include <memory>
#include <vector>

namespace calibration {

class CalibrationApriltagDetector {

public:
  CalibrationApriltagDetector(CalibrationConfig &calibrationCfg);
  ~CalibrationApriltagDetector();

  std::shared_ptr<std::vector<core::TagData>> GetEigenIsometry3fPosesAprilTag(core::Frame *frameHybridBufferPeekSlot = nullptr) const;
  void ToggleCalibration();
  double GetTagSizeMeters() const;

private:
  CalibrationConfig &m_calibrationCfg;
  calibration::solvepnp::SolvePnPIPPESquare m_solvePnPPIPPESquare;
  calibration::apriltag::ApriltagEstimateTagPose m_apriltagEstimateTagPose;

  apriltag_detector_t *m_td = nullptr;
  mutable std::atomic<std::shared_ptr<std::vector<core::TagData>>> m_tagDataStorage;
  bool m_calibrationToggle{true};

  zarray_t *DetectAprilTag(core::Frame *frameHybridBufferPeekSlot = nullptr) const;
};

} // namespace calibration