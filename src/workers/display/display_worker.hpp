#pragma once
#include "calibration/calibration_apriltag_detector.hpp"
#include "core/hybrid_buffer.hpp"
#include "core/types.hpp"
#include "engines/nvidia/nvar_config.hpp"
#include "engines/nvidia/nvar_pose.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <atomic>
#include <vector>

namespace workers::display {

class DisplayWorker {
public:
  DisplayWorker(core::HybridBuffer<core::Frame> &frameHybridBuffer, core::HybridBuffer<::engines::nvidia::NvARPose> &nvARPoseHybridBuffer, const engines::nvidia::NvidiaConfig &nvConfig, calibration::CalibrationApriltagDetector &calibrationApriltagDetector);

  void Run(std::atomic<bool> &running);

private:
  core::HybridBuffer<core::Frame> &m_frameHybridBuffer;
  core::HybridBuffer<engines::nvidia::NvARPose> &m_nvARPoseHybridBuffer;
  const engines::nvidia::NvidiaConfig &m_nvConfig;
  calibration::CalibrationApriltagDetector &m_calibrationApriltagDetector;

  core::Frame *m_frameHybridBufferPeekSlot = nullptr;
  engines::nvidia::NvARPose *m_nvARPoseHybridBufferPeekSlot = nullptr;
  bool m_attemptCalibrationAtNewPeek{false};
  float m_focalLength{800.79041f};

  std::vector<Eigen::Vector3f> m_tagObjPoints;
  double m_tagSize;
};

} // namespace workers::display