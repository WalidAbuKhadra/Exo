#pragma once
#include "calibration/calibration_apriltag_detector.hpp"
#include "core/hybrid_buffer.hpp"
#include "engines/nvidia/nvar_pose.hpp"
#include "osc/osc_sender.hpp"

#include <atomic>

namespace workers::output {

class OutputWorker {
public:
  OutputWorker(core::HybridBuffer<engines::nvidia::NvARPose> &nvARPoseHybridBuffer, calibration::CalibrationApriltagDetector &calibrationApriltagDetector, osc::OscSender &oscSender);

  void Run(std::atomic<bool> &running);

private:
  core::HybridBuffer<engines::nvidia::NvARPose> &m_nvARPoseHybridBuffer;
  calibration::CalibrationApriltagDetector &m_calibrationApriltagDetector;
  osc::OscSender &m_oscSender;

  engines::nvidia::NvARPose *m_nvARPoseHybridBufferReadSlot = nullptr;
};

} // namespace workers::output