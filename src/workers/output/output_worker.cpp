#include "output_worker.hpp"

#include "calibration/calibration_apriltag_detector.hpp"
#include "core/eigen_helpers.hpp"
#include "core/hybrid_buffer.hpp"
#include "engines/nvidia/nvar_pose.hpp"
#include "osc/osc_sender.hpp"

#include <Eigen/src/Core/Map.h>
#include <Eigen/src/Core/Matrix.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

using namespace calibration;
using namespace osc;
using namespace core;

namespace workers::output {

OutputWorker::OutputWorker(core::HybridBuffer<engines::nvidia::NvARPose> &nvARPoseHybridBuffer, CalibrationApriltagDetector &calibrationApriltagDetector, OscSender &oscSender) : m_nvARPoseHybridBuffer(nvARPoseHybridBuffer), m_calibrationApriltagDetector(calibrationApriltagDetector), m_oscSender(oscSender) {
}

void OutputWorker::Run(std::atomic<bool> &running) {

  while (running) {
    m_nvARPoseHybridBufferReadSlot = m_nvARPoseHybridBuffer.Fetch();

    if (!m_nvARPoseHybridBufferReadSlot) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    auto rawKpts = AsEigenMap(m_nvARPoseHybridBufferReadSlot->keypoints3D) * 0.001f;

    const auto &nvRot = m_nvARPoseHybridBufferReadSlot->jointAngles;
    Eigen::Map<const Eigen::Matrix4Xf> rotMap((float *)nvRot.data(), 4, nvRot.size());

    m_oscSender.SendSkeleton(rawKpts, rotMap, m_nvARPoseHybridBufferReadSlot->keypointsConfidence);

    auto currentTags = m_calibrationApriltagDetector.GetEigenIsometry3fPosesAprilTag();
    if (currentTags && !currentTags->empty()) {
      m_oscSender.SendTags(*currentTags);
    }
  }
}

} // namespace workers::output