#include "output_worker.hpp"

#include "calibration/calibration_apriltag_detector.hpp"
#include "core/eigen_helpers.hpp"
#include "core/hybrid_buffer.hpp"
#include "engines/nvidia/nvar_pose.hpp"
#include "osc/osc_sender.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
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

    auto currentPoses = m_calibrationApriltagDetector.GetEigenIsometry3fPosesAprilTag();
    if (currentPoses && !currentPoses->empty()) {
      Eigen::Isometry3f inversePose = currentPoses->at(0).inverse();

      std::vector<Eigen::Vector3f> tagSpacePoints = ApplyPoseToEigen(rawKpts, inversePose);
      std::vector<Eigen::Vector3f> unityPoints(tagSpacePoints.size());

      for (size_t i = 0; i < tagSpacePoints.size(); i++) {

        unityPoints[i].x() = tagSpacePoints[i].x();
        unityPoints[i].y() = -tagSpacePoints[i].z();
        unityPoints[i].z() = tagSpacePoints[i].y();
      }
      m_oscSender.SendJoint(AsEigenMap(unityPoints));
    } else {
      m_oscSender.SendJoint(rawKpts);
    }
  }
}

} // namespace workers::output