#include "display_worker.hpp"

#include "calibration/calibration_apriltag_detector.hpp"
#include "core/eigen_helpers.hpp"
#include "core/hybrid_buffer.hpp"
#include "core/types.hpp"
#include "draw/draw_skeleton_perspective_projection.hpp"
#include "engines/nvidia/nvar_config.hpp"
#include "engines/nvidia/nvar_pose.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <opencv2/highgui.hpp>
#include <thread>
#include <vector>

using namespace core;

namespace workers::display {

DisplayWorker::DisplayWorker(core::HybridBuffer<core::Frame> &frameHybridBuffer, core::HybridBuffer<::engines::nvidia::NvARPose> &nvARPoseHybridBuffer, const engines::nvidia::NvidiaConfig &nvConfig, calibration::CalibrationApriltagDetector &calibrationApriltagDetector) : m_frameHybridBuffer(frameHybridBuffer), m_nvARPoseHybridBuffer(nvARPoseHybridBuffer), m_nvConfig(nvConfig), m_calibrationApriltagDetector(calibrationApriltagDetector), m_tagSize(calibrationApriltagDetector.GetTagSizeMeters()) {
  double s = m_tagSize / 2.0;

  m_tagObjPoints.reserve(4);
  m_tagObjPoints.emplace_back(-s, s, 0);
  m_tagObjPoints.emplace_back(s, s, 0);
  m_tagObjPoints.emplace_back(s, -s, 0);
  m_tagObjPoints.emplace_back(-s, -s, 0);

  m_focalLength = nvConfig.lensFocalLengthMM * nvConfig.res.width / nvConfig.sensorWidthMM;
}

void DisplayWorker::Run(std::atomic<bool> &running) {
  cv::namedWindow("Exo - Camera Feed", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Exo - 3D Preview", cv::WINDOW_AUTOSIZE);

  m_frameHybridBuffer.RequestNewPeek();
  m_nvARPoseHybridBuffer.RequestNewPeek();

  while (running) {

    m_frameHybridBufferPeekSlot = m_frameHybridBuffer.GetPeekIfNew();
    m_nvARPoseHybridBufferPeekSlot = m_nvARPoseHybridBuffer.GetPeekIfNew();

    if (!m_frameHybridBufferPeekSlot && !m_nvARPoseHybridBufferPeekSlot) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (m_frameHybridBufferPeekSlot && !m_frameHybridBufferPeekSlot->image.empty()) {
      cv::imshow("Exo - Camera Feed", m_frameHybridBufferPeekSlot->image);
    }

    if (m_nvARPoseHybridBufferPeekSlot) {

      auto poses = m_calibrationApriltagDetector.GetEigenIsometry3fPosesAprilTag();

      const auto &confidence = m_nvARPoseHybridBufferPeekSlot->keypointsConfidence;

      if (poses && !poses->empty()) {
        draw::DrawSkeletonPerspectiveProjection::Draw3DInversed(AsEigenMap(m_nvARPoseHybridBufferPeekSlot->keypoints3D) * 0.001f, confidence, m_nvConfig, "Exo - 3D Preview", m_focalLength, AsEigenMap(ApplyPoseToEigen(AsEigenMap(m_tagObjPoints), poses->at(0))));
      } else {
        draw::DrawSkeletonPerspectiveProjection::Draw3DInversed(AsEigenMap(m_nvARPoseHybridBufferPeekSlot->keypoints3D) * 0.001f, confidence, m_nvConfig, "Exo - 3D Preview", m_focalLength);
      }
    }

    int key = cv::waitKey(1);
    if (key == 27) {
      running = false;
    } else if (key == 'c' || key == 'C') {
      m_attemptCalibrationAtNewPeek = true;
    } else if (key == 'X' || key == 'x') {
      m_calibrationApriltagDetector.ToggleCalibration();
    }

    if (m_attemptCalibrationAtNewPeek && m_frameHybridBufferPeekSlot && !m_frameHybridBufferPeekSlot->image.empty()) {
      m_calibrationApriltagDetector.GetEigenIsometry3fPosesAprilTag(m_frameHybridBufferPeekSlot);
      m_attemptCalibrationAtNewPeek = false;
    }

    if (m_frameHybridBufferPeekSlot) {
      m_frameHybridBuffer.RequestNewPeek();
    }
    if (m_nvARPoseHybridBufferPeekSlot) {
      m_nvARPoseHybridBuffer.RequestNewPeek();
    }
  }

  cv::destroyAllWindows();
}

} // namespace workers::display