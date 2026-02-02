#include "calibration_apriltag_detector.hpp"

#include "calibration/apriltag/apriltag_estimate_tag_pose.hpp"
#include "calibration/solvepnp/solvepnp_ippe_square.hpp"
#include "calibration_config.hpp"
#include "core/types.hpp"

#include <Eigen/src/Core/Map.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Geometry/Transform.h>
#include <apriltag.h>
#include <apriltag_pose.h>
#include <atomic>
#include <common/image_types.h>
#include <common/zarray.h>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include <tagStandard41h12.h>
#include <vector>

namespace calibration {
CalibrationApriltagDetector::CalibrationApriltagDetector(CalibrationConfig &calibrationCfg) : m_calibrationCfg(calibrationCfg), m_solvePnPPIPPESquare(calibrationCfg.tagSizeMeters), m_apriltagEstimateTagPose(calibrationCfg.tagSizeMeters) {
  m_td = apriltag_detector_create();
  apriltag_detector_add_family(m_td, tagStandard41h12_create());

  m_td->nthreads = 8;
  m_td->quad_decimate = 1.0;
}

CalibrationApriltagDetector::~CalibrationApriltagDetector() {
  if (m_td) {
    apriltag_detector_destroy(m_td);
  }
}

zarray_t *CalibrationApriltagDetector::DetectAprilTag(core::Frame *frameHybridBufferPeekSlot) const {
  if (!frameHybridBufferPeekSlot || frameHybridBufferPeekSlot->image.empty()) {
    return nullptr;
  }
  cv::Mat gray;
  cv::cvtColor(frameHybridBufferPeekSlot->image, gray, cv::COLOR_BGR2GRAY);
  image_u8_t img_header = {.width = gray.cols, .height = gray.rows, .stride = gray.cols, .buf = gray.data};
  zarray_t *detections = apriltag_detector_detect(m_td, &img_header);

  spdlog::info("Apriltag Detections: {}", zarray_size(detections));
  return detections;
}

double CalibrationApriltagDetector::GetTagSizeMeters() const {
  return m_calibrationCfg.tagSizeMeters;
}

std::shared_ptr<std::vector<core::TagData>> CalibrationApriltagDetector::GetEigenIsometry3fPosesAprilTag(core::Frame *frameHybridBufferPeekSlot) const {

  if (m_calibrationToggle) {
    return nullptr;
  }

  if (!frameHybridBufferPeekSlot) {
    return m_tagDataStorage.load(std::memory_order_acquire);
  }

  if (frameHybridBufferPeekSlot->image.empty()) {
    spdlog::info("Invalid frame peek");
    return m_tagDataStorage.load(std::memory_order_acquire);
  }

  spdlog::info("Solver: Apriltag", m_calibrationCfg.solverId);

  auto newTagDataList = std::make_shared<std::vector<core::TagData>>();

  zarray_t *detections = DetectAprilTag(frameHybridBufferPeekSlot);
  if (!detections) {
    return m_tagDataStorage.load(std::memory_order_acquire);
  }

  int detCount = zarray_size(detections);

  switch (m_calibrationCfg.solverId) {
  case 0: {
    auto poses = m_apriltagEstimateTagPose.estimateTagPose(detections, frameHybridBufferPeekSlot);
    if (!poses || poses->empty()) {
      spdlog::info("No Apriltag poses");
      apriltag_detections_destroy(detections);
      return m_tagDataStorage.load(std::memory_order_acquire);
    }

    for (int i = 0; i < detCount; i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);

      const auto &pose = poses->at(i);

      Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
      T.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(pose.R->data).cast<float>();
      T.translation() = Eigen::Map<const Eigen::Vector3d>(pose.t->data).cast<float>();

      newTagDataList->push_back({det->id, T});
    }

    break;
  }
  case 1: {
    auto [tvecs, rvecs] = m_solvePnPPIPPESquare.GetTvecsRvecs(detections, frameHybridBufferPeekSlot);

    if (!tvecs || !rvecs || tvecs->size() != rvecs->size() || tvecs->empty()) {
      spdlog::info("No solvePnP results");
      apriltag_detections_destroy(detections);
      return m_tagDataStorage.load(std::memory_order_acquire);
    }

    for (size_t i = 0; i < tvecs->size(); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, (int)i, &det);

      cv::Mat R;
      cv::Rodrigues(rvecs->at(i), R);

      Eigen::Matrix3f eigenR;
      cv::cv2eigen(R, eigenR);

      Eigen::Vector3f eigenT;
      cv::cv2eigen(tvecs->at(i), eigenT);

      Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
      T.linear() = eigenR;
      T.translation() = eigenT;

      newTagDataList->push_back({det->id, T});
    }
    break;
  }
  default:
    break;
  }

  apriltag_detections_destroy(detections);

  m_tagDataStorage.store(newTagDataList, std::memory_order_release);

  return m_tagDataStorage.load(std::memory_order_acquire);
}

void CalibrationApriltagDetector::ToggleCalibration() {
  m_calibrationToggle = !m_calibrationToggle;
}

} // namespace calibration