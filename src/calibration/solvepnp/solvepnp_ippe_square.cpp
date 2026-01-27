#include "solvepnp_ippe_square.hpp"

#include "core/types.hpp"

#include <apriltag.h>
#include <atomic>
#include <common/zarray.h>
#include <memory>
#include <nvAR_defs.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <utility>
#include <vector>

namespace calibration::solvepnp {

SolvePnPIPPESquare::SolvePnPIPPESquare(double tagSize) : m_tagSize(tagSize) {
  double s = m_tagSize / 2.0;
  m_tagObjPoints.push_back(cv::Point3d(-s, s, 0.0));
  m_tagObjPoints.push_back(cv::Point3d(s, s, 0.0));
  m_tagObjPoints.push_back(cv::Point3d(s, -s, 0.0));
  m_tagObjPoints.push_back(cv::Point3d(-s, -s, 0.0));
}

SolvePnPIPPESquare::~SolvePnPIPPESquare() {}

std::pair<std::shared_ptr<std::vector<cv::Mat>>, std::shared_ptr<std::vector<cv::Mat>>> SolvePnPIPPESquare::GetTvecsRvecs(zarray_t *detections, core::Frame *frameHybridBufferPeekSlot) const {

  if (!detections || zarray_size(detections) == 0) {
    return {m_tvecs.load(std::memory_order_acquire), m_rvecs.load(std::memory_order_acquire)};
  }

  if (!frameHybridBufferPeekSlot || frameHybridBufferPeekSlot->image.empty()) {
    return {m_tvecs.load(std::memory_order_acquire), m_rvecs.load(std::memory_order_acquire)};
  }

  auto newTvecslist = std::make_shared<std::vector<cv::Mat>>();
  auto newRvecslist = std::make_shared<std::vector<cv::Mat>>();

  auto new_tagsCornersCameraSpace = std::make_shared<std::vector<std::vector<NvAR_Point3f>>>();

  double width = static_cast<double>(frameHybridBufferPeekSlot->image.cols);
  double height = static_cast<double>(frameHybridBufferPeekSlot->image.rows);

  double fx = frameHybridBufferPeekSlot->lensFocalLengthMM * width / frameHybridBufferPeekSlot->sensorWidthMM;
  double fy = fx;
  double cx = width / 2.0;
  double cy = height / 2.0;

  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
  cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

  spdlog::info("[April] Detections: {} ", zarray_size(detections));
  for (int i = 0; i < zarray_size(detections); i++) {

    apriltag_detection_t *det;
    zarray_get(detections, i, &det);

    float confidence = det->decision_margin;
    int tagID = det->id;
    std::string family = det->family->name;

    spdlog::info("[April] Tag [{}]: Family={}, Confidence={:.2f}, Hamming={}", tagID, family, confidence, det->hamming);

    std::vector<cv::Point2d> imagePoints;
    imagePoints.push_back(cv::Point2d(det->p[3][0], det->p[3][1]));
    imagePoints.push_back(cv::Point2d(det->p[2][0], det->p[2][1]));
    imagePoints.push_back(cv::Point2d(det->p[1][0], det->p[1][1]));
    imagePoints.push_back(cv::Point2d(det->p[0][0], det->p[0][1]));

    cv::Mat rvec, tvec;
    cv::solvePnP(m_tagObjPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

    newTvecslist->push_back(tvec);
    newRvecslist->push_back(rvec);
  }

  m_tvecs.store(newTvecslist, std::memory_order_release);
  m_rvecs.store(newRvecslist, std::memory_order_release);

  return {m_tvecs.load(std::memory_order_acquire), m_rvecs.load(std::memory_order_acquire)};
};

} // namespace calibration::solvepnp