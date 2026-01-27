#pragma once

#include "core/types.hpp"

#include <common/zarray.h>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <utility>
#include <vector>

namespace calibration::solvepnp {

class SolvePnPIPPESquare {
public:
  SolvePnPIPPESquare(double tagSize);
  ~SolvePnPIPPESquare();

  std::pair<std::shared_ptr<std::vector<cv::Mat>>, std::shared_ptr<std::vector<cv::Mat>>> GetTvecsRvecs(zarray_t *detections = nullptr, core::Frame *frameHybridBufferPeekSlot = nullptr) const;

private:
  mutable std::atomic<std::shared_ptr<std::vector<cv::Mat>>> m_tvecs;
  mutable std::atomic<std::shared_ptr<std::vector<cv::Mat>>> m_rvecs;

  double m_tagSize;
  std::vector<cv::Point3d> m_tagObjPoints;
};
} // namespace calibration::solvepnp