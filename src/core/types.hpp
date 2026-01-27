#pragma once
#include <chrono>
#include <cstdint>
#include <opencv2/core/mat.hpp>

namespace core {

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

struct Resolution {
  int width;
  int height;
};

struct Frame {
  cv::Mat image;
  TimePoint timestamp;
  uint64_t frameID = 0;

  float sensorWidthMM = 4.8f;
  float lensFocalLengthMM = 3.67f;
};

} // namespace core
