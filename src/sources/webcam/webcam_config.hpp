#pragma once
#include "core/types.hpp"

namespace sources::webcam {

struct WebcamConfig {
  int deviceIndex{0};
  core::Resolution res = {1920, 1080};
  int fps{30};

  float sensorWidthMM = 4.8f;
  float lensFocalLengthMM = 3.67f;
};
} // namespace sources::webcam