#pragma once
#include "core/types.hpp"
#include "sources/webcam/webcam_config.hpp"

#include <cstdint>

namespace engines::nvidia {

struct NvidiaConfig {
  NvidiaConfig(const sources::webcam::WebcamConfig webcamConfig) : res({webcamConfig.res.width, webcamConfig.res.height}), sensorWidthMM(webcamConfig.sensorWidthMM), lensFocalLengthMM(webcamConfig.lensFocalLengthMM) {}
  core::Resolution res;
  float sensorWidthMM;
  float lensFocalLengthMM;

  char *modelDir = nullptr;
  uint32_t mode = 0;
  bool useCudaGraph = true;
  uint32_t temporal = 1;
  uint32_t fullBodyOnly = 0;
  bool postprocessJointAngle = true;
  uint32_t trackPeople = 0;
  uint32_t shadowTrackingAge = 90;
  uint32_t probationAge = 10;
  uint32_t maxTargetsTracked = 30;
};
} // namespace engines::nvidia