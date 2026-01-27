#pragma once
#include "core/types.hpp"
#include "sources/isource.hpp"
#include "webcam_config.hpp"

#include <cstdint>
#include <opencv2/videoio.hpp>
#include <string_view>

namespace sources::webcam {

class WebcamSource : public IFrameSource {
public:
  explicit WebcamSource(const WebcamConfig &config);

  ~WebcamSource() override;

  WebcamSource(const WebcamSource &) = delete;
  WebcamSource &operator=(const WebcamSource &) = delete;

  bool Start() override;

  void Stop() override;

  bool CaptureInto(core::Frame *target) override;

  std::string_view Name() const override { return "Webcam"; }

private:
  int m_deviceIndex;
  core::Resolution m_res;
  int m_fps;

  float m_sensorWidthMM;
  float m_lensFocalLengthMM;

  cv::VideoCapture m_cap;

  uint64_t m_frameCount{0};
};

} // namespace sources::webcam