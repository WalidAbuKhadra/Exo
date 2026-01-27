#include "webcam_source.hpp"

#include "core/types.hpp"
#include "webcam_config.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

namespace sources::webcam {

WebcamSource::WebcamSource(const WebcamConfig &config) : m_deviceIndex(config.deviceIndex), m_res(config.res), m_fps(config.fps), m_sensorWidthMM(config.sensorWidthMM), m_lensFocalLengthMM(config.lensFocalLengthMM) {}

WebcamSource::~WebcamSource() {
  if (m_cap.isOpened()) {
    m_cap.release();
  }
}

bool WebcamSource::Start() {
  if (m_cap.isOpened())
    return true;

  m_cap.open(m_deviceIndex, cv::CAP_ANY);

  if (!m_cap.isOpened()) {
    return false;
  }

  m_cap.set(cv::CAP_PROP_FRAME_WIDTH, m_res.width);
  m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, m_res.height);
  m_cap.set(cv::CAP_PROP_FPS, m_fps);

  cv::Mat tmp;
  if (!m_cap.read(tmp) || tmp.empty()) {
    Stop();
    return false;
  }

  return true;
}

void WebcamSource::Stop() {
  if (m_cap.isOpened()) {
    m_cap.release();
  }
}

bool WebcamSource::CaptureInto(core::Frame *target) {
  if (!m_cap.isOpened() || !target)
    return false;

  if (!m_cap.read(target->image))
    return false;

  if (target->image.empty()) {
    return false;
  }

  target->timestamp = core::Clock::now();
  target->frameID = m_frameCount++;
  target->lensFocalLengthMM = m_lensFocalLengthMM;
  target->sensorWidthMM = m_sensorWidthMM;
  return true;
}

} // namespace sources::webcam