#include "capture_worker.hpp"

#include "core/hybrid_buffer.hpp"
#include "core/types.hpp"
#include "sources/isource.hpp"
#include "sources/source_config.hpp"
#include "sources/source_factory.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

namespace workers::capture {

CaptureWorker::CaptureWorker(const sources::SourceConfig &config, core::HybridBuffer<core::Frame> &frameHybridBuffer) : m_frameHybridBuffer(frameHybridBuffer) {
  m_source = sources::FrameSourceFactory::Create(config);
}

void CaptureWorker::Run(std::atomic<bool> &running) {

  if (!m_source->Start()) {
    running = false;
    return;
  }

  while (running) {
    m_frameHybridBufferWriteSlot = m_frameHybridBuffer.GetWriteBuffer();

    if (m_source->CaptureInto(m_frameHybridBufferWriteSlot)) {
      m_frameHybridBuffer.Publish();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  m_source->Stop();
}

} // namespace workers::capture