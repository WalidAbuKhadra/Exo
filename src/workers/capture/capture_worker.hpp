#pragma once
#include "core/hybrid_buffer.hpp"
#include "core/types.hpp"
#include "sources/isource.hpp"
#include "sources/source_config.hpp"

#include <atomic>
#include <memory>

namespace workers::capture {

class CaptureWorker {
public:
  CaptureWorker(const sources::SourceConfig &config, core::HybridBuffer<core::Frame> &frameHybridBuffer);

  void Run(std::atomic<bool> &running);

private:
  core::Frame *m_frameHybridBufferWriteSlot = nullptr;
  std::unique_ptr<sources::IFrameSource> m_source;
  core::HybridBuffer<core::Frame> &m_frameHybridBuffer;
};

} // namespace workers::capture