#include "engine_worker.hpp"

#include "core/hybrid_buffer.hpp"
#include "core/types.hpp"
#include "engines/nvidia/nvar_body_pose_estimation.hpp"
#include "engines/nvidia/nvar_config.hpp"
#include "engines/nvidia/nvar_pose.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

namespace workers::engine {

EngineWorker::EngineWorker(const engines::nvidia::NvidiaConfig &config, core::HybridBuffer<core::Frame> &frameHybridBuffer, core::HybridBuffer<engines::nvidia::NvARPose> &nvARPoseHybridBuffer) : m_frameHybridBuffer(frameHybridBuffer), m_nvARPoseHybridBuffer(nvARPoseHybridBuffer) {
  m_engine = std::make_unique<engines::nvidia::NvARBodyPoseEstimation>(config);
  m_engine->Initialize(m_nvARPoseHybridBuffer);
}

void EngineWorker::Run(std::atomic<bool> &running) {

  if (!m_engine) {
    running = false;
    return;
  }

  while (running) {
    m_frameHybridBufferReadSlot = m_frameHybridBuffer.Fetch();
    m_nvARPoseHybridBufferWriteSlot = m_nvARPoseHybridBuffer.GetWriteBuffer();
    if (!m_frameHybridBufferReadSlot || m_frameHybridBufferReadSlot->image.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    m_engine->Run(*m_frameHybridBufferReadSlot, m_nvARPoseHybridBufferWriteSlot);
    m_nvARPoseHybridBuffer.Publish();
  }
}

} // namespace workers::engine