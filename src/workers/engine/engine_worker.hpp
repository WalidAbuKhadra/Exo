#pragma once
#include "core/hybrid_buffer.hpp"
#include "core/types.hpp"
#include "engines/nvidia/nvar_body_pose_estimation.hpp"
#include "engines/nvidia/nvar_config.hpp"
#include "engines/nvidia/nvar_pose.hpp"

#include <atomic>
#include <memory>

namespace workers::engine {

class EngineWorker {
public:
  EngineWorker(const engines::nvidia::NvidiaConfig &config, core::HybridBuffer<core::Frame> &frameHybridBuffer, core::HybridBuffer<engines::nvidia::NvARPose> &nvARPoseHybridBuffer);

  void Run(std::atomic<bool> &running);

private:
  core::Frame *m_frameHybridBufferReadSlot = nullptr;
  engines::nvidia::NvARPose *m_nvARPoseHybridBufferWriteSlot = nullptr;

  std::unique_ptr<engines::nvidia::NvARBodyPoseEstimation> m_engine;

  core::HybridBuffer<core::Frame> &m_frameHybridBuffer;
  core::HybridBuffer<engines::nvidia::NvARPose> &m_nvARPoseHybridBuffer;
};

} // namespace workers::engine