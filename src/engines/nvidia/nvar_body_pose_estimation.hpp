#pragma once
#include "core/hybrid_buffer.hpp"
#include "core/types.hpp"
#include "nvar_config.hpp"
#include "nvar_pose.hpp"

#include <cstdint>
#include <nvAR.h>
#include <nvAR_defs.h>
#include <nvCVImage.h>
#include <vector>

namespace engines::nvidia {

class NvARBodyPoseEstimation {

public:
  explicit NvARBodyPoseEstimation(const NvidiaConfig &config);
  ~NvARBodyPoseEstimation();
  bool Initialize(core::HybridBuffer<engines::nvidia::NvARPose> &nvARPoseHybridBuffer);
  bool Run(core::Frame &frameHybridBufferReadSlot, engines::nvidia::NvARPose *nvARPoseHybridBufferWriteSlot);

private:
  const NvidiaConfig &m_config;
  NvAR_FeatureHandle m_keypointDetectHandle = nullptr;

  NvCVImage m_srcCPUImg{};
  NvCVImage m_srcGPUImg{};
  NvCVImage m_stagingImg{};

  float m_focalLengthPixels{0.0f};

  char *m_featureDescription = nullptr;
  CUstream m_cudaStream = nullptr;
  char *m_modelDir = nullptr;
  uint32_t m_batchSize{1};
  uint32_t m_mode = 0;
  bool m_useCudaGraph = true;
  uint32_t m_temporal = 1;
  uint32_t m_numKeyPoints{0};
  std::vector<NvAR_Point3f> m_referencePose;
  uint32_t m_fullBodyOnly = 0;
  bool m_postprocessJointAngle = false;
  std::vector<NvAR_Quaternion> m_targetSeatedPoseForInterpolation;
  std::vector<NvAR_Quaternion> m_targetStandPoseForInterpolation;
  uint32_t m_trackPeople = 0;
  uint32_t m_shadowTrackingAge = 90;
  uint32_t m_probationAge = 10;
  uint32_t m_maxTargetsTracked = 30;
};

} // namespace engines::nvidia