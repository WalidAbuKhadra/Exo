#include "nvar_body_pose_estimation.hpp"

#include "core/hybrid_buffer.hpp"
#include "core/types.hpp"
#include "helpers/helpers.hpp"
#include "nvar_config.hpp"
#include "nvar_pose.hpp"

#include <cstdint>
#include <cstring>
#include <nvAR.h>
#include <nvARBodyPoseEstimation.h>
#include <nvAR_defs.h>
#include <nvCVImage.h>
#include <spdlog/spdlog.h>

namespace engines::nvidia {

NvARBodyPoseEstimation::NvARBodyPoseEstimation(const NvidiaConfig &config) : m_config(config) {
  m_focalLengthPixels = (m_config.lensFocalLengthMM * (float)m_config.res.width) / m_config.sensorWidthMM;

  m_modelDir = m_config.modelDir;
  m_batchSize = m_config.trackPeople ? 8 : 1;
  m_mode = m_config.fullBodyOnly ? m_config.mode : 0;
  m_useCudaGraph = m_config.useCudaGraph;
  m_temporal = m_config.temporal;
  m_fullBodyOnly = m_config.fullBodyOnly;
  m_postprocessJointAngle = m_config.fullBodyOnly ? false : m_config.postprocessJointAngle;
  m_trackPeople = m_config.trackPeople;
  m_shadowTrackingAge = m_config.shadowTrackingAge;
  m_probationAge = m_config.probationAge;
  m_maxTargetsTracked = m_config.maxTargetsTracked;
}

NvARBodyPoseEstimation::~NvARBodyPoseEstimation() {
  NvCVImage_Dealloc(&m_srcGPUImg);
  NvCVImage_Dealloc(&m_srcCPUImg);

  if (m_keypointDetectHandle) {
    NvAR_Destroy(m_keypointDetectHandle);
    m_keypointDetectHandle = nullptr;
  }

  if (m_cudaStream) {
    NvAR_CudaStreamDestroy(m_cudaStream);
    m_cudaStream = nullptr;
  }
}

static void NvARLogCallback(void *context, const char *msg) {
  (void)context; // suppress unused parameter warnings
  spdlog::info("[NvAR SDK Internal] {}", msg);
}

bool NvARBodyPoseEstimation::Initialize(core::HybridBuffer<engines::nvidia::NvARPose> &nvARPoseHybridBuffer) {
  NvAR_ConfigureLogger(3, nullptr, &NvARLogCallback, nullptr);
  NvCVImage_Alloc(&m_srcGPUImg, m_config.res.width, m_config.res.height, NVCV_BGR, NVCV_U8, NVCV_CHUNKY, NVCV_GPU, 1);

  NvAR_Create(NvAR_Feature_BodyPoseEstimation, &m_keypointDetectHandle);
  NvAR_CudaStreamCreate(&m_cudaStream);

  NvAR_SetCudaStream(m_keypointDetectHandle, NvAR_Parameter_Config(CUDAStream), m_cudaStream);
  NvAR_SetString(m_keypointDetectHandle, NvAR_Parameter_Config(ModelDir), m_modelDir);
  NvAR_SetU32(m_keypointDetectHandle, NvAR_Parameter_Config(BatchSize), m_batchSize);
  NvAR_SetU32(m_keypointDetectHandle, NvAR_Parameter_Config(Mode), m_mode);
  NvAR_SetU32(m_keypointDetectHandle, NvAR_Parameter_Config(UseCudaGraph), m_useCudaGraph);
  NvAR_SetU32(m_keypointDetectHandle, NvAR_Parameter_Config(Temporal), m_temporal);

  NvAR_SetU32(m_keypointDetectHandle, NvAR_Parameter_Config(FullBodyOnly), m_fullBodyOnly);
  NvAR_SetU32(m_keypointDetectHandle, NvAR_Parameter_Config(PostprocessJointAngle), m_postprocessJointAngle);

  NvAR_SetU32(m_keypointDetectHandle, NvAR_Parameter_Config(TrackPeople), m_trackPeople);
  NvAR_SetU32(m_keypointDetectHandle, NvAR_Parameter_Config(ShadowTrackingAge), m_shadowTrackingAge);
  NvAR_SetU32(m_keypointDetectHandle, NvAR_Parameter_Config(ProbationAge), m_probationAge);
  NvAR_SetU32(m_keypointDetectHandle, NvAR_Parameter_Config(MaxTargetsTracked), m_maxTargetsTracked);

  NvAR_Load(m_keypointDetectHandle);

  NvAR_GetU32(m_keypointDetectHandle, NvAR_Parameter_Config(NumKeyPoints), &m_numKeyPoints);

  m_referencePose.assign(m_numKeyPoints, {0.f, 0.f, 0.f});
  const void *tempReferencePose = nullptr;
  NvAR_GetObject(m_keypointDetectHandle, NvAR_Parameter_Config(ReferencePose), &tempReferencePose,
                 sizeof(NvAR_Point3f));
  memcpy(m_referencePose.data(), tempReferencePose, sizeof(NvAR_Point3f) * m_numKeyPoints);

  m_targetSeatedPoseForInterpolation.assign(m_numKeyPoints, {0.f, 0.f, 0.f, 1.f});
  const void *tempTargetSeatedPoseForInterpolation = nullptr;
  NvAR_GetObject(m_keypointDetectHandle, NvAR_Parameter_Config(TargetSeatedPoseForInterpolation),
                 &tempTargetSeatedPoseForInterpolation, sizeof(NvAR_Quaternion));
  memcpy(m_targetSeatedPoseForInterpolation.data(), tempTargetSeatedPoseForInterpolation,
         sizeof(NvAR_Quaternion) * m_numKeyPoints);

  m_targetStandPoseForInterpolation.assign(m_numKeyPoints, {0.f, 0.f, 0.f, 1.f});
  const void *tempTargetStandPoseForInterpolation = nullptr;
  NvAR_GetObject(m_keypointDetectHandle, NvAR_Parameter_Config(TargetStandPoseForInterpolation),
                 &tempTargetStandPoseForInterpolation, sizeof(NvAR_Quaternion));
  memcpy(m_targetStandPoseForInterpolation.data(), tempTargetStandPoseForInterpolation,
         sizeof(NvAR_Quaternion) * m_numKeyPoints);

  NvAR_SetF32(m_keypointDetectHandle, NvAR_Parameter_Input(FocalLength), m_focalLengthPixels);

  uint32_t bSize = m_batchSize;
  uint32_t nPoints = m_numKeyPoints;
  uint32_t tPeople = m_trackPeople;

  nvARPoseHybridBuffer.InitializeBuffer([bSize, nPoints, tPeople](engines::nvidia::NvARPose &pose) {
    uint64_t totalKeypoints = static_cast<uint64_t>(bSize) * static_cast<uint64_t>(nPoints);
    pose.keypoints.assign(static_cast<size_t>(totalKeypoints), {0.f, 0.f});
    pose.keypoints3D.assign(static_cast<size_t>(totalKeypoints), {0.f, 0.f, 0.f});
    pose.jointAngles.assign(static_cast<size_t>(totalKeypoints), {0.f, 0.f, 0.f, 1.f});
    pose.keypointsConfidence.assign(static_cast<size_t>(totalKeypoints), 0.f);

    if (tPeople) {
      pose.trackingBoundingBox.assign(30, {{0.f, 0.f, 0.f, 0.f}, 0});
      pose.trackingBoundingBoxes.boxes = pose.trackingBoundingBox.data();
      pose.trackingBoundingBoxes.max_boxes = (uint8_t)pose.trackingBoundingBox.size();
      pose.trackingBoundingBoxes.num_boxes = 0;
      pose.boundingBoxConfidence.assign(pose.trackingBoundingBox.size(), 0.f);

    } else {
      pose.boundingBox.assign(25, {0.f, 0.f, 0.f, 0.f});
      pose.boundingBoxes.boxes = pose.boundingBox.data();
      pose.boundingBoxes.max_boxes = (uint8_t)pose.boundingBox.size();
      pose.boundingBoxes.num_boxes = 0;
      pose.boundingBoxConfidence.assign(pose.boundingBox.size(), 0.f);
    }
  });

  return true;
}

bool NvARBodyPoseEstimation::Run(core::Frame &frameHybridBufferReadSlot, engines::nvidia::NvARPose *nvARPoseHybridBufferWriteSlot) {
  estimation::nvidia::helpers::NVWrapperForCVMat(&frameHybridBufferReadSlot.image, &m_srcCPUImg);
  NvCVImage_Transfer(&m_srcCPUImg, &m_srcGPUImg, 1.0f, m_cudaStream, &m_stagingImg);
  NvAR_SetObject(m_keypointDetectHandle, NvAR_Parameter_Input(Image), &m_srcGPUImg, sizeof(NvCVImage));

  NvAR_SetF32Array(m_keypointDetectHandle, NvAR_Parameter_Output(BoundingBoxesConfidence),
                   nvARPoseHybridBufferWriteSlot->boundingBoxConfidence.data(),
                   nvARPoseHybridBufferWriteSlot->boundingBoxes.max_boxes);

  NvAR_SetObject(m_keypointDetectHandle, NvAR_Parameter_Output(KeyPoints),
                 nvARPoseHybridBufferWriteSlot->keypoints.data(), sizeof(NvAR_Point2f));
  NvAR_SetObject(m_keypointDetectHandle, NvAR_Parameter_Output(KeyPoints3D),
                 nvARPoseHybridBufferWriteSlot->keypoints3D.data(), sizeof(NvAR_Point3f));
  NvAR_SetObject(m_keypointDetectHandle, NvAR_Parameter_Output(JointAngles),
                 nvARPoseHybridBufferWriteSlot->jointAngles.data(), sizeof(NvAR_Quaternion));
  NvAR_SetF32Array(m_keypointDetectHandle, NvAR_Parameter_Output(KeyPointsConfidence),
                   nvARPoseHybridBufferWriteSlot->keypointsConfidence.data(), sizeof(float));
  NvAR_SetObject(m_keypointDetectHandle, NvAR_Parameter_Output(BoundingBoxes),
                 &nvARPoseHybridBufferWriteSlot->boundingBoxes, sizeof(NvAR_BBoxes));
  NvAR_SetObject(m_keypointDetectHandle, NvAR_Parameter_Output(TrackingBoundingBoxes),
                 &nvARPoseHybridBufferWriteSlot->trackingBoundingBoxes, sizeof(NvAR_TrackingBBoxes));

  NvAR_Run(m_keypointDetectHandle);
  return true;
}
} // namespace engines::nvidia