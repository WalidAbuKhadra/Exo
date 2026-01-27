#include "NvidiaARLoader.hpp"
#include "calibration/calibration_apriltag_detector.hpp"
#include "calibration/calibration_config.hpp"
#include "config/config_application.hpp"
#include "core/hybrid_buffer.hpp"
#include "core/types.hpp"
#include "engines/nvidia/nvar_config.hpp"
#include "engines/nvidia/nvar_pose.hpp"
#include "osc/osc_config.hpp"
#include "osc/osc_sender.hpp"
#include "sources/webcam/webcam_config.hpp"
#include "workers/capture/capture_worker.hpp"
#include "workers/display/display_worker.hpp"
#include "workers/engine/engine_worker.hpp"
#include "workers/output/output_worker.hpp"

#include <atomic>
#include <filesystem>
#include <functional>
#include <thread>

std::atomic<bool> g_running{true};
std::atomic<bool> g_calibrationRequest{false};

int main() {
  auto appDir = std::filesystem::current_path();

  config::ConfigApplication appConfig = config::ConfigApplication::LoadOrCreate(appDir / "config.toml");

  NvidiaARLoader nvLoader(appDir);

  sources::webcam::WebcamConfig webcamCfg;
  webcamCfg.deviceIndex = appConfig.webcam.device_index;
  webcamCfg.res = {appConfig.webcam.width, appConfig.webcam.height};
  webcamCfg.sensorWidthMM = appConfig.webcam.sensor_width_mm;
  webcamCfg.lensFocalLengthMM = appConfig.webcam.focal_length_mm;

  engines::nvidia::NvidiaConfig nvidiaCfg(webcamCfg);
  nvidiaCfg.useCudaGraph = appConfig.nvidia.use_cuda_graph;
  nvidiaCfg.mode = appConfig.nvidia.mode;
  nvidiaCfg.trackPeople = appConfig.nvidia.track_people;
  nvidiaCfg.fullBodyOnly = appConfig.nvidia.full_body_only;
  nvidiaCfg.postprocessJointAngle = appConfig.nvidia.post_process_angles;
  nvidiaCfg.temporal = appConfig.nvidia.temporal;
  nvidiaCfg.modelDir = const_cast<char *>(nvLoader.getLocalModelPath());

  osc::OscConfig oscCfg;
  oscCfg.ip = appConfig.osc.ip;
  oscCfg.port = appConfig.osc.port;
  osc::OscSender oscSender(oscCfg);

  calibration::CalibrationConfig calibrationCfg;
  calibrationCfg.solverId = appConfig.calibration.solver_id;
  calibrationCfg.tagSizeMeters = appConfig.calibration.tag_size_meters;
  calibration::CalibrationApriltagDetector calibrationApriltagDetector(calibrationCfg);

  core::HybridBuffer<core::Frame> frameHybridBuffer;
  core::HybridBuffer<engines::nvidia::NvARPose> nvARPoseHybridBuffer;

  workers::capture::CaptureWorker captureWorker(webcamCfg, frameHybridBuffer);
  workers::engine::EngineWorker engineWorker(nvidiaCfg, frameHybridBuffer, nvARPoseHybridBuffer);
  workers::output::OutputWorker outputWorker(nvARPoseHybridBuffer, calibrationApriltagDetector, oscSender);
  workers::display::DisplayWorker displayWorker(frameHybridBuffer, nvARPoseHybridBuffer, nvidiaCfg, calibrationApriltagDetector);

  std::thread t1(&workers::capture::CaptureWorker::Run, &captureWorker, std::ref(g_running));
  std::thread t2(&workers::engine::EngineWorker::Run, &engineWorker, std::ref(g_running));
  std::thread t3(&workers::output::OutputWorker::Run, &outputWorker, std::ref(g_running));
  std::thread t4(&workers::display::DisplayWorker::Run, &displayWorker, std::ref(g_running));

  if (t4.joinable())
    t4.join();

  g_running = false;
  if (t3.joinable())
    t3.join();
  if (t2.joinable())
    t2.join();
  if (t1.joinable())
    t1.join();

  return 0;
}