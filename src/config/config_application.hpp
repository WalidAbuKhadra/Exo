#pragma once
#include <filesystem>
#include <string>

namespace config {

struct OscSettings {
  std::string ip = "127.0.0.1";
  int port = 9000;
};

struct WebcamSettings {
  int device_index = 0;
  int width = 1920;
  int height = 1080;
  int fps = 30;
  float sensor_width_mm = 4.8f;
  float focal_length_mm = 3.67f;
};

struct NvidiaSettings {
  bool use_cuda_graph = true;
  int mode = 0;
  bool track_people = false;
  bool full_body_only = false;
  bool post_process_angles = true;
  int temporal = 1;
};

struct CalibrationSettings {
  double tag_size_meters = 0.045;
  int solver_id = 0;
};

struct ConfigApplication {
  WebcamSettings webcam;
  NvidiaSettings nvidia;
  OscSettings osc;
  CalibrationSettings calibration;

  static ConfigApplication LoadOrCreate(const std::filesystem::path &path);
  void Save(const std::filesystem::path &path) const;
};

} // namespace config