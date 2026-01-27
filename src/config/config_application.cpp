#include "config_application.hpp"

#include <filesystem>
#include <fstream>
#include <spdlog/spdlog.h>
#include <toml++/impl/parse_error.hpp>
#include <toml++/impl/parser.hpp>
#include <toml++/impl/table.hpp>

namespace config {

void ConfigApplication::Save(const std::filesystem::path &path) const {
  auto root = toml::table{{"webcam", toml::table{
                                         {"device_index", webcam.device_index},
                                         {"width", webcam.width},
                                         {"height", webcam.height},
                                         {"fps", webcam.fps},
                                         {"sensor_width_mm", webcam.sensor_width_mm},
                                         {"focal_length_mm", webcam.focal_length_mm},
                                     }},
                          {"nvidia", toml::table{
                                         {"use_cuda_graph", nvidia.use_cuda_graph},
                                         {"mode", nvidia.mode},
                                         {"track_people", nvidia.track_people},
                                         {"full_body_only", nvidia.full_body_only},
                                         {"post_process_angles", nvidia.post_process_angles},
                                         {"temporal_filter_level", nvidia.temporal},
                                     }},
                          {"osc", toml::table{
                                      {"ip", osc.ip},
                                      {"port", osc.port},
                                  }},
                          {"calibration", toml::table{
                                              {"tag_size_meters", calibration.tag_size_meters},
                                              {"solver_id", calibration.solver_id},
                                          }}};

  std::ofstream ofs(path);
  if (ofs.is_open()) {
    ofs << root << "\n";
    spdlog::info("Configuration saved to {}", path.string());
  } else {
    spdlog::error("Failed to save configuration to {}", path.string());
  }
}

ConfigApplication ConfigApplication::LoadOrCreate(const std::filesystem::path &path) {
  ConfigApplication config;

  if (!std::filesystem::exists(path)) {
    spdlog::warn("Config file not found. Creating default at {}", path.string());
    config.Save(path);
    return config;
  }

  try {
    toml::table tbl = toml::parse_file(path.string());

    if (auto section = tbl["webcam"]) {
      config.webcam.device_index = section["device_index"].value_or(config.webcam.device_index);
      config.webcam.width = section["width"].value_or(config.webcam.width);
      config.webcam.height = section["height"].value_or(config.webcam.height);
      config.webcam.fps = section["fps"].value_or(config.webcam.fps);
      config.webcam.sensor_width_mm = section["sensor_width_mm"].value_or(config.webcam.sensor_width_mm);
      config.webcam.focal_length_mm = section["focal_length_mm"].value_or(config.webcam.focal_length_mm);
    }

    if (auto section = tbl["nvidia"]) {
      config.nvidia.use_cuda_graph = section["use_cuda_graph"].value_or(config.nvidia.use_cuda_graph);
      config.nvidia.mode = section["mode"].value_or(config.nvidia.mode);
      config.nvidia.track_people = section["track_people"].value_or(config.nvidia.track_people);
      config.nvidia.full_body_only = section["full_body_only"].value_or(config.nvidia.full_body_only);
      config.nvidia.post_process_angles = section["post_process_angles"].value_or(config.nvidia.post_process_angles);
      config.nvidia.temporal = section["temporal_filter_level"].value_or(config.nvidia.temporal);
    }

    if (auto section = tbl["osc"]) {
      config.osc.ip = section["ip"].value_or(config.osc.ip);
      config.osc.port = section["port"].value_or(config.osc.port);
    }

    if (auto section = tbl["calibration"]) {
      config.calibration.tag_size_meters = section["tag_size_meters"].value_or(config.calibration.tag_size_meters);
      config.calibration.solver_id = section["solver_id"].value_or(config.calibration.solver_id);
    }

    spdlog::info("Configuration loaded from {}", path.string());

  } catch (const toml::parse_error &err) {
    spdlog::error("Failed to parse config: {}", err.description());
    spdlog::warn("Using default configuration.");
  }

  return config;
}

} // namespace config