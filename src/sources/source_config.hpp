#pragma once
#include "webcam/webcam_config.hpp"

#include <variant>

namespace sources {

using SourceConfig = std::variant<webcam::WebcamConfig>;

} // namespace sources