#pragma once
#include "isource.hpp"
#include "source_config.hpp"

#include <memory>

namespace sources {

class FrameSourceFactory {
public:
  static std::unique_ptr<IFrameSource> Create(const SourceConfig &config);
};

} // namespace sources