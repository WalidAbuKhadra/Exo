#pragma once
#include "core/types.hpp"

#include <string_view>

namespace sources {

class IFrameSource {
public:
  virtual ~IFrameSource() = default;
  virtual bool Start() = 0;
  virtual void Stop() = 0;

  virtual bool CaptureInto(core::Frame *target) = 0;

  virtual std::string_view Name() const = 0;
};

} // namespace sources