#include "source_factory.hpp"

#include "isource.hpp"
#include "source_config.hpp"
#include "webcam/webcam_config.hpp"
#include "webcam/webcam_source.hpp"

#include <memory>
#include <variant>

namespace sources {

template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

std::unique_ptr<IFrameSource> FrameSourceFactory::Create(const SourceConfig &config) {
  return std::visit(overloaded{[](const webcam::WebcamConfig &cfg) -> std::unique_ptr<IFrameSource> {
                      auto src = std::make_unique<webcam::WebcamSource>(cfg);
                      if (src->Start())
                        return src;
                      return nullptr;
                    }},
                    config);
}

} // namespace sources