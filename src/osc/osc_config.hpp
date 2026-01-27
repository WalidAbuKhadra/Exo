#pragma once
#include <string>

namespace osc {

struct OscConfig {
  std::string ip = "127.0.0.1";
  int port = 9000;
};
} // namespace osc