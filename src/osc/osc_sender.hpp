#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include "osc_config.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/Ref.h>
#include <ip/UdpSocket.h>
#include <memory>
#include <osc/OscOutboundPacketStream.h>
#include <vector>

namespace osc {

struct TrackerMap {
  int trackerID;
  int jointIndex;
};

class OscSender {
public:
  OscSender(OscConfig &oscCfg);

  void SendJoint(const Eigen::Ref<const Eigen::Matrix3Xf> &points);

private:
  std::unique_ptr<UdpTransmitSocket> m_socket;
  std::vector<char> m_buffer;
  std::unique_ptr<osc::OutboundPacketStream> m_packetStream;

  std::vector<TrackerMap> m_mappings;
};

} // namespace osc