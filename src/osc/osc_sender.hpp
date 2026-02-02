#pragma once

#include "core/types.hpp"
#include "osc_config.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/Ref.h>
#include <ip/UdpSocket.h>
#include <memory>
#include <osc/OscOutboundPacketStream.h>
#include <vector>

namespace osc {

class OscSender {
public:
  OscSender(OscConfig &oscCfg);

  void SendSkeleton(const Eigen::Ref<const Eigen::Matrix3Xf> &points, const Eigen::Ref<const Eigen::Matrix4Xf> &rotations, const std::vector<float> &confidences);

  void SendTags(const std::vector<core::TagData> &tags);

private:
  std::unique_ptr<UdpTransmitSocket> m_socket;
  std::vector<char> m_buffer;
  std::unique_ptr<osc::OutboundPacketStream> m_packetStream;
};

} // namespace osc