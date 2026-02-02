#include "osc_sender.hpp"

#include "core/types.hpp"
#include "osc_config.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/Ref.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <ip/IpEndpointName.h>
#include <ip/UdpSocket.h>
#include <memory>
#include <osc/OscOutboundPacketStream.h>
#include <osc/OscTypes.h>
#include <string>
#include <vector>

namespace osc {

OscSender::OscSender(OscConfig &oscCfg) {
  m_socket = std::make_unique<UdpTransmitSocket>(IpEndpointName(oscCfg.ip.c_str(), oscCfg.port));
  m_buffer.resize(8192);
  m_packetStream = std::make_unique<osc::OutboundPacketStream>(m_buffer.data(), m_buffer.size());
}

void OscSender::SendSkeleton(const Eigen::Ref<const Eigen::Matrix3Xf> &points, const Eigen::Ref<const Eigen::Matrix4Xf> &rotations, const std::vector<float> &confidences) {

  m_packetStream->Clear();
  (*m_packetStream) << osc::BeginBundleImmediate;

  (*m_packetStream) << osc::BeginMessage("/skeleton/pos");
  for (int i = 0; i < points.cols(); ++i) {
    (*m_packetStream) << points(0, i) << points(1, i) << points(2, i);
  }
  (*m_packetStream) << osc::EndMessage;

  (*m_packetStream) << osc::BeginMessage("/skeleton/rot");
  for (int i = 0; i < rotations.cols(); ++i) {
    (*m_packetStream) << rotations(0, i) << rotations(1, i) << rotations(2, i) << rotations(3, i);
  }
  (*m_packetStream) << osc::EndMessage;

  (*m_packetStream) << osc::BeginMessage("/skeleton/conf");
  for (float c : confidences) {
    (*m_packetStream) << c;
  }
  (*m_packetStream) << osc::EndMessage;

  (*m_packetStream) << osc::EndBundle;
  m_socket->Send(m_packetStream->Data(), m_packetStream->Size());
}

void OscSender::SendTags(const std::vector<core::TagData> &tags) {
  m_packetStream->Clear();
  (*m_packetStream) << osc::BeginBundleImmediate;

  for (const auto &tag : tags) {
    Eigen::Vector3f pos = tag.transform.translation();
    Eigen::Quaternionf rot(tag.transform.rotation());

    std::string base = "/calibration/tag/" + std::to_string(tag.id);

    (*m_packetStream) << osc::BeginMessage((base + "/position").c_str())
                      << pos.x() << pos.y() << pos.z() << osc::EndMessage;

    (*m_packetStream) << osc::BeginMessage((base + "/rotation").c_str())
                      << rot.x() << rot.y() << rot.z() << rot.w() << osc::EndMessage;
  }

  (*m_packetStream) << osc::EndBundle;
  m_socket->Send(m_packetStream->Data(), m_packetStream->Size());
}

} // namespace osc