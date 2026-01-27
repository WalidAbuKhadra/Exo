#include "osc_sender.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include "draw/skeleton.hpp"
#include "osc_config.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/Ref.h>
#include <ip/IpEndpointName.h>
#include <ip/UdpSocket.h>
#include <memory>
#include <osc/OscOutboundPacketStream.h>
#include <osc/OscTypes.h>
#include <string>

namespace osc {

OscSender::OscSender(OscConfig &oscCfg) {
  m_socket = std::make_unique<UdpTransmitSocket>(IpEndpointName(oscCfg.ip.c_str(), oscCfg.port));
  m_buffer.resize(4096);
  m_packetStream = std::make_unique<osc::OutboundPacketStream>(m_buffer.data(), m_buffer.size());

  m_mappings = {{1, draw::KP::Pelvis}, {2, draw::KP::L_Hip}, {3, draw::KP::R_Hip}, {4, draw::KP::Torso}, {5, draw::KP::L_Knee}, {6, draw::KP::R_Knee}, {7, draw::KP::Neck}, {8, draw::KP::L_Ankle}, {9, draw::KP::R_Ankle}, {10, draw::KP::L_BigToe}, {11, draw::KP::R_BigToe}, {12, draw::KP::L_SmallToe}, {13, draw::KP::R_SmallToe}, {14, draw::KP::L_Heel}, {15, draw::KP::R_Heel}, {16, draw::KP::Nose}, {17, draw::KP::L_Eye}, {18, draw::KP::R_Eye}, {19, draw::KP::L_Ear}, {20, draw::KP::R_Ear}, {21, draw::KP::L_Shoulder}, {22, draw::KP::R_Shoulder}, {23, draw::KP::L_Elbow}, {24, draw::KP::R_Elbow}, {25, draw::KP::L_Wrist}, {26, draw::KP::R_Wrist}, {27, draw::KP::L_Pinky}, {28, draw::KP::R_Pinky}, {29, draw::KP::L_Middle}, {30, draw::KP::R_Middle}, {31, draw::KP::L_Index}, {32, draw::KP::R_Index}, {33, draw::KP::L_Thumb}, {34, draw::KP::R_Thumb}};
}

void OscSender::SendJoint(const Eigen::Ref<const Eigen::Matrix3Xf> &points) {

  m_packetStream->Clear();
  (*m_packetStream) << osc::BeginBundleImmediate;

  for (const auto &map : m_mappings) {

    const auto &pt = points.col(map.jointIndex);

    std::string posAddr = "/tracking/trackers/" + std::to_string(map.trackerID) + "/position";
    std::string rotAddr = "/tracking/trackers/" + std::to_string(map.trackerID) + "/rotation";

    (*m_packetStream) << osc::BeginMessage(posAddr.c_str()) << pt.x() << pt.y() << pt.z() << osc::EndMessage;

    (*m_packetStream) << osc::BeginMessage(rotAddr.c_str()) << 0.0f << 0.0f << 0.0f << osc::EndMessage;
  }

  (*m_packetStream) << osc::EndBundle;
  m_socket->Send(m_packetStream->Data(), m_packetStream->Size());
}
} // namespace osc