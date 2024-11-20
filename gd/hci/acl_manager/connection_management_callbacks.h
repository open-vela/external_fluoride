/*
 * Copyright 2020 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <memory>
#include "hci/hci_packets.h"

namespace bluetooth {
namespace hci {
namespace acl_manager {

class ConnectionManagementCallbacks {
 public:
  virtual ~ConnectionManagementCallbacks() = default;
  // Invoked when controller sends Connection Packet Type Changed event with Success error code
  virtual void OnConnectionPacketTypeChanged(uint16_t packet_type) = 0;
  // Invoked when controller sends Authentication Complete event with Success error code
  virtual void OnAuthenticationComplete() = 0;
  // Invoked when controller sends Encryption Change event with Success error code
  virtual void OnEncryptionChange(EncryptionEnabled enabled) = 0;
  // Invoked when controller sends Change Connection Link Key Complete event with Success error code
  virtual void OnChangeConnectionLinkKeyComplete() = 0;
  // Invoked when controller sends Read Clock Offset Complete event with Success error code
  virtual void OnReadClockOffsetComplete(uint16_t clock_offset) = 0;
  // Invoked when controller sends Mode Change event with Success error code
  virtual void OnModeChange(Mode current_mode, uint16_t interval) = 0;
  // Invoked when controller sends QoS Setup Complete event with Success error code
  virtual void OnQosSetupComplete(ServiceType service_type, uint32_t token_rate, uint32_t peak_bandwidth,
                                  uint32_t latency, uint32_t delay_variation) = 0;
  // Invoked when controller sends Flow Specification Complete event with Success error code
  virtual void OnFlowSpecificationComplete(FlowDirection flow_direction, ServiceType service_type, uint32_t token_rate,
                                           uint32_t token_bucket_size, uint32_t peak_bandwidth,
                                           uint32_t access_latency) = 0;
  // Invoked when controller sends Flush Occurred event
  virtual void OnFlushOccurred() = 0;
  // Invoked when controller sends Command Complete event for Role Discovery command with Success error code
  virtual void OnRoleDiscoveryComplete(Role current_role) = 0;
  // Invoked when controller sends Command Complete event for Read Link Policy Settings command with Success error code
  virtual void OnReadLinkPolicySettingsComplete(uint16_t link_policy_settings) = 0;
  // Invoked when controller sends Command Complete event for Read Automatic Flush Timeout command with Success error
  // code
  virtual void OnReadAutomaticFlushTimeoutComplete(uint16_t flush_timeout) = 0;
  // Invoked when controller sends Command Complete event for Read Transmit Power Level command with Success error code
  virtual void OnReadTransmitPowerLevelComplete(uint8_t transmit_power_level) = 0;
  // Invoked when controller sends Command Complete event for Read Link Supervision Time out command with Success error
  // code
  virtual void OnReadLinkSupervisionTimeoutComplete(uint16_t link_supervision_timeout) = 0;
  // Invoked when controller sends Command Complete event for Read Failed Contact Counter command with Success error
  // code
  virtual void OnReadFailedContactCounterComplete(uint16_t failed_contact_counter) = 0;
  // Invoked when controller sends Command Complete event for Read Link Quality command with Success error code
  virtual void OnReadLinkQualityComplete(uint8_t link_quality) = 0;
  // Invoked when controller sends Command Complete event for Read AFH Channel Map command with Success error code
  virtual void OnReadAfhChannelMapComplete(AfhMode afh_mode, std::array<uint8_t, 10> afh_channel_map) = 0;
  // Invoked when controller sends Command Complete event for Read RSSI command with Success error code
  virtual void OnReadRssiComplete(uint8_t rssi) = 0;
  // Invoked when controller sends Command Complete event for Read Clock command with Success error code
  virtual void OnReadClockComplete(uint32_t clock, uint16_t accuracy) = 0;
  // Invoked when controller sends Master Link Key Complete event
  virtual void OnMasterLinkKeyComplete(KeyFlag key_flag) = 0;
  // Invoked when controller sends Role Change event
  virtual void OnRoleChange(Role new_role) = 0;
  // Invoked when controller sends DisconnectComplete
  virtual void OnDisconnection(ErrorCode reason) = 0;
};

}  // namespace acl_manager
}  // namespace hci
}  // namespace bluetooth
