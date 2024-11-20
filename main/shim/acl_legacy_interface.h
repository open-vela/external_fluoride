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

#include <cstdint>
#include "stack/include/bt_types.h"
#include "types/raw_address.h"

namespace bluetooth {
namespace shim {
namespace legacy {

typedef struct {
  void (*on_connected)(const RawAddress& bda, uint16_t handle, uint8_t status,
                       uint8_t enc_mode);
  void (*on_failed)(const RawAddress& bda, uint16_t handle, uint8_t status,
                    uint8_t enc_mode);
  void (*on_disconnected)(uint16_t handle, uint8_t reason);
} acl_classic_connection_interface_t;

typedef struct {
  void (*on_connected)(const tBLE_BD_ADDR& address_with_type, uint16_t handle,
                       uint8_t role, bool match, uint16_t conn_interval,
                       uint16_t conn_latency, uint16_t conn_timeout,
                       const RawAddress& local_rpa, const RawAddress& peer_rpa,
                       uint8_t peer_addr_type);
  void (*on_failed)(const tBLE_BD_ADDR& address_with_type, uint16_t handle,
                    bool enhanced, uint8_t status);
  void (*on_disconnected)(uint16_t handle, uint8_t reason);
} acl_le_connection_interface_t;

typedef struct {
  void (*on_authentication_complete)();
  void (*on_change_connection_link_key_complete)();
  void (*on_encryption_change)(bool enabled);
  void (*on_flow_specification_complete)(uint16_t flow_direction,
                                         uint16_t service_type,
                                         uint32_t token_rate,
                                         uint32_t token_bucket_size,
                                         uint32_t peak_bandwidth,
                                         uint32_t access_latency);
  void (*on_flush_occurred)();
  void (*on_master_link_key_complete)(uint8_t key_flag);
  void (*on_mode_change)(uint16_t current_mode, uint16_t interval);
  void (*on_packet_type_changed)(uint16_t packet_type);
  void (*on_qos_setup_complete)(uint16_t service_type, uint32_t token_rate,
                                uint32_t peak_bandwidth, uint32_t latency,
                                uint32_t delay_variation);
  void (*on_read_afh_channel_map_complete)(uint8_t afh_mode,
                                           uint8_t afh_channel_map[]);
  void (*on_read_automatic_flush_timeout_complete)(uint16_t flush_timeout);
  void (*on_read_clock_complete)(uint32_t clock, uint16_t accuracy);
  void (*on_read_clock_offset_complete)(uint16_t clock_offset);
  void (*on_read_failed_contact_counter_complete)(
      uint16_t failed_contact_counter);
  void (*on_read_link_policy_settings_complete)(uint16_t link_policy_settings);
  void (*on_read_link_quality_complete)(uint8_t link_quality);
  void (*on_read_link_supervision_timeout_complete)(
      uint16_t link_supervision_timeout);
  void (*on_read_rssi_complete)(uint8_t rssi);
  void (*on_read_remote_version_information_complete)();
  void (*on_read_transmit_power_level_complete)(uint8_t transmit_power_level);
  void (*on_role_change)(uint8_t new_role);
  void (*on_role_discovery_complete)(uint8_t current_role);
} acl_classic_link_interface_t;
;

typedef struct {
  void (*on_connection_update)(uint16_t connection_interval,
                               uint16_t connection_latency,
                               uint16_t supervision_timeout);
  void (*on_data_length_change)(uint16_t tx_octets, uint16_t tx_time,
                                uint16_t rx_octets, uint16_t rx_time);
} acl_le_link_interface_t;

typedef struct {
  acl_classic_connection_interface_t classic;
  acl_le_connection_interface_t le;
} acl_connection_interface_t;

typedef struct {
  acl_classic_link_interface_t classic;
  acl_le_link_interface_t le;
} acl_link_interface_t;

typedef struct {
  void (*on_send_data_upwards)(BT_HDR*);
  acl_connection_interface_t connection;
  acl_link_interface_t link;
} acl_interface_t;

const acl_interface_t GetAclInterface();

}  // namespace legacy
}  // namespace shim
}  // namespace bluetooth
