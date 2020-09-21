/******************************************************************************
 *
 *  Copyright 2016 Google, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#include <unistd.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>

#include <base/base64.h>
#include <base/logging.h>
#include <include/hardware/bt_av.h>
#include <statslog.h>

#include "common/metrics.h"
#include "osi/include/osi.h"
#include "stack/include/btm_api_types.h"

namespace bluetooth {
  namespace common {

    void LogLinkLayerConnectionEvent(const RawAddress* address, uint32_t connection_handle,
        android::bluetooth::DirectionEnum direction, uint16_t link_type, uint32_t hci_cmd,
        uint16_t hci_event, uint16_t hci_ble_event, uint16_t cmd_status, uint16_t reason_code) { }

    void LogHciTimeoutEvent(uint32_t hci_cmd) { }
    void LogRemoteVersionInfo(uint16_t handle, uint8_t status, uint8_t version,
        uint16_t manufacturer_name, uint16_t subversion) { }

    void LogA2dpAudioUnderrunEvent(const RawAddress& address, uint64_t encoding_interval_millis, int num_missing_pcm_bytes) { }
    void LogA2dpAudioOverrunEvent(const RawAddress& address, uint64_t encoding_interval_millis, int num_dropped_buffers, int num_dropped_encoded_frames, int num_dropped_encoded_bytes) { }
    void LogReadRssiResult(const RawAddress& address, uint16_t handle, uint32_t cmd_status, int8_t rssi) { }
    void LogReadFailedContactCounterResult(const RawAddress& address, uint16_t handle, uint32_t cmd_status, int32_t failed_contact_counter) { }
    void LogReadTxPowerLevelResult(const RawAddress& address, uint16_t handle, uint32_t cmd_status, int32_t transmit_power_level) { }
    void LogSmpPairingEvent(const RawAddress& address, uint8_t smp_cmd, android::bluetooth::DirectionEnum direction, uint8_t smp_fail_reason) { }
    void LogClassicPairingEvent(const RawAddress& address, uint16_t handle, uint32_t hci_cmd, uint16_t hci_event, uint16_t cmd_status, uint16_t reason_code, int64_t event_value) { }
    void LogSdpAttribute(const RawAddress& address, uint16_t protocol_uuid, uint16_t attribute_id, size_t attribute_size, const char* attribute_value) { }
    void LogSocketConnectionState(const RawAddress& address, int port, int type, android::bluetooth::SocketConnectionstateEnum connection_state,
        int64_t tx_bytes, int64_t rx_bytes, int uid, int server_port, android::bluetooth::SocketRoleEnum socket_role) { }
    void LogManufacturerInfo(const RawAddress& address, android::bluetooth::DeviceInfoSrcEnum source_type,
        const std::string& source_name, const std::string& manufacturer, const std::string& model, const std::string& hardware_version, const std::string& software_version) { }

  }  // namespace common
}  // namespace bluetooth
