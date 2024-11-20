/*
 * Copyright 2019 The Android Open Source Project
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

#define LOG_TAG "bt_shim_hci"

#include "hci/hci_layer.h"

#include <base/bind.h>

#include <algorithm>
#include <cstdint>

#include "hci/hci_packets.h"
#include "hci/include/packet_fragmenter.h"
#include "hci/le_acl_connection_interface.h"
#include "main/shim/hci_layer.h"
#include "main/shim/shim.h"
#include "osi/include/allocator.h"
#include "osi/include/future.h"
#include "packet/raw_builder.h"
#include "stack/include/bt_types.h"

/**
 * Callback data wrapped as opaque token bundled with the command
 * transmit request to the Gd layer.
 *
 * Upon completion a token for a corresponding command transmit.
 * request is returned from the Gd layer.
 */
using CommandCallbackData = struct { void* context; };

constexpr size_t kBtHdrSize = sizeof(BT_HDR);
constexpr size_t kCommandLengthSize = sizeof(uint8_t);
constexpr size_t kCommandOpcodeSize = sizeof(uint16_t);

static base::Callback<void(const base::Location&, BT_HDR*)> send_data_upwards;
static const packet_fragmenter_t* packet_fragmenter;

bluetooth::common::BidiQueueEnd<bluetooth::hci::AclPacketBuilder,
                                bluetooth::hci::AclPacketView>* hci_queue_end =
    nullptr;
static bluetooth::os::EnqueueBuffer<bluetooth::hci::AclPacketBuilder>*
    pending_data = nullptr;

namespace {
bool IsCommandStatusOpcode(bluetooth::hci::OpCode op_code) {
  switch (op_code) {
    case bluetooth::hci::OpCode::INQUIRY:
    case bluetooth::hci::OpCode::CREATE_CONNECTION:
    case bluetooth::hci::OpCode::DISCONNECT:
    case bluetooth::hci::OpCode::ACCEPT_CONNECTION_REQUEST:
    case bluetooth::hci::OpCode::REJECT_CONNECTION_REQUEST:
    case bluetooth::hci::OpCode::CHANGE_CONNECTION_PACKET_TYPE:
    case bluetooth::hci::OpCode::AUTHENTICATION_REQUESTED:
    case bluetooth::hci::OpCode::SET_CONNECTION_ENCRYPTION:
    case bluetooth::hci::OpCode::CHANGE_CONNECTION_LINK_KEY:
    case bluetooth::hci::OpCode::MASTER_LINK_KEY:
    case bluetooth::hci::OpCode::REMOTE_NAME_REQUEST:
    case bluetooth::hci::OpCode::READ_REMOTE_SUPPORTED_FEATURES:
    case bluetooth::hci::OpCode::READ_REMOTE_EXTENDED_FEATURES:
    case bluetooth::hci::OpCode::READ_REMOTE_VERSION_INFORMATION:
    case bluetooth::hci::OpCode::READ_CLOCK_OFFSET:
    case bluetooth::hci::OpCode::SETUP_SYNCHRONOUS_CONNECTION:
    case bluetooth::hci::OpCode::ACCEPT_SYNCHRONOUS_CONNECTION:
    case bluetooth::hci::OpCode::REJECT_SYNCHRONOUS_CONNECTION:
    case bluetooth::hci::OpCode::ENHANCED_SETUP_SYNCHRONOUS_CONNECTION:
    case bluetooth::hci::OpCode::ENHANCED_ACCEPT_SYNCHRONOUS_CONNECTION:
    case bluetooth::hci::OpCode::HOLD_MODE:
    case bluetooth::hci::OpCode::SNIFF_MODE:
    case bluetooth::hci::OpCode::EXIT_SNIFF_MODE:
    case bluetooth::hci::OpCode::QOS_SETUP:
    case bluetooth::hci::OpCode::SWITCH_ROLE:
    case bluetooth::hci::OpCode::FLOW_SPECIFICATION:
    case bluetooth::hci::OpCode::REFRESH_ENCRYPTION_KEY:
    case bluetooth::hci::OpCode::ENHANCED_FLUSH:
    case bluetooth::hci::OpCode::LE_CREATE_CONNECTION:
    case bluetooth::hci::OpCode::LE_CONNECTION_UPDATE:
    case bluetooth::hci::OpCode::LE_READ_REMOTE_FEATURES:
    case bluetooth::hci::OpCode::LE_START_ENCRYPTION:
    case bluetooth::hci::OpCode::LE_READ_LOCAL_P_256_PUBLIC_KEY_COMMAND:
    case bluetooth::hci::OpCode::LE_GENERATE_DHKEY_COMMAND:
    case bluetooth::hci::OpCode::LE_SET_PHY:
    case bluetooth::hci::OpCode::LE_EXTENDED_CREATE_CONNECTION:
    case bluetooth::hci::OpCode::LE_PERIODIC_ADVERTISING_CREATE_SYNC:
    case bluetooth::hci::OpCode::LE_ACCEPT_CIS_REQUEST:
    case bluetooth::hci::OpCode::LE_CREATE_BIG:
    case bluetooth::hci::OpCode::LE_TERMINATE_BIG:
    case bluetooth::hci::OpCode::LE_BIG_CREATE_SYNC:
    case bluetooth::hci::OpCode::LE_REQUEST_PEER_SCA:
    case bluetooth::hci::OpCode::LE_READ_REMOTE_TRANSMIT_POWER_LEVEL:

      return true;
    default:
      return false;
  }
}

bool is_valid_event_code(uint8_t event_code_raw) {
  auto event_code = static_cast<bluetooth::hci::EventCode>(event_code_raw);
  switch (event_code) {
    case bluetooth::hci::EventCode::INQUIRY_COMPLETE:
    case bluetooth::hci::EventCode::INQUIRY_RESULT:
    case bluetooth::hci::EventCode::CONNECTION_COMPLETE:
    case bluetooth::hci::EventCode::CONNECTION_REQUEST:
    case bluetooth::hci::EventCode::DISCONNECTION_COMPLETE:
    case bluetooth::hci::EventCode::AUTHENTICATION_COMPLETE:
    case bluetooth::hci::EventCode::REMOTE_NAME_REQUEST_COMPLETE:
    case bluetooth::hci::EventCode::ENCRYPTION_CHANGE:
    case bluetooth::hci::EventCode::CHANGE_CONNECTION_LINK_KEY_COMPLETE:
    case bluetooth::hci::EventCode::MASTER_LINK_KEY_COMPLETE:
    case bluetooth::hci::EventCode::READ_REMOTE_SUPPORTED_FEATURES_COMPLETE:
    case bluetooth::hci::EventCode::READ_REMOTE_VERSION_INFORMATION_COMPLETE:
    case bluetooth::hci::EventCode::QOS_SETUP_COMPLETE:
    case bluetooth::hci::EventCode::COMMAND_COMPLETE:
    case bluetooth::hci::EventCode::COMMAND_STATUS:
    case bluetooth::hci::EventCode::HARDWARE_ERROR:
    case bluetooth::hci::EventCode::FLUSH_OCCURRED:
    case bluetooth::hci::EventCode::ROLE_CHANGE:
    case bluetooth::hci::EventCode::NUMBER_OF_COMPLETED_PACKETS:
    case bluetooth::hci::EventCode::MODE_CHANGE:
    case bluetooth::hci::EventCode::RETURN_LINK_KEYS:
    case bluetooth::hci::EventCode::PIN_CODE_REQUEST:
    case bluetooth::hci::EventCode::LINK_KEY_REQUEST:
    case bluetooth::hci::EventCode::LINK_KEY_NOTIFICATION:
    case bluetooth::hci::EventCode::LOOPBACK_COMMAND:
    case bluetooth::hci::EventCode::DATA_BUFFER_OVERFLOW:
    case bluetooth::hci::EventCode::MAX_SLOTS_CHANGE:
    case bluetooth::hci::EventCode::READ_CLOCK_OFFSET_COMPLETE:
    case bluetooth::hci::EventCode::CONNECTION_PACKET_TYPE_CHANGED:
    case bluetooth::hci::EventCode::QOS_VIOLATION:
    case bluetooth::hci::EventCode::PAGE_SCAN_REPETITION_MODE_CHANGE:
    case bluetooth::hci::EventCode::FLOW_SPECIFICATION_COMPLETE:
    case bluetooth::hci::EventCode::INQUIRY_RESULT_WITH_RSSI:
    case bluetooth::hci::EventCode::READ_REMOTE_EXTENDED_FEATURES_COMPLETE:
    case bluetooth::hci::EventCode::SYNCHRONOUS_CONNECTION_COMPLETE:
    case bluetooth::hci::EventCode::SYNCHRONOUS_CONNECTION_CHANGED:
    case bluetooth::hci::EventCode::SNIFF_SUBRATING:
    case bluetooth::hci::EventCode::EXTENDED_INQUIRY_RESULT:
    case bluetooth::hci::EventCode::ENCRYPTION_KEY_REFRESH_COMPLETE:
    case bluetooth::hci::EventCode::IO_CAPABILITY_REQUEST:
    case bluetooth::hci::EventCode::IO_CAPABILITY_RESPONSE:
    case bluetooth::hci::EventCode::USER_CONFIRMATION_REQUEST:
    case bluetooth::hci::EventCode::USER_PASSKEY_REQUEST:
    case bluetooth::hci::EventCode::REMOTE_OOB_DATA_REQUEST:
    case bluetooth::hci::EventCode::SIMPLE_PAIRING_COMPLETE:
    case bluetooth::hci::EventCode::LINK_SUPERVISION_TIMEOUT_CHANGED:
    case bluetooth::hci::EventCode::ENHANCED_FLUSH_COMPLETE:
    case bluetooth::hci::EventCode::USER_PASSKEY_NOTIFICATION:
    case bluetooth::hci::EventCode::KEYPRESS_NOTIFICATION:
    case bluetooth::hci::EventCode::REMOTE_HOST_SUPPORTED_FEATURES_NOTIFICATION:
    case bluetooth::hci::EventCode::LE_META_EVENT:
    case bluetooth::hci::EventCode::NUMBER_OF_COMPLETED_DATA_BLOCKS:
    case bluetooth::hci::EventCode::VENDOR_SPECIFIC:
      return true;
  }
  return false;
};

static bool event_already_registered_in_hci_layer(
    bluetooth::hci::EventCode event_code) {
  switch (event_code) {
    case bluetooth::hci::EventCode::COMMAND_COMPLETE:
    case bluetooth::hci::EventCode::COMMAND_STATUS:
    case bluetooth::hci::EventCode::PAGE_SCAN_REPETITION_MODE_CHANGE:
    case bluetooth::hci::EventCode::MAX_SLOTS_CHANGE:
    case bluetooth::hci::EventCode::VENDOR_SPECIFIC:
      return true;
    case bluetooth::hci::EventCode::LE_META_EVENT:
    case bluetooth::hci::EventCode::DISCONNECTION_COMPLETE:
      return bluetooth::shim::is_gd_shim_enabled();
    default:
      return false;
  }
}

static bool event_already_registered_in_le_advertising_manager(
    bluetooth::hci::EventCode event_code) {
  for (auto event : bluetooth::hci::AclConnectionEvents) {
    if (event == event_code) {
      return bluetooth::shim::is_gd_advertising_enabled();
    }
  }
  return false;
}

std::unique_ptr<bluetooth::packet::RawBuilder> MakeUniquePacket(
    const uint8_t* data, size_t len) {
  bluetooth::packet::RawBuilder builder;
  std::vector<uint8_t> bytes(data, data + len);

  auto payload = std::make_unique<bluetooth::packet::RawBuilder>();
  payload->AddOctets(bytes);

  return payload;
}
}  // namespace

BT_HDR* WrapPacketAndCopy(
    uint16_t event,
    bluetooth::hci::PacketView<bluetooth::hci::kLittleEndian>* data) {
  size_t packet_size = data->size() + kBtHdrSize;
  BT_HDR* packet = reinterpret_cast<BT_HDR*>(osi_malloc(packet_size));
  packet->offset = 0;
  packet->len = data->size();
  packet->layer_specific = 0;
  packet->event = event;
  std::copy(data->begin(), data->end(), packet->data);
  return packet;
}

static void event_callback(bluetooth::hci::EventPacketView event_packet_view) {
  if (!send_data_upwards) {
    return;
  }
  send_data_upwards.Run(FROM_HERE, WrapPacketAndCopy(MSG_HC_TO_STACK_HCI_EVT,
                                                     &event_packet_view));
}

static void set_data_cb(
    base::Callback<void(const base::Location&, BT_HDR*)> send_data_cb) {
  send_data_upwards = std::move(send_data_cb);
}

void OnTransmitPacketCommandComplete(command_complete_cb complete_callback,
                                     void* context,
                                     bluetooth::hci::CommandCompleteView view) {
  LOG_DEBUG("Received cmd complete for %s",
            bluetooth::hci::OpCodeText(view.GetCommandOpCode()).c_str());
  std::vector<const uint8_t> data(view.begin(), view.end());
  BT_HDR* response = WrapPacketAndCopy(MSG_HC_TO_STACK_HCI_EVT, &view);
  complete_callback(response, context);
}

class OsiObject {
 public:
  OsiObject(void* ptr) : ptr_(ptr) {}
  ~OsiObject() {
    if (ptr_ != nullptr) {
      osi_free(ptr_);
    }
  }
  void* Release() {
    void* ptr = ptr_;
    ptr_ = nullptr;
    return ptr;
  }

 private:
  void* ptr_;
};

void OnTransmitPacketStatus(command_status_cb status_callback, void* context,
                            std::unique_ptr<OsiObject> command,
                            bluetooth::hci::CommandStatusView view) {
  LOG_DEBUG("Received cmd status %s for %s",
            bluetooth::hci::ErrorCodeText(view.GetStatus()).c_str(),
            bluetooth::hci::OpCodeText(view.GetCommandOpCode()).c_str());
  uint8_t status = static_cast<uint8_t>(view.GetStatus());
  status_callback(status, static_cast<BT_HDR*>(command->Release()), context);
}

using bluetooth::common::BindOnce;
using bluetooth::common::Unretained;

static void transmit_command(BT_HDR* command,
                             command_complete_cb complete_callback,
                             command_status_cb status_callback, void* context) {
  CHECK(command != nullptr);
  uint8_t* data = command->data + command->offset;
  size_t len = command->len;
  CHECK(len >= (kCommandOpcodeSize + kCommandLengthSize));

  // little endian command opcode
  uint16_t command_op_code = (data[1] << 8 | data[0]);
  // Gd stack API requires opcode specification and calculates length, so
  // no need to provide opcode or length here.
  data += (kCommandOpcodeSize + kCommandLengthSize);
  len -= (kCommandOpcodeSize + kCommandLengthSize);

  auto op_code = static_cast<const bluetooth::hci::OpCode>(command_op_code);

  auto payload = MakeUniquePacket(data, len);
  auto packet =
      bluetooth::hci::CommandPacketBuilder::Create(op_code, std::move(payload));

  LOG_INFO("Sending command %s", bluetooth::hci::OpCodeText(op_code).c_str());

  if (IsCommandStatusOpcode(op_code)) {
    auto command_unique = std::make_unique<OsiObject>(command);
    bluetooth::shim::GetHciLayer()->EnqueueCommand(
        std::move(packet), bluetooth::shim::GetGdShimHandler()->BindOnce(
                               OnTransmitPacketStatus, status_callback, context,
                               std::move(command_unique)));
  } else {
    bluetooth::shim::GetHciLayer()->EnqueueCommand(
        std::move(packet),
        bluetooth::shim::GetGdShimHandler()->BindOnce(
            OnTransmitPacketCommandComplete, complete_callback, context));
    osi_free(command);
  }
}

static void command_complete_callback(BT_HDR* response, void* context) {
  auto future = static_cast<future_t*>(context);
  future_ready(future, response);
}

static void command_status_callback(uint8_t status, BT_HDR* command,
                                    void* context) {
  LOG_ALWAYS_FATAL(
      "transmit_command_futured should only send command complete opcode");
}

static future_t* transmit_command_futured(BT_HDR* command) {
  future_t* future = future_new();
  transmit_command(command, command_complete_callback, command_status_callback,
                   future);
  return future;
}

static void transmit_fragment(BT_HDR* packet, bool send_transmit_finished) {
  // HCI command packets are freed on a different thread when the matching
  // event is received. Check packet->event before sending to avoid a race.
  bool free_after_transmit =
      (packet->event & MSG_EVT_MASK) != MSG_STACK_TO_HC_HCI_CMD &&
      send_transmit_finished;

  uint8_t* stream = packet->data + packet->offset;
  size_t length = packet->len;
  uint16_t handle_with_flags;
  STREAM_TO_UINT16(handle_with_flags, stream);
  auto pb_flag = static_cast<bluetooth::hci::PacketBoundaryFlag>(
      handle_with_flags >> 12 & 0b11);
  auto bc_flag =
      static_cast<bluetooth::hci::BroadcastFlag>(handle_with_flags >> 14);
  uint16_t handle = handle_with_flags & 0xEFF;
  length -= 2;
  // skip data total length
  stream += 2;
  length -= 2;
  auto payload = MakeUniquePacket(stream, length);
  auto acl_packet = bluetooth::hci::AclPacketBuilder::Create(
      handle, pb_flag, bc_flag, std::move(payload));
  pending_data->Enqueue(std::move(acl_packet),
                        bluetooth::shim::GetGdShimHandler());
  if (free_after_transmit) {
    osi_free(packet);
  }
}
static void dispatch_reassembled(BT_HDR* packet) {
  // Events should already have been dispatched before this point
  CHECK((packet->event & MSG_EVT_MASK) != MSG_HC_TO_STACK_HCI_EVT);
  CHECK(!send_data_upwards.is_null());
  send_data_upwards.Run(FROM_HERE, packet);
}
static void fragmenter_transmit_finished(BT_HDR* packet,
                                         bool all_fragments_sent) {
  if (all_fragments_sent) {
    osi_free(packet);
  } else {
    // This is kind of a weird case, since we're dispatching a partially sent
    // packet up to a higher layer.
    // TODO(zachoverflow): rework upper layer so this isn't necessary.
    send_data_upwards.Run(FROM_HERE, packet);
  }
}

static const packet_fragmenter_callbacks_t packet_fragmenter_callbacks = {
    transmit_fragment, dispatch_reassembled, fragmenter_transmit_finished};

static void transmit_downward(uint16_t type, void* raw_data) {
  bluetooth::shim::GetGdShimHandler()->Call(
      packet_fragmenter->fragment_and_dispatch, static_cast<BT_HDR*>(raw_data));
}

static hci_t interface = {.set_data_cb = set_data_cb,
                          .transmit_command = transmit_command,
                          .transmit_command_futured = transmit_command_futured,
                          .transmit_downward = transmit_downward};

const hci_t* bluetooth::shim::hci_layer_get_interface() {
  packet_fragmenter = packet_fragmenter_get_interface();
  packet_fragmenter->init(&packet_fragmenter_callbacks);
  return &interface;
}

static void acl_data_callback() {
  if (hci_queue_end == nullptr) {
    return;
  }
  auto packet = hci_queue_end->TryDequeue();
  ASSERT(packet != nullptr);
  if (!packet->IsValid()) {
    LOG_INFO("Dropping invalid packet of size %zu", packet->size());
    return;
  }
  if (!send_data_upwards) {
    return;
  }
  auto data = WrapPacketAndCopy(MSG_HC_TO_STACK_HCI_ACL, packet.get());
  packet_fragmenter->reassemble_and_dispatch(data);
}

void bluetooth::shim::hci_on_reset_complete() {
  ASSERT(send_data_upwards);
  for (uint8_t event_code_raw = 0; event_code_raw < 0xFF; event_code_raw++) {
    if (!is_valid_event_code(event_code_raw)) {
      continue;
    }
    auto event_code = static_cast<bluetooth::hci::EventCode>(event_code_raw);
    if (event_already_registered_in_hci_layer(event_code)) {
      continue;
    } else if (event_already_registered_in_le_advertising_manager(event_code)) {
      continue;
    }
    auto handler = bluetooth::shim::GetGdShimHandler();
    bluetooth::shim::GetHciLayer()->RegisterEventHandler(
        event_code, handler->Bind(event_callback));
  }

  hci_queue_end = bluetooth::shim::GetHciLayer()->GetAclQueueEnd();

  // if gd advertising enabled, hci_queue_end will be register in
  // AclManager::impl::Start
  if (!bluetooth::shim::is_gd_advertising_enabled()) {
    hci_queue_end->RegisterDequeue(bluetooth::shim::GetGdShimHandler(),
                                   bluetooth::common::Bind(acl_data_callback));
  }

  pending_data =
      new bluetooth::os::EnqueueBuffer<bluetooth::hci::AclPacketBuilder>(
          hci_queue_end);
}

void bluetooth::shim::hci_on_shutting_down() {
  if (pending_data != nullptr) {
    pending_data->Clear();
    delete pending_data;
    pending_data = nullptr;
  }
  if (hci_queue_end != nullptr) {
    if (!bluetooth::shim::is_gd_advertising_enabled()) {
      hci_queue_end->UnregisterDequeue();
    }
    for (uint8_t event_code_raw = 0; event_code_raw < 0xFF; event_code_raw++) {
      if (!is_valid_event_code(event_code_raw)) {
        continue;
      }
      auto event_code = static_cast<bluetooth::hci::EventCode>(event_code_raw);
      if (event_already_registered_in_hci_layer(event_code)) {
        continue;
      } else if (event_already_registered_in_le_advertising_manager(
                     event_code)) {
        continue;
      }
      bluetooth::shim::GetHciLayer()->UnregisterEventHandler(event_code);
    }
    hci_queue_end = nullptr;
  }
}
