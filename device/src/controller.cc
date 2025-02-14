/******************************************************************************
 *
 *  Copyright 2014 Google, Inc.
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

#define LOG_TAG "bt_controller"

#include "device/include/controller.h"

#include <base/logging.h>

#include "bt_types.h"
#include "btcore/include/event_mask.h"
#include "btcore/include/module.h"
#include "btcore/include/version.h"
#include "hcimsgs.h"
#include "main/shim/controller.h"
#include "main/shim/shim.h"
#include "osi/include/future.h"
#include "osi/include/properties.h"
#include "stack/include/btm_ble_api.h"

const bt_event_mask_t BLE_EVENT_MASK = {{0x00, 0x00, 0x00, 0x00, 0x7F, 0x02,
#if (BLE_PRIVACY_SPT == TRUE)
                                         0xFE,
#else
                                         /* Disable "LE Enhanced Connection
                                            Complete" when privacy is off */
                                         0xFC,
#endif
                                         0x7f}};

const bt_event_mask_t CLASSIC_EVENT_MASK = {HCI_DUMO_EVENT_MASK_EXT};

// TODO(zachoverflow): factor out into common module
const uint8_t SCO_HOST_BUFFER_SIZE = 0xff;

#define HCI_SUPPORTED_COMMANDS_ARRAY_SIZE 64
#define MAX_FEATURES_CLASSIC_PAGE_COUNT 3
#define BLE_SUPPORTED_STATES_SIZE 8
#define BLE_SUPPORTED_FEATURES_SIZE 8
#define MAX_LOCAL_SUPPORTED_CODECS_SIZE 8
#define LL_FEATURE_BIT_ISO_HOST_SUPPORT 32

static const hci_t* local_hci;
static const hci_packet_factory_t* packet_factory;
static const hci_packet_parser_t* packet_parser;

static RawAddress address;
static bt_version_t bt_version;

static uint8_t supported_commands[HCI_SUPPORTED_COMMANDS_ARRAY_SIZE];
static bt_device_features_t features_classic[MAX_FEATURES_CLASSIC_PAGE_COUNT];
static uint8_t last_features_classic_page_index;

static uint16_t acl_data_size_classic;
static uint16_t acl_data_size_ble;
static uint16_t iso_data_size;

static uint16_t acl_buffer_count_classic;
static uint8_t acl_buffer_count_ble;
static uint8_t iso_buffer_count;

static uint8_t ble_acceptlist_size;
static uint8_t ble_resolving_list_max_size;
static uint8_t ble_supported_states[BLE_SUPPORTED_STATES_SIZE];
static bt_device_features_t features_ble;
static uint16_t ble_suggested_default_data_length;
static uint16_t ble_supported_max_tx_octets;
static uint16_t ble_supported_max_tx_time;
static uint16_t ble_supported_max_rx_octets;
static uint16_t ble_supported_max_rx_time;

static uint16_t ble_maxium_advertising_data_length;
static uint8_t ble_number_of_supported_advertising_sets;
static uint8_t ble_periodic_advertiser_list_size;
static uint8_t local_supported_codecs[MAX_LOCAL_SUPPORTED_CODECS_SIZE];
static uint8_t number_of_local_supported_codecs = 0;

static bool readable;
static bool ble_supported;
static bool iso_supported;
static bool simple_pairing_supported;
static bool secure_connections_supported;

#if defined(CONFIG_FLUORIDE_BLE_ENABLED)
  #define FLUORIDE_BLE_ENABLED (1)
#else
  #define FLUORIDE_BLE_ENABLED (0)
#endif

#define AWAIT_COMMAND(command) \
  static_cast<BT_HDR*>(        \
      future_await(local_hci->transmit_command_futured(command)))

// Module lifecycle functions

static future_t* start_up(void) {
  BT_HDR* response;

  // Send the initial reset command
  response = AWAIT_COMMAND(packet_factory->make_reset());
  packet_parser->parse_generic_command_complete(response);

  // Request the classic buffer size next
  response = AWAIT_COMMAND(packet_factory->make_read_buffer_size());
  packet_parser->parse_read_buffer_size_response(
      response, &acl_data_size_classic, &acl_buffer_count_classic);

  // Tell the controller about our buffer sizes and buffer counts next
  // TODO(zachoverflow): factor this out. eww l2cap contamination. And why just
  // a hardcoded 10?
  response = AWAIT_COMMAND(packet_factory->make_host_buffer_size(
      L2CAP_MTU_SIZE, SCO_HOST_BUFFER_SIZE, L2CAP_HOST_FC_ACL_BUFS, 10));

  packet_parser->parse_generic_command_complete(response);

  // Read the local version info off the controller next, including
  // information such as manufacturer and supported HCI version
  response = AWAIT_COMMAND(packet_factory->make_read_local_version_info());
  packet_parser->parse_read_local_version_info_response(response, &bt_version);

  // Read the bluetooth address off the controller next
  response = AWAIT_COMMAND(packet_factory->make_read_bd_addr());
  packet_parser->parse_read_bd_addr_response(response, &address);

  // Request the controller's supported commands next
  response =
      AWAIT_COMMAND(packet_factory->make_read_local_supported_commands());
  packet_parser->parse_read_local_supported_commands_response(
      response, supported_commands, HCI_SUPPORTED_COMMANDS_ARRAY_SIZE);

  // Read page 0 of the controller features next
  uint8_t page_number = 0;
  response = AWAIT_COMMAND(
      packet_factory->make_read_local_extended_features(page_number));
  packet_parser->parse_read_local_extended_features_response(
      response, &page_number, &last_features_classic_page_index,
      features_classic, MAX_FEATURES_CLASSIC_PAGE_COUNT);

  CHECK(page_number == 0);
  page_number++;

  // Inform the controller what page 0 features we support, based on what
  // it told us it supports. We need to do this first before we request the
  // next page, because the controller's response for page 1 may be
  // dependent on what we configure from page 0
  simple_pairing_supported =
      HCI_SIMPLE_PAIRING_SUPPORTED(features_classic[0].as_array);
  if (simple_pairing_supported) {
    response = AWAIT_COMMAND(
        packet_factory->make_write_simple_pairing_mode(HCI_SP_MODE_ENABLED));
    packet_parser->parse_generic_command_complete(response);
  }

  if (HCI_LE_SPT_SUPPORTED(features_classic[0].as_array)) {
    uint8_t simultaneous_le_host =
        HCI_SIMUL_LE_BREDR_SUPPORTED(features_classic[0].as_array)
            ? BTM_BLE_SIMULTANEOUS_HOST
            : 0;
    response = AWAIT_COMMAND(packet_factory->make_ble_write_host_support(
        BTM_BLE_HOST_SUPPORT, simultaneous_le_host));

    packet_parser->parse_generic_command_complete(response);

    // If we modified the BT_HOST_SUPPORT, we will need ext. feat. page 1
    if (last_features_classic_page_index < 1)
      last_features_classic_page_index = 1;
  }

  // Done telling the controller about what page 0 features we support
  // Request the remaining feature pages
  while (page_number <= last_features_classic_page_index &&
         page_number < MAX_FEATURES_CLASSIC_PAGE_COUNT) {
    response = AWAIT_COMMAND(
        packet_factory->make_read_local_extended_features(page_number));
    packet_parser->parse_read_local_extended_features_response(
        response, &page_number, &last_features_classic_page_index,
        features_classic, MAX_FEATURES_CLASSIC_PAGE_COUNT);

    page_number++;
  }

#if (SC_MODE_INCLUDED == TRUE)
  secure_connections_supported =
      HCI_SC_CTRLR_SUPPORTED(features_classic[2].as_array);
  if (secure_connections_supported) {
    response = AWAIT_COMMAND(
        packet_factory->make_write_secure_connections_host_support(
            HCI_SC_MODE_ENABLED));
    packet_parser->parse_generic_command_complete(response);
  }
#endif

  ble_supported = last_features_classic_page_index >= 1 &&
                  HCI_LE_HOST_SUPPORTED(features_classic[1].as_array) &&
                  FLUORIDE_BLE_ENABLED;
  if (ble_supported) {
    // Request the ble acceptlist size next
    response = AWAIT_COMMAND(packet_factory->make_ble_read_acceptlist_size());
    packet_parser->parse_ble_read_acceptlist_size_response(
        response, &ble_acceptlist_size);

    // Request the ble supported features next
    response =
        AWAIT_COMMAND(packet_factory->make_ble_read_local_supported_features());
    packet_parser->parse_ble_read_local_supported_features_response(
        response, &features_ble);

    iso_supported = HCI_LE_CIS_CENTRAL(features_ble.as_array) ||
                    HCI_LE_CIS_PERIPHERAL(features_ble.as_array) ||
                    HCI_LE_ISO_BROADCASTER(features_ble.as_array);

    if (iso_supported) {
      // Request the ble buffer size next
      response = AWAIT_COMMAND(packet_factory->make_ble_read_buffer_size_v2());
      packet_parser->parse_ble_read_buffer_size_v2_response(
          response, &acl_data_size_ble, &acl_buffer_count_ble, &iso_data_size,
          &iso_buffer_count);

    } else {
      // Request the ble buffer size next
      response = AWAIT_COMMAND(packet_factory->make_ble_read_buffer_size());
      packet_parser->parse_ble_read_buffer_size_response(
          response, &acl_data_size_ble, &acl_buffer_count_ble);
    }

    // Response of 0 indicates ble has the same buffer size as classic
    if (acl_data_size_ble == 0) acl_data_size_ble = acl_data_size_classic;

    // Request the ble supported states next
    response = AWAIT_COMMAND(packet_factory->make_ble_read_supported_states());
    packet_parser->parse_ble_read_supported_states_response(
        response, ble_supported_states, sizeof(ble_supported_states));

    if (HCI_LE_ENHANCED_PRIVACY_SUPPORTED(features_ble.as_array)) {
      response =
          AWAIT_COMMAND(packet_factory->make_ble_read_resolving_list_size());
      packet_parser->parse_ble_read_resolving_list_size_response(
          response, &ble_resolving_list_max_size);
    }

    if (HCI_LE_DATA_LEN_EXT_SUPPORTED(features_ble.as_array)) {
      response =
          AWAIT_COMMAND(packet_factory->make_ble_read_maximum_data_length());
      packet_parser->parse_ble_read_maximum_data_length_response(
          response, &ble_supported_max_tx_octets, &ble_supported_max_tx_time,
          &ble_supported_max_rx_octets, &ble_supported_max_rx_time);

      response = AWAIT_COMMAND(
          packet_factory->make_ble_read_suggested_default_data_length());
      packet_parser->parse_ble_read_suggested_default_data_length_response(
          response, &ble_suggested_default_data_length);
    }

    if (HCI_LE_EXTENDED_ADVERTISING_SUPPORTED(features_ble.as_array)) {
      response = AWAIT_COMMAND(
          packet_factory->make_ble_read_maximum_advertising_data_length());
      packet_parser->parse_ble_read_maximum_advertising_data_length(
          response, &ble_maxium_advertising_data_length);

      response = AWAIT_COMMAND(
          packet_factory->make_ble_read_number_of_supported_advertising_sets());
      packet_parser->parse_ble_read_number_of_supported_advertising_sets(
          response, &ble_number_of_supported_advertising_sets);
    } else {
      /* If LE Excended Advertising is not supported, use the default value */
      ble_maxium_advertising_data_length = 31;
    }

    if (HCI_LE_PERIODIC_ADVERTISING_SUPPORTED(features_ble.as_array)) {
      response = AWAIT_COMMAND(
          packet_factory->make_ble_read_periodic_advertiser_list_size());

      packet_parser->parse_ble_read_size_of_advertiser_list(
          response, &ble_periodic_advertiser_list_size);
    }

    // Set the ble event mask next
    response =
        AWAIT_COMMAND(packet_factory->make_ble_set_event_mask(&BLE_EVENT_MASK));
    packet_parser->parse_generic_command_complete(response);

    if (HCI_LE_SET_HOST_FEATURE_SUPPORTED(supported_commands)) {
      response = AWAIT_COMMAND(packet_factory->make_ble_set_host_features(
          LL_FEATURE_BIT_ISO_HOST_SUPPORT, 0x01));
      packet_parser->parse_generic_command_complete(response);
    }
  }

  if (simple_pairing_supported) {
    response =
        AWAIT_COMMAND(packet_factory->make_set_event_mask(&CLASSIC_EVENT_MASK));
    packet_parser->parse_generic_command_complete(response);
  }

  // read local supported codecs
  if (HCI_READ_LOCAL_CODECS_SUPPORTED(supported_commands)) {
    response =
        AWAIT_COMMAND(packet_factory->make_read_local_supported_codecs());
    packet_parser->parse_read_local_supported_codecs_response(
        response, &number_of_local_supported_codecs, local_supported_codecs);
  }

  if (!HCI_READ_ENCR_KEY_SIZE_SUPPORTED(supported_commands)) {
    LOG(FATAL) << " Controller must support Read Encryption Key Size command";
  }

  readable = true;
  return future_new_immediate(FUTURE_SUCCESS);
}

static future_t* shut_down(void) {
  readable = false;
  return future_new_immediate(FUTURE_SUCCESS);
}

EXPORT_SYMBOL extern const module_t controller_module = {
    .name = CONTROLLER_MODULE,
    .init = NULL,
    .start_up = start_up,
    .shut_down = shut_down,
    .clean_up = NULL,
    .dependencies = {HCI_MODULE, NULL}};

// Interface functions

static bool get_is_ready(void) { return readable; }

static const RawAddress* get_address(void) {
  CHECK(readable);
  return &address;
}

static const bt_version_t* get_bt_version(void) {
  CHECK(readable);
  return &bt_version;
}

static uint8_t* get_local_supported_codecs(uint8_t* number_of_codecs) {
  CHECK(readable);
  if (number_of_local_supported_codecs) {
    *number_of_codecs = number_of_local_supported_codecs;
    return local_supported_codecs;
  }
  return NULL;
}

static const uint8_t* get_ble_supported_states(void) {
  CHECK(readable);
  return ble_supported_states;
}

static bool supports_simple_pairing(void) {
  CHECK(readable);
  return simple_pairing_supported;
}

static bool supports_secure_connections(void) {
  CHECK(readable);
  return secure_connections_supported;
}

static bool supports_simultaneous_le_bredr(void) {
  CHECK(readable);
  return HCI_SIMUL_LE_BREDR_SUPPORTED(features_classic[0].as_array);
}

static bool supports_reading_remote_extended_features(void) {
  CHECK(readable);
  return HCI_READ_REMOTE_EXT_FEATURES_SUPPORTED(supported_commands);
}

static bool supports_interlaced_inquiry_scan(void) {
  CHECK(readable);
  return HCI_LMP_INTERLACED_INQ_SCAN_SUPPORTED(features_classic[0].as_array);
}

static bool supports_rssi_with_inquiry_results(void) {
  CHECK(readable);
  return HCI_LMP_INQ_RSSI_SUPPORTED(features_classic[0].as_array);
}

static bool supports_extended_inquiry_response(void) {
  CHECK(readable);
  return HCI_EXT_INQ_RSP_SUPPORTED(features_classic[0].as_array);
}

static bool supports_central_peripheral_role_switch(void) {
  CHECK(readable);
  return HCI_SWITCH_SUPPORTED(features_classic[0].as_array);
}

static bool supports_enhanced_setup_synchronous_connection(void) {
  assert(readable);
  return HCI_ENH_SETUP_SYNCH_CONN_SUPPORTED(supported_commands);
}

static bool supports_enhanced_accept_synchronous_connection(void) {
  assert(readable);
  return HCI_ENH_ACCEPT_SYNCH_CONN_SUPPORTED(supported_commands);
}

static bool supports_3_slot_packets(void) {
  CHECK(readable);
  return HCI_3_SLOT_PACKETS_SUPPORTED(features_classic[0].as_array);
}

static bool supports_5_slot_packets(void) {
  CHECK(readable);
  return HCI_5_SLOT_PACKETS_SUPPORTED(features_classic[0].as_array);
}

static bool supports_classic_2m_phy(void) {
  CHECK(readable);
  return HCI_EDR_ACL_2MPS_SUPPORTED(features_classic[0].as_array);
}

static bool supports_classic_3m_phy(void) {
  CHECK(readable);
  return HCI_EDR_ACL_3MPS_SUPPORTED(features_classic[0].as_array);
}

static bool supports_3_slot_edr_packets(void) {
  CHECK(readable);
  return HCI_3_SLOT_EDR_ACL_SUPPORTED(features_classic[0].as_array);
}

static bool supports_5_slot_edr_packets(void) {
  CHECK(readable);
  return HCI_5_SLOT_EDR_ACL_SUPPORTED(features_classic[0].as_array);
}

static bool supports_sco(void) {
  CHECK(readable);
  return HCI_SCO_LINK_SUPPORTED(features_classic[0].as_array);
}

static bool supports_hv2_packets(void) {
  CHECK(readable);
  return HCI_HV2_PACKETS_SUPPORTED(features_classic[0].as_array);
}

static bool supports_hv3_packets(void) {
  CHECK(readable);
  return HCI_HV3_PACKETS_SUPPORTED(features_classic[0].as_array);
}

static bool supports_ev3_packets(void) {
  CHECK(readable);
  return HCI_ESCO_EV3_SUPPORTED(features_classic[0].as_array);
}

static bool supports_ev4_packets(void) {
  CHECK(readable);
  return HCI_ESCO_EV4_SUPPORTED(features_classic[0].as_array);
}

static bool supports_ev5_packets(void) {
  CHECK(readable);
  return HCI_ESCO_EV5_SUPPORTED(features_classic[0].as_array);
}

static bool supports_esco_2m_phy(void) {
  CHECK(readable);
  return HCI_EDR_ESCO_2MPS_SUPPORTED(features_classic[0].as_array);
}

static bool supports_esco_3m_phy(void) {
  CHECK(readable);
  return HCI_EDR_ESCO_3MPS_SUPPORTED(features_classic[0].as_array);
}

static bool supports_3_slot_esco_edr_packets(void) {
  CHECK(readable);
  return HCI_3_SLOT_EDR_ESCO_SUPPORTED(features_classic[0].as_array);
}

static bool supports_role_switch(void) {
  CHECK(readable);
  return HCI_SWITCH_SUPPORTED(features_classic[0].as_array);
}

static bool supports_hold_mode(void) {
  CHECK(readable);
  return HCI_HOLD_MODE_SUPPORTED(features_classic[0].as_array);
}

static bool supports_sniff_mode(void) {
  CHECK(readable);
  return HCI_SNIFF_MODE_SUPPORTED(features_classic[0].as_array);
}

static bool supports_park_mode(void) {
  CHECK(readable);
  return HCI_PARK_MODE_SUPPORTED(features_classic[0].as_array);
}

static bool supports_non_flushable_pb(void) {
  CHECK(readable);
  return HCI_NON_FLUSHABLE_PB_SUPPORTED(features_classic[0].as_array);
}

static bool supports_sniff_subrating(void) {
  CHECK(readable);
  return HCI_SNIFF_SUB_RATE_SUPPORTED(features_classic[0].as_array);
}

static bool supports_encryption_pause(void) {
  CHECK(readable);
  return HCI_ATOMIC_ENCRYPT_SUPPORTED(features_classic[0].as_array);
}

static bool supports_ble(void) {
  CHECK(readable);
  return ble_supported;
}

static bool supports_ble_privacy(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_ENHANCED_PRIVACY_SUPPORTED(features_ble.as_array);
}

static bool supports_ble_set_privacy_mode() {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_ENHANCED_PRIVACY_SUPPORTED(features_ble.as_array) &&
         HCI_LE_SET_PRIVACY_MODE_SUPPORTED(supported_commands);
}

static bool supports_ble_packet_extension(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_DATA_LEN_EXT_SUPPORTED(features_ble.as_array);
}

static bool supports_ble_connection_parameters_request(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_CONN_PARAM_REQ_SUPPORTED(features_ble.as_array);
}

static bool supports_ble_2m_phy(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_2M_PHY_SUPPORTED(features_ble.as_array);
}

static bool supports_ble_coded_phy(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_CODED_PHY_SUPPORTED(features_ble.as_array);
}

static bool supports_ble_extended_advertising(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_EXTENDED_ADVERTISING_SUPPORTED(features_ble.as_array);
}

static bool supports_ble_periodic_advertising(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_PERIODIC_ADVERTISING_SUPPORTED(features_ble.as_array);
}

static bool supports_ble_peripheral_initiated_feature_exchange(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_PERIPHERAL_INIT_FEAT_EXC_SUPPORTED(features_ble.as_array);
}

static bool supports_ble_connection_parameter_request(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_CONN_PARAM_REQ_SUPPORTED(features_ble.as_array);
}

static bool supports_ble_periodic_advertising_sync_transfer_sender(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_PERIODIC_ADVERTISING_SYNC_TRANSFER_SENDER(
      features_ble.as_array);
}

static bool supports_ble_periodic_advertising_sync_transfer_recipient(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_PERIODIC_ADVERTISING_SYNC_TRANSFER_RECIPIENT(
      features_ble.as_array);
}

static bool supports_ble_connected_isochronous_stream_central(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_CIS_CENTRAL(features_ble.as_array);
}

static bool supports_ble_connected_isochronous_stream_peripheral(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_CIS_PERIPHERAL(features_ble.as_array);
}

static bool supports_ble_isochronous_broadcaster(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_ISO_BROADCASTER(features_ble.as_array);
}

static bool supports_ble_synchronized_receiver(void) {
  CHECK(readable);
  if (!ble_supported) return false;
  return HCI_LE_SYNCHRONIZED_RECEIVER(features_ble.as_array);
}

static uint16_t get_acl_data_size_classic(void) {
  CHECK(readable);
  return acl_data_size_classic;
}

static uint16_t get_acl_data_size_ble(void) {
  CHECK(readable);
  if (!ble_supported) return 0;
  return acl_data_size_ble;
}

static uint16_t get_iso_data_size(void) {
  CHECK(readable);
  return iso_data_size;
}

static uint16_t get_acl_packet_size_classic(void) {
  CHECK(readable);
  return acl_data_size_classic + HCI_DATA_PREAMBLE_SIZE;
}

static uint16_t get_acl_packet_size_ble(void) {
  CHECK(readable);
  return acl_data_size_ble + HCI_DATA_PREAMBLE_SIZE;
}

static uint16_t get_iso_packet_size(void) {
  CHECK(readable);
  return iso_data_size + HCI_DATA_PREAMBLE_SIZE;
}

static uint16_t get_ble_suggested_default_data_length(void) {
  CHECK(readable);
  if (!ble_supported) return 0;
  return ble_suggested_default_data_length;
}

static uint16_t get_ble_maximum_tx_data_length(void) {
  CHECK(readable);
  if (!ble_supported) return 0;
  return ble_supported_max_tx_octets;
}

static uint16_t get_ble_maxium_advertising_data_length(void) {
  CHECK(readable);
  if (!ble_supported) return 0;
  return ble_maxium_advertising_data_length;
}

static uint8_t get_ble_number_of_supported_advertising_sets(void) {
  CHECK(readable);
  if (!ble_supported) return 0;
  return ble_number_of_supported_advertising_sets;
}

static uint8_t get_ble_periodic_advertiser_list_size(void) {
  CHECK(readable);
  if (!ble_supported) return 0;
  return ble_periodic_advertiser_list_size;
}

static uint16_t get_acl_buffer_count_classic(void) {
  CHECK(readable);
  return acl_buffer_count_classic;
}

static uint8_t get_acl_buffer_count_ble(void) {
  CHECK(readable);
  if (!ble_supported) return 0;
  return acl_buffer_count_ble;
}

static uint8_t get_iso_buffer_count(void) {
  CHECK(readable);
  if (!ble_supported) return 0;
  return iso_buffer_count;
}

static uint8_t get_ble_acceptlist_size(void) {
  CHECK(readable);
  if (!ble_supported) return 0;
  return ble_acceptlist_size;
}

static uint8_t get_ble_resolving_list_max_size(void) {
  CHECK(readable);
  if (!ble_supported) return 0;
  return ble_resolving_list_max_size;
}

static void set_ble_resolving_list_max_size(int resolving_list_max_size) {
  // Setting "resolving_list_max_size" to 0 is done during cleanup,
  // hence we ignore the "readable" flag already set to false during shutdown.
  if (resolving_list_max_size != 0) {
    CHECK(readable);
  }
  if (!ble_supported) return;
  ble_resolving_list_max_size = resolving_list_max_size;
}

static uint8_t get_le_all_initiating_phys() {
  uint8_t phy = PHY_LE_1M;
  // TODO(jpawlowski): uncomment after next FW udpate
  // if (supports_ble_2m_phy()) phy |= PHY_LE_2M;
  // if (supports_ble_coded_phy()) phy |= PHY_LE_CODED;
  return phy;
}

static const controller_t interface = {
    get_is_ready,

    get_address,
    get_bt_version,

    get_ble_supported_states,

    supports_simple_pairing,
    supports_secure_connections,
    supports_simultaneous_le_bredr,
    supports_reading_remote_extended_features,
    supports_interlaced_inquiry_scan,
    supports_rssi_with_inquiry_results,
    supports_extended_inquiry_response,
    supports_central_peripheral_role_switch,
    supports_enhanced_setup_synchronous_connection,
    supports_enhanced_accept_synchronous_connection,
    supports_3_slot_packets,
    supports_5_slot_packets,
    supports_classic_2m_phy,
    supports_classic_3m_phy,
    supports_3_slot_edr_packets,
    supports_5_slot_edr_packets,
    supports_sco,
    supports_hv2_packets,
    supports_hv3_packets,
    supports_ev3_packets,
    supports_ev4_packets,
    supports_ev5_packets,
    supports_esco_2m_phy,
    supports_esco_3m_phy,
    supports_3_slot_esco_edr_packets,
    supports_role_switch,
    supports_hold_mode,
    supports_sniff_mode,
    supports_park_mode,
    supports_non_flushable_pb,
    supports_sniff_subrating,
    supports_encryption_pause,

    supports_ble,
    supports_ble_packet_extension,
    supports_ble_connection_parameters_request,
    supports_ble_privacy,
    supports_ble_set_privacy_mode,
    supports_ble_2m_phy,
    supports_ble_coded_phy,
    supports_ble_extended_advertising,
    supports_ble_periodic_advertising,
    supports_ble_peripheral_initiated_feature_exchange,
    supports_ble_connection_parameter_request,
    supports_ble_periodic_advertising_sync_transfer_sender,
    supports_ble_periodic_advertising_sync_transfer_recipient,
    supports_ble_connected_isochronous_stream_central,
    supports_ble_connected_isochronous_stream_peripheral,
    supports_ble_isochronous_broadcaster,
    supports_ble_synchronized_receiver,

    get_acl_data_size_classic,
    get_acl_data_size_ble,
    get_iso_data_size,

    get_acl_packet_size_classic,
    get_acl_packet_size_ble,
    get_iso_packet_size,

    get_ble_suggested_default_data_length,
    get_ble_maximum_tx_data_length,
    get_ble_maxium_advertising_data_length,
    get_ble_number_of_supported_advertising_sets,
    get_ble_periodic_advertiser_list_size,

    get_acl_buffer_count_classic,
    get_acl_buffer_count_ble,
    get_iso_buffer_count,

    get_ble_acceptlist_size,

    get_ble_resolving_list_max_size,
    set_ble_resolving_list_max_size,
    get_local_supported_codecs,
    get_le_all_initiating_phys};

static const controller_t* controller_get_interface_legacy() {
  static bool loaded = false;
  if (!loaded) {
    loaded = true;

    local_hci = hci_layer_get_interface();
    packet_factory = hci_packet_factory_get_interface();
    packet_parser = hci_packet_parser_get_interface();
  }

  return &interface;
}

const controller_t* controller_get_interface() {
  if (bluetooth::shim::is_gd_controller_enabled()) {
    return bluetooth::shim::controller_get_interface();
  } else {
    return controller_get_interface_legacy();
  }
}

const controller_t* controller_get_test_interface(
    const hci_t* hci_interface,
    const hci_packet_factory_t* packet_factory_interface,
    const hci_packet_parser_t* packet_parser_interface) {
  local_hci = hci_interface;
  packet_factory = packet_factory_interface;
  packet_parser = packet_parser_interface;
  return &interface;
}
