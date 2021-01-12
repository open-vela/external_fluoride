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

#define LOG_TAG "bt_headless"

#include "main/shim/acl_api.h"
#include "main/shim/config.h"
#include "main/shim/shim.h"
#include "main/shim/controller.h"

#include "hardware/bluetooth.h"

#include "base/process/memory.h"
#include "base/allocator/allocator_shim.h"
#include "base/memory/shared_memory.h"
#include "base/memory/shared_memory_handle.h"
#include "base/debug/stack_trace.h"
#include "base/sys_info.h"

#include "btif_bqr.h"
#include "btif_a2dp_sink.h"
#include "avdt_api.h"
#include "hardware/bluetooth.h"
#include "hardware/bt_av.h"
#include "hardware/bt_gatt.h"
#include "hardware/bt_rc.h"
#include "hardware/bt_sdp.h"
#include "hardware/avrcp/avrcp_common.h"
#include "hardware/avrcp/avrcp.h"
#include "property.h"
#include "btif_util.h"

#define __TRACE_CALLBACK

#if defined(__TRACE_CALLBACK)
#define TRACE_CALLBACK_BODY { LOG_SAMPLES("UNIMPLEMENTED: %s: %d\n", __func__, __LINE__); }
#else
#define TRACE_CALLBACK_BODY { }
#endif

#define LOG_SAMPLES(fmt, args...) printf("CALLBACK " fmt, ##args)

using bluetooth::Uuid;
using namespace bluetooth::avrcp;

extern bt_interface_t                     bluetoothInterface;

static bt_interface_t                     *g_interface = &bluetoothInterface;
static ServiceInterface                   *g_avrcp;
static const btgatt_interface_t           *g_gatt;
static const btrc_ctrl_interface_t        *g_ctrl;
static const btav_sink_interface_t        *g_sink;
static const btav_source_interface_t      *g_source;
static const btsdp_interface_t            *g_sdp;

static std::mutex                         g_state_mutex;
static std::condition_variable            g_state_cond;
static bt_state_t                         g_state { BT_STATE_OFF };

static void adapter_state_changed(bt_state_t state) {
  std::unique_lock<std::mutex> lck(g_state_mutex);
  g_state = state;
  g_state_cond.notify_all();
}

static void parse_properties(int num_properties, bt_property_t *property)
{
  while (num_properties-- > 0)
  {
    switch (property->type)
    {
      case BT_PROPERTY_BDNAME:
        {
          const bt_bdname_t *name = property_as_name(property);
          if (name)
            LOG_SAMPLES("[%s]: name: %s\n", dump_property_type(property->type), name->name);
        }
        break;

      case BT_PROPERTY_BDADDR:
        {
          const RawAddress *addr = property_as_addr(property);
          LOG_SAMPLES("[%s]: addr: %02x:%02x:%02x:%02x:%02x:%02x\n",
              dump_property_type(property->type),
              addr->address[0], addr->address[1], addr->address[2],
              addr->address[3], addr->address[4], addr->address[5]);
        }
        break;

      case BT_PROPERTY_TYPE_OF_DEVICE:
        {
          bt_device_type_t device_type = property_as_device_type(property);
          if (device_type) {
            const struct {
              const char *device_type;
            } device_type_lookup[] = {
              { "Unknown" },
              { "Classic Only" },
              { "BLE Only" },
              { "Both Classic and BLE" },
            };
            int idx = (int)device_type;
            if (idx > BT_DEVICE_DEVTYPE_DUAL)
              idx = 0;
            LOG_SAMPLES("[%s]: type: %s\n", dump_property_type(property->type),
                device_type_lookup[idx].device_type);
          }
        }
        break;

      case BT_PROPERTY_CLASS_OF_DEVICE:
        {
          LOG_SAMPLES("[%s]: class: 0x%x\n", dump_property_type(property->type),
              device_class_to_int(property_as_device_class(property)));
        }
        break;

      case BT_PROPERTY_REMOTE_RSSI:
        {
          LOG_SAMPLES("[%s]: rssi: %d\n", dump_property_type(property->type),
              property_as_rssi(property));
        }
        break;
      default:
        break;
    }
    property++;
  }
}

static void adapter_properties(bt_status_t status, int num_properties, bt_property_t *properties)
{
  parse_properties(num_properties, properties);
}

static void remote_device_properties(bt_status_t status, RawAddress *bd_addr, int num_properties, bt_property_t *properties)
{
  parse_properties(num_properties, properties);
}

static void device_found(int num_properties, bt_property_t *properties) TRACE_CALLBACK_BODY
static void discovery_state_changed(bt_discovery_state_t state) TRACE_CALLBACK_BODY
static void pin_request(RawAddress *remote_bd_addr, bt_bdname_t *bd_name, uint32_t cod, bool min_16_digit) TRACE_CALLBACK_BODY

static void ssp_request(RawAddress *remote_bd_addr, bt_bdname_t *bd_name, uint32_t cod,
    bt_ssp_variant_t pairing_variant, uint32_t pass_key) {

  LOG_SAMPLES("%s: class: %u passkey: %x variant: %d\n", __func__, cod, pass_key, pairing_variant);

  g_interface->ssp_reply(remote_bd_addr, pairing_variant, true, pass_key);
}

static void bond_state_changed(bt_status_t status, RawAddress *remote_bd_addr, bt_bond_state_t state)
{
  RawAddress bd_addr;

  LOG_SAMPLES("%s: state: %d\n", __func__, state);
  if (state == BT_BOND_STATE_BONDED) {
    bd_addr.FromOctets(reinterpret_cast<const uint8_t*>(remote_bd_addr));
    g_sink->connect(bd_addr);
  }
}

static void acl_state_changed(bt_status_t status, RawAddress *remote_bd_addr, bt_acl_state_t state)
{
  LOG_SAMPLES("%s: state: %d, acl_state: %d\n", __func__, status, state);
}

static void thread_event(bt_cb_thread_evt evt) TRACE_CALLBACK_BODY
static void dut_mode_recv(uint16_t opcode, uint8_t *buf, uint8_t len) TRACE_CALLBACK_BODY
static void le_test_mode(bt_status_t status, uint16_t num_packets) TRACE_CALLBACK_BODY
static void energy_info(bt_activity_energy_info *energy_info, bt_uid_traffic_t *uid_data) TRACE_CALLBACK_BODY

static bt_callbacks_t bt_callbacks {
  .size = sizeof(bt_callbacks_t),
    .adapter_state_changed_cb     = adapter_state_changed,
    .adapter_properties_cb        = adapter_properties,
    .remote_device_properties_cb  = remote_device_properties,
    .device_found_cb              = device_found,
    .discovery_state_changed_cb   = discovery_state_changed,
    .pin_request_cb               = pin_request,
    .ssp_request_cb               = ssp_request,
    .bond_state_changed_cb        = bond_state_changed,
    .acl_state_changed_cb         = acl_state_changed,
    .thread_evt_cb                = thread_event,
    .dut_mode_recv_cb             = dut_mode_recv,
    .le_test_mode_cb              = le_test_mode,
    .energy_info_cb               = energy_info,
};

static void bta2dp_connection_state_callback(const RawAddress& bd_addr, btav_connection_state_t state)
{
  LOG_SAMPLES("%s: state: %d\n", __func__, state);
  if (state == BTAV_CONNECTION_STATE_DISCONNECTED)
    g_sink->disconnect(bd_addr);
}

static void bta2dp_audio_state_callback(const RawAddress& bd_addr, btav_audio_state_t state)
{
  LOG_SAMPLES("%s: state: %d\n", __func__, state);
}

static void bta2dp_audio_config_callback(const RawAddress& bd_addr, uint32_t sample_rate, uint8_t channel_count)
{
  LOG_SAMPLES("%s: sample_rate: %d, channel_count: %d\n", __func__, sample_rate, channel_count);
}

static btav_sink_callbacks_t sSinkBluetoothA2dpCallbacks = {
  sizeof(sSinkBluetoothA2dpCallbacks),
  bta2dp_connection_state_callback,
  bta2dp_audio_state_callback,
  bta2dp_audio_config_callback,
};

static void btgattc_scan_result_cb(uint16_t event_type, uint8_t addr_type, RawAddress *bda,
    uint8_t primary_phy, uint8_t secondary_phy, uint8_t advertising_sid, int8_t tx_power,
    int8_t rssi, uint16_t periodic_adv_int, std::vector<uint8_t> adv_data) TRACE_CALLBACK_BODY
static void btgattc_batchscan_reports_cb(int client_if, int status, int report_format, int num_records, std::vector<uint8_t> data) TRACE_CALLBACK_BODY
static void btgattc_batchscan_threshold_cb(int client_if) TRACE_CALLBACK_BODY
static void btgattc_track_adv_event_cb(btgatt_track_adv_info_t *p_adv_track_info) TRACE_CALLBACK_BODY

static const btgatt_scanner_callbacks_t sGattScannerCallbacks = {
  btgattc_scan_result_cb,
  btgattc_batchscan_reports_cb,
  btgattc_batchscan_threshold_cb,
  btgattc_track_adv_event_cb,
};

static void btgattc_register_app_cb(int status, int clientIf, const Uuid& app_uuid) TRACE_CALLBACK_BODY void btgattc_open_cb(int conn_id, int status, int clientIf, const RawAddress& bda) TRACE_CALLBACK_BODY
static void btgattc_close_cb(int conn_id, int status, int clientIf, const RawAddress& bda) TRACE_CALLBACK_BODY
static void btgattc_search_complete_cb(int conn_id, int status) TRACE_CALLBACK_BODY
static void btgattc_register_for_notification_cb(int conn_id, int registered, int status, uint16_t handle) TRACE_CALLBACK_BODY
static void btgattc_notify_cb(int conn_id, const btgatt_notify_params_t& p_data) TRACE_CALLBACK_BODY
static void btgattc_read_characteristic_cb(int conn_id, int status, btgatt_read_params_t *p_data) TRACE_CALLBACK_BODY
static void btgattc_write_characteristic_cb(int conn_id, int status, uint16_t handle) TRACE_CALLBACK_BODY
static void btgattc_execute_write_cb(int conn_id, int status) TRACE_CALLBACK_BODY
static void btgattc_read_descriptor_cb(int conn_id, int status, const btgatt_read_params_t& p_data) TRACE_CALLBACK_BODY
static void btgattc_write_descriptor_cb(int conn_id, int status, uint16_t handle) TRACE_CALLBACK_BODY
static void btgattc_remote_rssi_cb(int client_if, const RawAddress& bda, int rssi, int status) TRACE_CALLBACK_BODY
static void btgattc_configure_mtu_cb(int conn_id, int status, int mtu) TRACE_CALLBACK_BODY
static void btgattc_congestion_cb(int conn_id, bool congested) TRACE_CALLBACK_BODY
static void btgattc_get_gatt_db_cb(int conn_id, const btgatt_db_element_t *db, int count) TRACE_CALLBACK_BODY
static void btgattc_phy_updated_cb(int conn_id, uint8_t tx_phy, uint8_t rx_phy, uint8_t status) TRACE_CALLBACK_BODY
static void btgattc_conn_updated_cb(int conn_id, uint16_t interval, uint16_t latency, uint16_t timeout, uint8_t status) TRACE_CALLBACK_BODY

static const btgatt_client_callbacks_t sGattClientCallbacks = {
  btgattc_register_app_cb,
  btgattc_open_cb,
  btgattc_close_cb,
  btgattc_search_complete_cb,
  btgattc_register_for_notification_cb,
  btgattc_notify_cb,
  btgattc_read_characteristic_cb,
  btgattc_write_characteristic_cb,
  btgattc_read_descriptor_cb,
  btgattc_write_descriptor_cb,
  btgattc_execute_write_cb,
  btgattc_remote_rssi_cb,
  btgattc_configure_mtu_cb,
  btgattc_congestion_cb,
  btgattc_get_gatt_db_cb,
  NULL, /* services_removed_cb */
  NULL, /* services_added_cb */
  btgattc_phy_updated_cb,
  btgattc_conn_updated_cb
};

static void btgatts_register_app_cb(int status, int server_if, const Uuid& uuid) TRACE_CALLBACK_BODY
static void btgatts_connection_cb(int conn_id, int server_if, int connected, const RawAddress& bda) TRACE_CALLBACK_BODY
static void btgatts_service_added_cb(int status, int server_if, std::vector<btgatt_db_element_t> service) TRACE_CALLBACK_BODY
static void btgatts_service_stopped_cb(int status, int server_if, int srvc_handle) TRACE_CALLBACK_BODY
static void btgatts_service_deleted_cb(int status, int server_if, int srvc_handle) TRACE_CALLBACK_BODY
static void btgatts_request_read_characteristic_cb(int conn_id, int trans_id, const RawAddress& bda, int attr_handle, int offset, bool is_long) TRACE_CALLBACK_BODY
static void btgatts_request_read_descriptor_cb(int conn_id, int trans_id, const RawAddress& bda, int attr_handle, int offset, bool is_long) TRACE_CALLBACK_BODY
static void btgatts_request_write_characteristic_cb(int conn_id, int trans_id, const RawAddress& bda, int attr_handle, int offset, bool need_rsp, bool is_prep, std::vector<uint8_t> value) TRACE_CALLBACK_BODY
static void btgatts_request_write_descriptor_cb(int conn_id, int trans_id, const RawAddress& bda, int attr_handle, int offset, bool need_rsp, bool is_prep, std::vector<uint8_t> value) TRACE_CALLBACK_BODY
static void btgatts_request_exec_write_cb(int conn_id, int trans_id, const RawAddress& bda, int exec_write) TRACE_CALLBACK_BODY
static void btgatts_response_confirmation_cb(int status, int handle) TRACE_CALLBACK_BODY
static void btgatts_indication_sent_cb(int conn_id, int status) TRACE_CALLBACK_BODY
static void btgatts_congestion_cb(int conn_id, bool congested) TRACE_CALLBACK_BODY
static void btgatts_mtu_changed_cb(int conn_id, int mtu) TRACE_CALLBACK_BODY
static void btgatts_phy_updated_cb(int conn_id, uint8_t tx_phy, uint8_t rx_phy, uint8_t status) TRACE_CALLBACK_BODY
static void btgatts_conn_updated_cb(int conn_id, uint16_t interval, uint16_t latency, uint16_t timeout, uint8_t status) TRACE_CALLBACK_BODY

static const btgatt_server_callbacks_t sGattServerCallbacks = {
  btgatts_register_app_cb,
  btgatts_connection_cb,
  btgatts_service_added_cb,
  btgatts_service_stopped_cb,
  btgatts_service_deleted_cb,
  btgatts_request_read_characteristic_cb,
  btgatts_request_read_descriptor_cb,
  btgatts_request_write_characteristic_cb,
  btgatts_request_write_descriptor_cb,
  btgatts_request_exec_write_cb,
  btgatts_response_confirmation_cb,
  btgatts_indication_sent_cb,
  btgatts_congestion_cb,
  btgatts_mtu_changed_cb,
  btgatts_phy_updated_cb,
  btgatts_conn_updated_cb
};

static const btgatt_callbacks_t sGattCallbacks = {
  sizeof(btgatt_callbacks_t),
  &sGattClientCallbacks,
  &sGattServerCallbacks,
  &sGattScannerCallbacks,
};

static void btavrcp_passthrough_response_callback(const RawAddress& bd_addr, int id, int pressed) TRACE_CALLBACK_BODY
static void btavrcp_groupnavigation_response_callback(int id, int pressed) TRACE_CALLBACK_BODY
static void btavrcp_connection_state_callback(bool rc_connect, bool br_connect, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
static void btavrcp_get_rcfeatures_callback(const RawAddress& bd_addr, int features) TRACE_CALLBACK_BODY
static void btavrcp_setplayerapplicationsetting_rsp_callback( const RawAddress& bd_addr, uint8_t accepted) TRACE_CALLBACK_BODY
static void btavrcp_playerapplicationsetting_callback( const RawAddress& bd_addr, uint8_t num_attr, btrc_player_app_attr_t *app_attrs, uint8_t num_ext_attr, btrc_player_app_ext_attr_t *ext_attrs) TRACE_CALLBACK_BODY
static void btavrcp_playerapplicationsetting_changed_callback( const RawAddress& bd_addr, const btrc_player_settings_t& vals) TRACE_CALLBACK_BODY
static void btavrcp_set_abs_vol_cmd_callback(const RawAddress& bd_addr, uint8_t abs_vol, uint8_t label) TRACE_CALLBACK_BODY
static void btavrcp_register_notification_absvol_callback( const RawAddress& bd_addr, uint8_t label) TRACE_CALLBACK_BODY
static void btavrcp_track_changed_callback(const RawAddress& bd_addr, uint8_t num_attr, btrc_element_attr_val_t *p_attrs) TRACE_CALLBACK_BODY
static void btavrcp_play_position_changed_callback(const RawAddress& bd_addr, uint32_t song_len, uint32_t song_pos) TRACE_CALLBACK_BODY
static void btavrcp_play_status_changed_callback( const RawAddress& bd_addr, btrc_play_status_t play_status) TRACE_CALLBACK_BODY
static void btavrcp_get_folder_items_callback( const RawAddress& bd_addr, btrc_status_t status, const btrc_folder_items_t *folder_items, uint8_t count) TRACE_CALLBACK_BODY
static void btavrcp_change_path_callback(const RawAddress& bd_addr, uint32_t count) TRACE_CALLBACK_BODY
static void btavrcp_set_browsed_player_callback(const RawAddress& bd_addr, uint8_t num_items, uint8_t depth) TRACE_CALLBACK_BODY
static void btavrcp_set_addressed_player_callback(const RawAddress& bd_addr, uint8_t status) TRACE_CALLBACK_BODY
static void btavrcp_addressed_player_changed_callback(const RawAddress& bd_addr, uint16_t id) TRACE_CALLBACK_BODY
static void btavrcp_now_playing_content_changed_callback( const RawAddress& bd_addr) TRACE_CALLBACK_BODY
static void btavrcp_available_player_changed_callback ( const RawAddress& bd_addr) TRACE_CALLBACK_BODY
static void btavrcp_get_rcpsm_callback(const RawAddress& bd_addr, uint16_t psm) TRACE_CALLBACK_BODY

static btrc_ctrl_callbacks_t sBluetoothAvrcpCallbacks =
{
  sizeof(sBluetoothAvrcpCallbacks),
  btavrcp_passthrough_response_callback,
  btavrcp_groupnavigation_response_callback,
  btavrcp_connection_state_callback,
  btavrcp_get_rcfeatures_callback,
  btavrcp_setplayerapplicationsetting_rsp_callback,
  btavrcp_playerapplicationsetting_callback,
  btavrcp_playerapplicationsetting_changed_callback,
  btavrcp_set_abs_vol_cmd_callback,
  btavrcp_register_notification_absvol_callback,
  btavrcp_track_changed_callback,
  btavrcp_play_position_changed_callback,
  btavrcp_play_status_changed_callback,
  btavrcp_get_folder_items_callback,
  btavrcp_change_path_callback,
  btavrcp_set_browsed_player_callback,
  btavrcp_set_addressed_player_callback,
  btavrcp_addressed_player_changed_callback,
  btavrcp_now_playing_content_changed_callback,
  btavrcp_available_player_changed_callback,
  btavrcp_get_rcpsm_callback
};

class AvrcpMediaInterfaceImpl : public MediaInterface {
  public:
    void SendKeyEvent(uint8_t key, KeyState state) TRACE_CALLBACK_BODY
      void GetSongInfo(SongInfoCallback cb) override TRACE_CALLBACK_BODY
      void GetPlayStatus(PlayStatusCallback cb) override TRACE_CALLBACK_BODY
      void GetNowPlayingList(NowPlayingCallback cb) override TRACE_CALLBACK_BODY
      void GetMediaPlayerList(MediaListCallback cb) override TRACE_CALLBACK_BODY
      void GetFolderItems(uint16_t player_id, std::string media_id, FolderItemsCallback folder_cb) override TRACE_CALLBACK_BODY
      void SetBrowsedPlayer(uint16_t player_id, SetBrowsedPlayerCallback browse_cb) override TRACE_CALLBACK_BODY
      void RegisterUpdateCallback(MediaCallbacks *callback) override TRACE_CALLBACK_BODY
      void UnregisterUpdateCallback(MediaCallbacks *callback) override TRACE_CALLBACK_BODY
      void PlayItem(uint16_t player_id, bool now_playing, std::string media_id) override TRACE_CALLBACK_BODY
      void SetActiveDevice(const RawAddress& address) override TRACE_CALLBACK_BODY
};

class VolumeInterfaceImpl : public VolumeInterface {
  public:
    void DeviceConnected(const RawAddress& bdaddr) override TRACE_CALLBACK_BODY
    void DeviceConnected(const RawAddress& bdaddr, VolumeChangedCb cb) override TRACE_CALLBACK_BODY
    void DeviceDisconnected(const RawAddress& bdaddr) override TRACE_CALLBACK_BODY
    void SetVolume(int8_t volume) override TRACE_CALLBACK_BODY
};

static AvrcpMediaInterfaceImpl mAvrcpInterface;
static VolumeInterfaceImpl     mVolumeInterface;

static void sdp_search_callback(bt_status_t status, const RawAddress& bd_addr,
    const Uuid& uuid_in, int count, bluetooth_sdp_record* records) TRACE_CALLBACK_BODY

static btsdp_callbacks_t sBluetoothSdpCallbacks = {
  sizeof(sBluetoothSdpCallbacks),
  sdp_search_callback
};

extern "C" int main(int argc, char **argv)
{
  bt_os_callouts_t bt_os_callouts = { sizeof(bt_os_callouts_t) };
  bt_property_t *property;
  int status;

  status = g_interface->init(&bt_callbacks, false, false, 0, nullptr, false);
  if (status < 0)
    return status;

  status = g_interface->set_os_callouts(&bt_os_callouts);
  if (status < 0)
    return status;

  status = g_interface->enable();
  if (status < 0)
    return status;

  std::unique_lock<std::mutex> lck(g_state_mutex);
  while (g_state != BT_STATE_ON)
    g_state_cond.wait(lck);

  property = property_new_name(CONFIG_FLUORIDE_DEVICE_NAME);
  status = g_interface->set_adapter_property(property);
  property_free(property);
  if (status < 0)
    return status;

  g_gatt = (const btgatt_interface_t *)g_interface->get_profile_interface(BT_PROFILE_GATT_ID);
  if (g_gatt == NULL)
    return status;

  status = g_gatt->init(&sGattCallbacks);
  if (status < 0)
    return status;

  g_ctrl = (const btrc_ctrl_interface_t *)g_interface->get_profile_interface(BT_PROFILE_AV_RC_CTRL_ID);
  if (g_ctrl == NULL)
    return status;

  status = g_ctrl->init(&sBluetoothAvrcpCallbacks);
  if (status < 0)
    return status;

  g_sdp = (const btsdp_interface_t *)g_interface->get_profile_interface(BT_PROFILE_SDP_CLIENT_ID);
  if (g_sdp == NULL)
    return status;

  g_sdp->init(&sBluetoothSdpCallbacks);

  g_sink = (const btav_sink_interface_t *)g_interface->get_profile_interface(BT_PROFILE_ADVANCED_AUDIO_SINK_ID);
  if (g_sink == NULL)
    return status;

  g_sink->init(&sSinkBluetoothA2dpCallbacks);

  g_sink->set_audio_focus_state(BTIF_A2DP_SINK_FOCUS_GRANTED);

  g_avrcp = g_interface->get_avrcp_service();
  if (g_avrcp == NULL)
    return status;

  g_avrcp->Init(&mAvrcpInterface, &mVolumeInterface);

  property = property_new_scan_mode(BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
  status = g_interface->set_adapter_property(property);
  property_free(property);
  if (status < 0)
    return status;

  pause();

  return 0;
}
