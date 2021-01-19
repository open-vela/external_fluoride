/****************************************************************************
 *
 *   Copyright (C) 2021 Xiaomi InC. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "fluoride.h"

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

const btgatt_interface_t *bt_profile_gatt_init(struct fluoride_s *flrd)
{
  const btgatt_interface_t *gatt;

  gatt = (const btgatt_interface_t *)
    flrd->interface->get_profile_interface(BT_PROFILE_GATT_ID);
  if (gatt == NULL)
    return gatt;

  gatt->init(&sGattCallbacks);

  return gatt;
}
