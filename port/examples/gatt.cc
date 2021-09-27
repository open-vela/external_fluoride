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

struct bt_conn {
  sq_entry_t          node;
  struct bt_conn_info info;
};

static struct bt_conn_cb *g_conn_callback_list;
static sq_queue_t         g_conn_list;

const uint8_t kHRBodyLocationOther    = 0;
const uint8_t kHRBodyLocationChest    = 1;
const uint8_t kHRBodyLocationWrist    = 2;
const uint8_t kHRBodyLocationFinger   = 3;
const uint8_t kHRBodyLocationHand     = 4;
const uint8_t kHRBodyLocationEarLobe  = 5;
const uint8_t kHRBodyLocationFoot     = 6;

const uint8_t  kHRValueFormat8Bit       = (0 << 0);
const uint8_t  kHRValueFormat16Bit      = (1 << 0);
const uint16_t kHRSensorContactDetected = (3 << 1);
const uint8_t  kHREnergyExpendedPresent = (1 << 3);

static void btgattc_scan_result_cb(uint16_t event_type, uint8_t addr_type, RawAddress *bda,
    uint8_t primary_phy, uint8_t secondary_phy, uint8_t advertising_sid, int8_t tx_power,
    int8_t rssi, uint16_t periodic_adv_int, std::vector<uint8_t> adv_data)
{
  LOG_SAMPLES("Device found: %s (%s) (RSSI %d)\n",
      bda->ToString().c_str(), AddressTypeText(addr_type).c_str(), rssi);
}

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


static void btgatts_register_server_cb(int status,
    int server_if, const Uuid& uuid)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  std::vector<btgatt_db_element_t> svc;
  int i;

  if (uuid == flrd->sid && flrd->sif == 0) {
    flrd->sif = server_if;
    for (i = 0; i < flrd->element_size; i++)
      svc.push_back(flrd->element[i]);

    flrd->gatt->server->add_service(flrd->sif, svc);
  }
}

static void btgatts_connection_cb(int conn_id,
    int server_if, int connected, const RawAddress& bda)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  struct bt_conn_info *info;
  FAR sq_entry_t *entry;
  struct bt_conn_cb *cb;
  struct bt_conn *conn;
  bool found = false;
  bt_addr_le_t *dst;

  if (connected)
    {
      for (entry = sq_peek(&g_conn_list); entry; entry = sq_next(entry)) {
        conn = (struct bt_conn *)entry;
        if (conn->info.id == conn_id || !memcpy((void *)conn->info.le.dst->a.val,
              bda.address, sizeof(bda.address)))
          return;
      }

      conn = (struct bt_conn *)calloc(1, sizeof(*conn) + sizeof(bt_addr_le_t));
      if (conn == NULL)
        return;

      dst = (bt_addr_le_t *)(conn + 1);

      info       = &conn->info;
      info->id   = conn_id;
      info->role = BT_CONN_ROLE_MASTER;
      info->type = BT_CONN_TYPE_LE;

      dst->type = BT_ADDR_LE_PUBLIC;
      memcpy(dst->a.val, bda.address, sizeof(dst->a.val));
      info->le.dst = dst;

      for (cb = g_conn_callback_list; cb; cb = cb->_next) {
        if (cb->connected)
          cb->connected(conn, 0);
      }

      sq_addlast(&conn->node, &g_conn_list);
    }
  else
    {
      for (entry = sq_peek(&g_conn_list); entry; entry = sq_next(entry)) {
        conn = (struct bt_conn *)entry;
        if (conn->info.id == conn_id || !memcpy((void *)conn->info.le.dst->a.val,
              bda.address, sizeof(bda.address))) {
          sq_rem(&conn->node, &g_conn_list);
          found = true;
          break;
        }
      }

      if (found) {
        for (cb = g_conn_callback_list; cb; cb = cb->_next) {
          if (cb->disconnected)
            cb->disconnected(conn, 0);
        }
        free(conn);
      }

    }

  if (server_if == flrd->sif)
    flrd->cid = conn_id;
}

extern "C"
{

  int bt_conn_get_info(const struct bt_conn *conn,
                       struct bt_conn_info *info)
  {
    memcpy(info, &conn->info, sizeof(*info));
    return 0;
  }

  void bt_conn_cb_register(struct bt_conn_cb *cb)
  {
    cb->_next = g_conn_callback_list;
    g_conn_callback_list = cb;
  }

}

static void btgatts_service_added_cb(int status,
    int server_if, std::vector<btgatt_db_element_t> service)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  size_t i;

  for (i = 0; i < service.size(); i++) {
    const btgatt_db_element_t &curr = service[i];
    flrd->handle[i] = curr.attribute_handle;
  }
}

static void btgatts_schedule_measurement(void *ptr)
{
  struct fluoride_s *flrd = (struct fluoride_s *)ptr;
  uint8_t flags = kHRValueFormat8Bit | kHRSensorContactDetected;
  uint8_t data[6];

  if (flrd->ncount > 130 || flrd->ncount < 90)
    flrd->ncount = 90;

  if (!(flrd->ncount % 10)) {
    flags |= kHREnergyExpendedPresent;
    *(uint16_t*)&data[2] = flrd->ncount;
    *(uint16_t*)&data[4] = flrd->ncount >> 8;
  }

  data[0] = flags;
  data[1] = flrd->ncount;

  flrd->ncount++;

  std::vector<uint8_t> vec(data, data + 6);

  flrd->gatt->server->send_indication(flrd->sif, flrd->handle[1], flrd->cid, false, vec);
}

static void btgatts_request_write_descriptor_cb(int conn_id, int trans_id,
    const RawAddress& bda, int attr_handle, int offset,
    bool need_rsp, bool is_prep, std::vector<uint8_t> value)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  btgatt_response_t response = {};
  struct itimerspec its = {};

  if (conn_id != flrd->cid)
    return;

  response.handle = attr_handle;
  response.attr_value.handle = attr_handle;

  if (is_prep) {
    flrd->gatt->server->send_response(conn_id, trans_id,
        bluetooth::GATT_ERROR_REQUEST_NOT_SUPPORTED, response);
    return;
  }

  if (value.size() != 2 || value[1] != 0x00 || value[0] > 0x01) {
    flrd->gatt->server->send_response(conn_id, trans_id,
        bluetooth::GATT_ERROR_CCCD_IMPROPERLY_CONFIGURED, response);
    return;
  }

  if (value[0]) {
    its.it_value.tv_sec = 1;
    its.it_interval.tv_sec = 1;
  } else {
    its.it_value.tv_sec = 0;
    its.it_interval.tv_sec = 0;
  }

  timer_settime(flrd->timer, 0, &its, NULL);

  response.attr_value.offset = offset;

  if (need_rsp) {
    flrd->gatt->server->send_response(conn_id, trans_id,
        bluetooth::GATT_ERROR_NONE, response);
  }
}

static void btgatts_request_read_descriptor_cb(int conn_id, int trans_id,
    const RawAddress& bda, int attr_handle, int offset, bool is_long)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  btgatt_response_t response = {};
  std::vector<uint8_t> value;

  if (conn_id != flrd->cid)
    return;

  response.handle = attr_handle;
  response.attr_value.handle = attr_handle;
  response.attr_value.offset = offset;
  response.attr_value.len = 2;

  flrd->gatt->server->send_response(conn_id, trans_id,
      bluetooth::GATT_ERROR_NONE, response);
}

static void btgatts_request_read_characteristic_cb(int conn_id, int trans_id,
    const RawAddress& bda, int attr_handle, int offset, bool is_long)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  btgatt_response_t response = {};

  if (conn_id != flrd->cid)
    return;

  response.handle = attr_handle;
  response.attr_value.handle = attr_handle;
  response.attr_value.value[0] = kHRBodyLocationFoot;
  response.attr_value.offset = offset;
  response.attr_value.len = 1;

  flrd->gatt->server->send_response(conn_id, trans_id,
      bluetooth::GATT_ERROR_NONE, response);
}

static void btgatts_service_stopped_cb(int status, int server_if, int srvc_handle) TRACE_CALLBACK_BODY
static void btgatts_service_deleted_cb(int status, int server_if, int srvc_handle) TRACE_CALLBACK_BODY
static void btgatts_request_write_characteristic_cb(int conn_id, int trans_id, const RawAddress& bda, int attr_handle, int offset, bool need_rsp, bool is_prep, std::vector<uint8_t> value) TRACE_CALLBACK_BODY
static void btgatts_request_exec_write_cb(int conn_id, int trans_id, const RawAddress& bda, int exec_write) TRACE_CALLBACK_BODY
static void btgatts_response_confirmation_cb(int status, int handle) TRACE_CALLBACK_BODY
static void btgatts_indication_sent_cb(int conn_id, int status) TRACE_CALLBACK_BODY
static void btgatts_congestion_cb(int conn_id, bool congested) TRACE_CALLBACK_BODY
static void btgatts_mtu_changed_cb(int conn_id, int mtu) TRACE_CALLBACK_BODY
static void btgatts_phy_updated_cb(int conn_id, uint8_t tx_phy, uint8_t rx_phy, uint8_t status) TRACE_CALLBACK_BODY
static void btgatts_conn_updated_cb(int conn_id, uint16_t interval, uint16_t latency, uint16_t timeout, uint8_t status) TRACE_CALLBACK_BODY

static const btgatt_server_callbacks_t sGattServerCallbacks = {
  btgatts_register_server_cb,
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

class FlrdAdvertisingCallbacks : public AdvertisingCallbacks {
  public:
    void OnAdvertisingSetStarted(int reg_id, uint8_t advertiser_id, int8_t tx_power, uint8_t status) TRACE_CALLBACK_BODY
      void OnAdvertisingEnabled(uint8_t advertiser_id, bool enable, uint8_t status) TRACE_CALLBACK_BODY
      void OnAdvertisingDataSet(uint8_t advertiser_id, uint8_t status) TRACE_CALLBACK_BODY
      void OnScanResponseDataSet(uint8_t advertiser_id, uint8_t status) TRACE_CALLBACK_BODY
      void OnAdvertisingParametersUpdated(uint8_t advertiser_id, int8_t tx_power, uint8_t status) TRACE_CALLBACK_BODY
      void OnPeriodicAdvertisingParametersUpdated(uint8_t advertiser_id, uint8_t status) TRACE_CALLBACK_BODY
      void OnPeriodicAdvertisingDataSet(uint8_t advertiser_id, uint8_t status) TRACE_CALLBACK_BODY
      void OnPeriodicAdvertisingEnabled(uint8_t advertiser_id, bool enable, uint8_t status) TRACE_CALLBACK_BODY
      void OnOwnAddressRead(uint8_t advertiser_id, uint8_t address_type, RawAddress address) TRACE_CALLBACK_BODY
};

class FlrdScanningCallbacks : public ScanningCallbacks {
  public:
    void OnScannerRegistered(const Uuid app_uuid, uint8_t scannerId, uint8_t status) TRACE_CALLBACK_BODY
      void OnScanResult(uint16_t event_type, uint8_t addr_type, RawAddress* bda, uint8_t primary_phy, uint8_t secondary_phy,
          uint8_t advertising_sid, int8_t tx_power, int8_t rssi, uint16_t periodic_adv_int, std::vector<uint8_t> adv_data) TRACE_CALLBACK_BODY
      void OnTrackAdvFoundLost(btgatt_track_adv_info_t* p_adv_track_info) TRACE_CALLBACK_BODY
      void OnBatchScanReports(int client_if, int status, int report_format, int num_records, std::vector<uint8_t> data) TRACE_CALLBACK_BODY
};

static FlrdAdvertisingCallbacks sAdvertisingCallbacks;
static FlrdScanningCallbacks    sScanningCallbacks;

const btgatt_interface_t *bt_profile_gatt_init(struct fluoride_s *flrd)
{
  const btgatt_interface_t *gatt;
  struct sigevent sigevent = {};

  gatt = (const btgatt_interface_t *)
    flrd->interface->get_profile_interface(BT_PROFILE_GATT_ID);
  if (gatt == NULL)
    return gatt;

  sq_init(&g_conn_list);

  gatt->init(&sGattCallbacks);
  gatt->advertiser->RegisterCallbacks(static_cast<AdvertisingCallbacks *>(&sAdvertisingCallbacks));
  gatt->scanner->RegisterCallbacks(static_cast<ScanningCallbacks *>(&sScanningCallbacks));

  sigevent.sigev_value.sival_ptr = flrd;
  sigevent.sigev_notify = SIGEV_THREAD;
  sigevent.sigev_notify_function =
    (void (*)(union sigval))btgatts_schedule_measurement;
  timer_create(CLOCK_REALTIME, &sigevent, &flrd->timer);

  return gatt;
}
