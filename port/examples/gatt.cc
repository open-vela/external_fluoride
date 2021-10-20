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
#include "gatt_api.h"

struct bt_conn
{
  sq_entry_t                 node;
  struct bt_conn_info        info;
  int                        tid;
  uint16_t                   mtu;
};

struct _bt_gatt_service
{
  sq_entry_t                 node;
  bluetooth::Uuid            uuid;
  int                        handle;
  int                        id;
  struct bt_gatt_service    *servs;
};

static struct bt_conn_cb    *g_conn_callback_list;
static sq_queue_t            g_conn_list;
static sq_queue_t            g_gatts_callback_list;
pthread_mutex_t              g_service_mutex = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
pthread_cond_t               g_service_cond = PTHREAD_COND_INITIALIZER;

static void btgattc_scan_result_cb(uint16_t event_type, uint8_t addr_type,
                                   RawAddress *bda, uint8_t primary_phy,
                                   uint8_t secondary_phy, uint8_t advertising_sid,
                                   int8_t tx_power, int8_t rssi,
                                   uint16_t periodic_adv_int,
                                   std::vector<uint8_t> adv_data)
{
  LOG_SAMPLES("Device found: %s (%s) (RSSI %d)\n",
      bda->ToString().c_str(), AddressTypeText(addr_type).c_str(), rssi);
}

static void btgattc_batchscan_reports_cb(int client_if, int status, int report_format,
                                         int num_records, std::vector<uint8_t> data) TRACE_CALLBACK_BODY
static void btgattc_batchscan_threshold_cb(int client_if) TRACE_CALLBACK_BODY
static void btgattc_track_adv_event_cb(btgatt_track_adv_info_t *p_adv_track_info) TRACE_CALLBACK_BODY

static const btgatt_scanner_callbacks_t sGattScannerCallbacks = {
  btgattc_scan_result_cb,
  btgattc_batchscan_reports_cb,
  btgattc_batchscan_threshold_cb,
  btgattc_track_adv_event_cb,
};

static void btgattc_register_app_cb(int status, int clientIf, const Uuid& app_uuid) TRACE_CALLBACK_BODY
static void btgattc_open_cb(int conn_id, int status, int clientIf, const RawAddress& bda) TRACE_CALLBACK_BODY
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

static const btgatt_client_callbacks_t sGattClientCallbacks =
{
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

static Uuid bt_gatt_conv_uuid(struct bt_uuid *u)
{
  struct bt_uuid_128 *u128;
  struct bt_uuid_32 *u32;
  struct bt_uuid_16 *u16;

  if (u->type == BT_UUID_TYPE_16) {
    u16 = (struct bt_uuid_16 *)u;
    return Uuid::From16Bit(u16->val);
  } else if (u->type == BT_UUID_TYPE_32) {
    u32 = (struct bt_uuid_32 *)u;
    return Uuid::From32Bit(u32->val);
  } else if (u->type == BT_UUID_TYPE_128) {
    u128 = (struct bt_uuid_128 *)u;
    return Uuid::From128BitBE(u128->val);
  }

  return Uuid::kEmpty;
}

static bt_gatt_db_attribute_type_t bt_gatt_conv_attr(uint16_t u16)
{
  if (u16 == BT_UUID_GATT_PRIMARY_VAL)
    return BTGATT_DB_PRIMARY_SERVICE;
  else if (u16 == BT_UUID_GATT_CHRC_VAL)
    return BTGATT_DB_CHARACTERISTIC;
  else if (u16 == BT_UUID_GATT_SECONDARY_VAL)
    return BTGATT_DB_SECONDARY_SERVICE;
  else if (u16 == BT_UUID_GATT_INCLUDE_VAL)
    return BTGATT_DB_INCLUDED_SERVICE;
  else if (u16 == BT_UUID_GATT_CCC_VAL)
    return BTGATT_DB_DESCRIPTOR;

  return BTGATT_DB_PRIMARY_SERVICE;
}

static struct _bt_gatt_service *
btgatt_service_find(const bluetooth::Uuid *uuid, int handle,
                    int id, struct bt_gatt_service *servs)
{
  struct _bt_gatt_service *serv;
  FAR sq_entry_t *entry;

  for (entry = sq_peek(&g_gatts_callback_list);
       entry; entry = sq_next(entry)) {
    serv = (struct _bt_gatt_service *)entry;
    if (handle > 0 && serv->handle == handle)
      return serv;
    else if (id > 0 && serv->id == id)
      return serv;
    else if (servs != NULL && serv->servs == servs)
      return serv;
    else if (*uuid != Uuid::kEmpty && *uuid == serv->uuid)
      return serv;
  }

  return NULL;
}

static struct _bt_gatt_service *
btgatt_service_find_by_uuid(const bluetooth::Uuid &uuid)
{
  return btgatt_service_find(&uuid, -1, -1, NULL);
}

static struct _bt_gatt_service *
btgatt_service_find_by_handle(int handle)
{
  return btgatt_service_find(&Uuid::kEmpty, handle, -1, NULL);
}

static struct _bt_gatt_service *
btgatt_service_find_by_id(int id)
{
  return btgatt_service_find(&Uuid::kEmpty, -1, id, NULL);
}

static struct _bt_gatt_service *
btgatt_service_find_by_servs(struct bt_gatt_service *servs)
{
  return btgatt_service_find(&Uuid::kEmpty, -1, -1, servs);
}

static void bt_gatt_service_insert(std::vector<btgatt_db_element_t> *svcs,
                                   btgatt_db_element_t *elem)
{
  svcs->push_back(*elem);
  memset(elem, 0x0, sizeof(*elem));
}

static uint16_t convert_permissions(uint8_t perm)
{
  uint16_t permissions = 0;

  if (perm & BT_GATT_PERM_READ)
    permissions |= bluetooth::kAttributePermissionRead;

  if (perm & BT_GATT_PERM_WRITE)
    permissions |= bluetooth::kAttributePermissionWrite;

  if (perm & BT_GATT_PERM_READ_ENCRYPT)
    permissions |= bluetooth::kAttributePermissionReadEncrypted;

  if (perm & BT_GATT_PERM_WRITE_ENCRYPT)
    permissions |= bluetooth::kAttributePermissionWriteEncrypted;

  if (perm & BT_GATT_PERM_READ_AUTHEN)
    permissions |= bluetooth::kAttributePermissionReadEncryptedMITM;

  if (perm & BT_GATT_PERM_PREPARE_WRITE)
    permissions |= bluetooth::kAttributePermissionWrite;

  return permissions;
}

static void btgatts_register_server_cb(int status, int server_if,
                                       const Uuid& uuid)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  std::vector<btgatt_db_element_t> svcs;
  struct _bt_gatt_service *serv;
  struct bt_gatt_attr *attrs;
  struct bt_gatt_chrc *chrc;
  btgatt_db_element_t elem;
  struct bt_uuid_16 usecondary;
  struct bt_uuid_16 uprimary;
  struct bt_uuid_16 uchrc;
  struct bt_uuid_16 uccc;
  struct bt_uuid_16 *u16;
  struct bt_uuid *u;
  int count;

  serv = btgatt_service_find_by_uuid(uuid);
  if (serv == NULL)
    return;

  attrs = serv->servs->attrs;
  count = serv->servs->attr_count;

  uchrc.uuid.type       = BT_UUID_TYPE_16;
  uchrc.val             = BT_UUID_GATT_CHRC_VAL;
  uccc.uuid.type        = BT_UUID_TYPE_16;
  uccc.val              = BT_UUID_GATT_CCC_VAL;
  uprimary.uuid.type    = BT_UUID_TYPE_16;
  uprimary.val          = BT_UUID_GATT_PRIMARY_VAL;
  usecondary.uuid.type  = BT_UUID_TYPE_16;
  usecondary.val        = BT_UUID_GATT_SECONDARY_VAL;

  memset(&elem, 0x0, sizeof(elem));

  for (; attrs && count; attrs++, count--) {
    attrs->handle = 0;

    if (!bt_uuid_cmp(attrs->uuid, (const struct bt_uuid *)&uchrc)) {
      chrc            = (struct bt_gatt_chrc *)attrs->user_data;
      elem.properties = chrc->properties;
      elem.type       = BTGATT_DB_CHARACTERISTIC;
    } else if (!bt_uuid_cmp(attrs->uuid, (const struct bt_uuid *)&uccc)) {
      elem.type        = BTGATT_DB_DESCRIPTOR;
      elem.uuid        = bt_gatt_conv_uuid((struct bt_uuid *)attrs->uuid);
      elem.permissions = convert_permissions(attrs->perm);
      bt_gatt_service_insert(&svcs, &elem);
    } else if ((!bt_uuid_cmp(attrs->uuid, (const struct bt_uuid *)&uprimary) ||
          !bt_uuid_cmp(attrs->uuid, (const struct bt_uuid *)&usecondary)) && attrs->user_data) {
      u         = (struct bt_uuid *)attrs->user_data;
      elem.uuid = bt_gatt_conv_uuid(u);
      if (u->type == BT_UUID_TYPE_16) {
        u16       = (struct bt_uuid_16 *)u;
        elem.type = bt_gatt_conv_attr(u16->val);
      }
      elem.permissions = convert_permissions(attrs->perm);
      bt_gatt_service_insert(&svcs, &elem);
    } else {
      elem.uuid        = bt_gatt_conv_uuid((struct bt_uuid *)attrs->uuid);
      elem.permissions = convert_permissions(attrs->perm);
      bt_gatt_service_insert(&svcs, &elem);
    }
  }

  serv->handle = server_if;
  flrd->gatt->server->add_service(server_if, svcs);
}

static bt_conn *btgatts_get_connection(int conn_id)
{
  FAR sq_entry_t *entry;
  struct bt_conn *conn;

  for (entry = sq_peek(&g_conn_list); entry; entry = sq_next(entry)) {
    conn = (struct bt_conn *)entry;
    if (conn->info.id == conn_id)
      return conn;
  }

  return NULL;
}

static void btgatts_connection_cb(int conn_id,
    int server_if, int connected, const RawAddress& bda)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  struct _bt_gatt_service *serv;
  struct bt_conn_info *info;
  FAR sq_entry_t *entry;
  struct bt_conn_cb *cb;
  struct bt_conn *conn;
  bool found = false;
  bt_addr_le_t *addr;

  serv = btgatt_service_find_by_handle(server_if);
  if (serv == NULL)
    return;

  if (connected) {
    for (entry = sq_peek(&g_conn_list); entry; entry = sq_next(entry)) {
      conn = (struct bt_conn *)entry;
      if (conn->info.id == conn_id || !memcmp((void *)conn->info.le.dst->a.val,
            bda.address, sizeof(bda.address)))
        return;
    }

    conn = (struct bt_conn *)calloc(1, sizeof(*conn) + sizeof(bt_addr_le_t) * 2);
    if (conn == NULL)
      return;

    info       = &conn->info;
    info->id   = conn_id;
    info->role = BT_CONN_ROLE_MASTER;
    info->type = BT_CONN_TYPE_LE;
    conn->mtu  = GATT_DEF_BLE_MTU_SIZE;

    addr = (bt_addr_le_t *)(conn + 1);
    addr->type = BT_ADDR_LE_PUBLIC;
    memcpy(addr->a.val, bda.address, sizeof(addr->a.val));
    info->le.dst    = addr;
    info->le.remote = addr;

    addr += 1;
    addr->type = BT_ADDR_LE_PUBLIC;
    memcpy(addr->a.val, flrd->laddr->address, sizeof(addr->a.val));
    info->le.src   = addr;
    info->le.local = addr;

    for (cb = g_conn_callback_list; cb; cb = cb->_next) {
      if (cb->connected)
        cb->connected(conn, 0);
    }

    sq_addlast(&conn->node, &g_conn_list);
    serv->id = conn_id;
  } else {
    for (entry = sq_peek(&g_conn_list); entry; entry = sq_next(entry)) {
      conn = (struct bt_conn *)entry;
      if (conn->info.id == conn_id || !memcmp((void *)conn->info.le.dst->a.val,
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

    serv->id = 0;
  }
}

extern "C"
{
  void bt_id_get(bt_addr_le_t *addrs, size_t *count)
  {
    struct fluoride_s *flrd = fluoride_interface_get();

    if (!addrs || !count)
      return;

    addrs[0].type = BT_ADDR_LE_PUBLIC;
    memcpy(addrs[0].a.val, flrd->laddr->address, sizeof(addrs[0].a.val));

    *count = 1;
  }

  uint16_t bt_gatt_get_mtu(struct bt_conn *conn)
  {
    return conn->mtu;
  }

  int bt_gatt_service_register(struct bt_gatt_service *servs)
  {
    struct fluoride_s *flrd = fluoride_interface_get();
    struct _bt_gatt_service *serv;
    bt_status_t status;

    if (servs->attr_count <= 0 || !servs->attrs)
      return BT_STATUS_PARM_INVALID;

    pthread_mutex_lock(&g_service_mutex);
    serv = btgatt_service_find_by_servs(servs);
    if (serv) {
      status = BT_STATUS_DONE;
      goto fail;
    }

    serv = (struct _bt_gatt_service *)calloc(1, sizeof(struct _bt_gatt_service));
    if (serv == NULL) {
      status = BT_STATUS_NOMEM;
      goto fail;
    }

    serv->uuid  = bluetooth::Uuid::GetRandom();
    serv->servs = servs;

    sq_addlast(&serv->node, &g_gatts_callback_list);

    status = flrd->gatt->server->register_server(serv->uuid, false);
    if (status == BT_STATUS_SUCCESS)
      pthread_cond_wait(&g_service_cond, &g_service_mutex);

fail:
    pthread_mutex_unlock(&g_service_mutex);

    return status;
  }

  int bt_conn_get_info(const struct bt_conn *conn,
                       struct bt_conn_info *info)
  {
    memcpy(info, &conn->info, sizeof(*info));
    return 0;
  }

  void bt_conn_cb_register(struct bt_conn_cb *cb)
  {
    struct bt_conn_cb *_cb;

    for (_cb = g_conn_callback_list; _cb; _cb = _cb->_next)
      if (_cb == cb)
        return;

    cb->_next = g_conn_callback_list;
    g_conn_callback_list = cb;
  }

  ssize_t bt_gatt_attr_write_ccc(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr, const void *buf,
                                 uint16_t len, uint16_t offset, uint8_t flags)
  {
    return len;
  }

  ssize_t bt_gatt_attr_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            void *buf, uint16_t buf_len, uint16_t offset,
                            const void *value, uint16_t value_len)
  {
    struct fluoride_s *flrd = fluoride_interface_get();
    btgatt_response_t response = {};

    response.handle = attr->handle;
    response.attr_value.handle = attr->handle;
    memcpy(response.attr_value.value, value, value_len);
    response.attr_value.offset = offset;
    response.attr_value.len = value_len;

    flrd->gatt->server->send_response(conn->info.id, conn->tid,
        bluetooth::GATT_ERROR_NONE, response);

    return value_len;
  }

  ssize_t bt_gatt_attr_read_ccc(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr, void *buf,
                                uint16_t len, uint16_t offset)
  {
    return len;
  }

  ssize_t bt_gatt_attr_read_chrc(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr, void *buf,
                                 uint16_t len, uint16_t offset)
  {
    return len;
  }

  ssize_t bt_gatt_attr_read_service(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset)
  {
    return len;
  }

  int bt_gatt_notify_cb(struct bt_conn *conn,
                        struct bt_gatt_notify_params *params)
  {
    struct fluoride_s *flrd = fluoride_interface_get();
    struct _bt_gatt_service *serv = btgatt_service_find_by_id(conn->info.id);
    std::vector<uint8_t> vec((uint8_t *)params->data,
                             (uint8_t *)params->data + params->len);

    if (serv == NULL)
      return -EINVAL;

    return flrd->gatt->server->send_indication(serv->handle,
                                               params->attr->handle,
                                               conn->info.id, false, vec);
  }

  int bt_conn_le_param_update(struct bt_conn *conn,
                              const struct bt_le_conn_param *param)
  {
    struct fluoride_s *flrd = fluoride_interface_get();
    RawAddress bda;

    bda.FromOctets(conn->info.le.dst->a.val);
    return flrd->gatt->client->conn_parameter_update(bda,
                                                     param->interval_min, param->interval_max,
                                                     param->latency, param->timeout,
                                                     0, 0);
  }
}

static void btgatts_service_added_cb(int status, int server_if,
                                     std::vector<btgatt_db_element_t> elems)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  struct _bt_gatt_service *serv;
  struct bt_gatt_attr *attrs;
  size_t count, i, k;

  serv = btgatt_service_find_by_handle(server_if);
  if (serv == NULL)
    return;

  attrs = serv->servs->attrs;
  count = serv->servs->attr_count;

  for (i = 0; i < elems.size(); i++) {
    for (k = 0; k < count; k++) {
      if (attrs[k].handle == 0 &&
          elems[i].uuid == bt_gatt_conv_uuid((struct bt_uuid *)attrs[k].uuid)) {
        attrs[k].handle = elems[i].attribute_handle;
        break;
      }
    }
  }

  pthread_mutex_lock(&g_service_mutex);
  pthread_cond_signal(&g_service_cond);
  pthread_mutex_unlock(&g_service_mutex);
}

static struct bt_gatt_attr *btgatts_find_attribute(struct _bt_gatt_service *serv,
                                                   int attr_handle)
{
  struct bt_gatt_attr *attrs = serv->servs->attrs;
  int count = serv->servs->attr_count;
  int i;

  for (i = 0; i < count; i++)
    if (attrs[i].handle == attr_handle)
      return &attrs[i];

  return NULL;
}

static void btgatts_request_write_cb(int conn_id, int trans_id, const RawAddress& bda,
                                     int attr_handle, int offset, bool need_rsp, bool is_prep,
                                     std::vector<uint8_t> value, bt_gatt_db_attribute_type_t type)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  btgatt_response_t response = {};
  struct _bt_gatt_service *serv;
  struct bt_gatt_attr *attr;
  struct _bt_gatt_ccc *ccc;
  struct bt_uuid_16 *u16;
  struct bt_conn *conn;

  serv = btgatt_service_find_by_id(conn_id);
  if (serv == NULL)
    return;

  response.handle            = attr_handle;
  response.attr_value.handle = attr_handle;
  response.attr_value.offset = offset;

  if (is_prep) {
    flrd->gatt->server->send_response(conn_id, trans_id,
        bluetooth::GATT_ERROR_REQUEST_NOT_SUPPORTED, response);
    return;
  }

  attr = btgatts_find_attribute(serv, attr_handle);

  if (attr) {
    if (type == BTGATT_DB_CHARACTERISTIC) {
      if (attr->write && (conn = btgatts_get_connection(conn_id)) != NULL) {
        conn->tid = trans_id;
        attr->write(conn, attr, value.data(), value.size(), offset, 0);
      }
    } else {
      u16 = (struct bt_uuid_16 *)attr->uuid;
      if (u16->uuid.type == BT_UUID_TYPE_16 && u16->val == BT_UUID_GATT_CCC_VAL) {
        ccc = (struct _bt_gatt_ccc *)attr->user_data;
        ccc->value = value[0] + (value[1] << 8);
        if (ccc->cfg_changed)
          ccc->cfg_changed(attr, ccc->value);
      }
    }
  }

  if (need_rsp) {
    flrd->gatt->server->send_response(conn_id, trans_id,
        bluetooth::GATT_ERROR_NONE, response);
  }
}

static void btgatts_request_write_descriptor_cb(int conn_id, int trans_id, const RawAddress& bda,
                                                int attr_handle, int offset, bool need_rsp, bool is_prep,
                                                std::vector<uint8_t> value)
{
  btgatts_request_write_cb(conn_id, trans_id, bda, attr_handle, offset,
                           need_rsp, is_prep, value, BTGATT_DB_DESCRIPTOR);
}

static void btgatts_request_write_characteristic_cb(int conn_id, int trans_id, const RawAddress& bda,
                                                    int attr_handle, int offset, bool need_rsp, bool is_prep,
                                                    std::vector<uint8_t> value)
{
  btgatts_request_write_cb(conn_id, trans_id, bda, attr_handle, offset,
                           need_rsp, is_prep, value, BTGATT_DB_CHARACTERISTIC);
}

static void btgatts_request_read_descriptor_cb(int conn_id, int trans_id,
                                               const RawAddress& bda, int attr_handle,
                                               int offset, bool is_long)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  btgatt_response_t response = {};
  struct _bt_gatt_service *serv;
  struct bt_gatt_attr *attr;
  struct _bt_gatt_ccc *ccc;
  struct bt_uuid_16 *u16;
  uint16_t value = 0;

  serv = btgatt_service_find_by_id(conn_id);
  if (serv == NULL)
    return;

  attr = btgatts_find_attribute(serv, attr_handle);
  if (attr) {
    u16 = (struct bt_uuid_16 *)attr->uuid;
    if (u16->uuid.type == BT_UUID_TYPE_16 &&
        u16->val == BT_UUID_GATT_CCC_VAL) {
      ccc = (struct _bt_gatt_ccc *)attr->user_data;
      if (ccc->cfg_changed)
        ccc->cfg_changed(attr, ccc->value);
      value = ccc->value;
    }
  }

  response.handle = attr_handle;
  response.attr_value.handle = attr_handle;
  response.attr_value.offset = offset;
  memcpy(response.attr_value.value, &value, sizeof(uint16_t));
  response.attr_value.len = 2;

  flrd->gatt->server->send_response(conn_id, trans_id,
      bluetooth::GATT_ERROR_NONE, response);
}

static void btgatts_request_read_characteristic_cb(int conn_id, int trans_id,
                                                   const RawAddress& bda, int attr_handle,
                                                   int offset, bool is_long)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  btgatt_response_t response = {};
  struct _bt_gatt_service *serv;
  struct bt_gatt_attr *attr;
  struct bt_conn *conn;

  serv = btgatt_service_find_by_id(conn_id);
  if (serv == NULL)
    return;

  attr = btgatts_find_attribute(serv, attr_handle);
  if (attr && attr->read && (conn = btgatts_get_connection(conn_id)) != NULL) {
    conn->tid = trans_id;
    attr->read(conn, attr, NULL, 0, offset);
  } else {
    response.handle = attr_handle;
    response.attr_value.handle = attr_handle;
    response.attr_value.offset = offset;
    response.attr_value.len = 0;

    flrd->gatt->server->send_response(conn_id, trans_id,
        bluetooth::GATT_ERROR_NONE, response);
  }
}

static void btgatts_mtu_changed_cb(int conn_id, int mtu)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  struct bt_conn *conn;

  conn = btgatts_get_connection(conn_id);
  if (conn == NULL)
    return;

  conn->mtu = mtu;
}

static void btgatts_service_stopped_cb(int status, int server_if, int srvc_handle) TRACE_CALLBACK_BODY
static void btgatts_service_deleted_cb(int status, int server_if, int srvc_handle) TRACE_CALLBACK_BODY
static void btgatts_request_exec_write_cb(int conn_id, int trans_id, const RawAddress& bda, int exec_write) TRACE_CALLBACK_BODY
static void btgatts_response_confirmation_cb(int status, int handle) TRACE_CALLBACK_BODY
static void btgatts_indication_sent_cb(int conn_id, int status) TRACE_CALLBACK_BODY
static void btgatts_congestion_cb(int conn_id, bool congested) TRACE_CALLBACK_BODY
static void btgatts_phy_updated_cb(int conn_id, uint8_t tx_phy, uint8_t rx_phy, uint8_t status) TRACE_CALLBACK_BODY
static void btgatts_conn_updated_cb(int conn_id, uint16_t interval, uint16_t latency,
                                    uint16_t timeout, uint8_t status) TRACE_CALLBACK_BODY

static const btgatt_server_callbacks_t sGattServerCallbacks =
{
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

static const btgatt_callbacks_t sGattCallbacks =
{
  sizeof(btgatt_callbacks_t),
  &sGattClientCallbacks,
  &sGattServerCallbacks,
  &sGattScannerCallbacks,
};

class FlrdAdvertisingCallbacks : public AdvertisingCallbacks
{
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

class FlrdScanningCallbacks : public ScanningCallbacks
{
  public:
    void OnScannerRegistered(const Uuid app_uuid, uint8_t scannerId, uint8_t status) TRACE_CALLBACK_BODY
      void OnScanResult(uint16_t event_type, uint8_t addr_type, RawAddress* bda,
                        uint8_t primary_phy, uint8_t secondary_phy,
                        uint8_t advertising_sid, int8_t tx_power, int8_t rssi,
                        uint16_t periodic_adv_int, std::vector<uint8_t> adv_data) TRACE_CALLBACK_BODY
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
  sq_init(&g_gatts_callback_list);

  gatt->init(&sGattCallbacks);
  gatt->advertiser->RegisterCallbacks(static_cast<AdvertisingCallbacks *>(&sAdvertisingCallbacks));
  gatt->scanner->RegisterCallbacks(static_cast<ScanningCallbacks *>(&sScanningCallbacks));

  return gatt;
}
