/******************************************************************************
 *
 *  Copyright 1999-2012 Broadcom Corporation
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

/******************************************************************************
 *
 *  This file contains functions for BLE acceptlist operation.
 *
 ******************************************************************************/

#include <base/bind.h>
#include <cstdint>
#include <unordered_map>

#include "device/include/controller.h"
#include "stack/btm/btm_dev.h"
#include "stack/btm/btm_int_types.h"
#include "stack/btm/security_device_record.h"
#include "stack/include/bt_types.h"
#include "stack/include/hcimsgs.h"
#include "types/raw_address.h"

extern tBTM_CB btm_cb;

extern void btm_send_hci_create_connection(
    uint16_t scan_int, uint16_t scan_win, uint8_t init_filter_policy,
    uint8_t addr_type_peer, const RawAddress& bda_peer, uint8_t addr_type_own,
    uint16_t conn_int_min, uint16_t conn_int_max, uint16_t conn_latency,
    uint16_t conn_timeout, uint16_t min_ce_len, uint16_t max_ce_len,
    uint8_t phy);
extern void btm_ble_create_conn_cancel();

static bool btm_ble_stop_auto_conn();
static void wl_remove_complete(uint8_t* p_data, uint16_t /* evt_len */);

// Unfortunately (for now?) we have to maintain a copy of the device acceptlist
// on the host to determine if a device is pending to be connected or not. This
// controls whether the host should keep trying to scan for acceptlisted
// peripherals or not.
// TODO: Move all of this to controller/le/background_list or similar?
struct BackgroundConnection {
  RawAddress address;
  uint8_t addr_type;
  bool in_controller_wl;
  uint8_t addr_type_in_wl;
  bool pending_removal;
};

struct BgConnHash {
  std::size_t operator()(const RawAddress& x) const {
    const uint8_t* a = x.address;
    return a[0] ^ (a[1] << 8) ^ (a[2] << 16) ^ (a[3] << 24) ^ a[4] ^
           (a[5] << 8);
  }
};

static std::unordered_map<RawAddress, BackgroundConnection, BgConnHash>
    background_connections;

static void background_connection_add(uint8_t addr_type,
                                      const RawAddress& address) {
  auto map_iter = background_connections.find(address);
  if (map_iter == background_connections.end()) {
    background_connections[address] =
        BackgroundConnection{address, addr_type, false, 0, false};
  } else {
    BackgroundConnection* connection = &map_iter->second;
    if (addr_type != connection->addr_type) {
      LOG(INFO) << __func__ << " Addr type mismatch " << address;
      btsnd_hcic_ble_remove_from_acceptlist(connection->addr_type_in_wl,
                                            connection->address,
                                            base::Bind(&wl_remove_complete));
      connection->addr_type = addr_type;
      connection->in_controller_wl = false;
    }
    connection->pending_removal = false;
  }
}

static void background_connection_remove(const RawAddress& address) {
  auto map_iter = background_connections.find(address);
  if (map_iter != background_connections.end()) {
    if (map_iter->second.in_controller_wl) {
      map_iter->second.pending_removal = true;
    } else {
      background_connections.erase(map_iter);
    }
  }
}

static void background_connections_clear() { background_connections.clear(); }

static bool background_connections_pending() {
  for (auto& map_el : background_connections) {
    BackgroundConnection* connection = &map_el.second;
    if (connection->pending_removal) continue;
    const bool connected =
        BTM_IsAclConnectionUp(connection->address, BT_TRANSPORT_LE);
    if (!connected) {
      return true;
    }
  }
  return false;
}

static int background_connections_count() {
  int count = 0;
  for (auto& map_el : background_connections) {
    if (!map_el.second.pending_removal) ++count;
  }
  return count;
}

/*******************************************************************************
 *
 * Function         btm_update_scanner_filter_policy
 *
 * Description      This function updates the filter policy of scanner
 ******************************************************************************/
void btm_update_scanner_filter_policy(tBTM_BLE_SFP scan_policy) {
  tBTM_BLE_INQ_CB* p_inq = &btm_cb.ble_ctr_cb.inq_var;

  uint32_t scan_interval =
      !p_inq->scan_interval ? BTM_BLE_GAP_DISC_SCAN_INT : p_inq->scan_interval;
  uint32_t scan_window =
      !p_inq->scan_window ? BTM_BLE_GAP_DISC_SCAN_WIN : p_inq->scan_window;

  BTM_TRACE_EVENT("%s", __func__);

  p_inq->sfp = scan_policy;
  p_inq->scan_type = p_inq->scan_type == BTM_BLE_SCAN_MODE_NONE
                         ? BTM_BLE_SCAN_MODE_ACTI
                         : p_inq->scan_type;

  btm_send_hci_set_scan_params(
      p_inq->scan_type, (uint16_t)scan_interval, (uint16_t)scan_window,
      btm_cb.ble_ctr_cb.addr_mgnt_cb.own_addr_type, scan_policy);
}

/*******************************************************************************
 *
 * Function         btm_ble_bgconn_cancel_if_disconnected
 *
 * Description      If a device has been disconnected, it must be re-added to
 *                  the acceptlist. If needed, this function cancels a pending
 *                  initiate command in order to trigger restart of the initiate
 *                  command which in turn updates the acceptlist.
 *
 * Parameters       bd_addr: updated device
 *
 ******************************************************************************/
void btm_ble_bgconn_cancel_if_disconnected(const RawAddress& bd_addr) {
  if (!btm_cb.ble_ctr_cb.is_connection_state_connecting()) return;

  auto map_it = background_connections.find(bd_addr);
  if (map_it != background_connections.end()) {
    BackgroundConnection* connection = &map_it->second;
    if (!connection->in_controller_wl && !connection->pending_removal &&
        !BTM_IsAclConnectionUp(bd_addr, BT_TRANSPORT_LE)) {
      btm_ble_stop_auto_conn();
    }
  }
}

bool BTM_BackgroundConnectAddressKnown(const RawAddress& address) {
  tBTM_SEC_DEV_REC* p_dev_rec = btm_find_dev(address);

  //  not a known device, or a classic device, we assume public address
  if (p_dev_rec == NULL || (p_dev_rec->device_type & BT_DEVICE_TYPE_BLE) == 0)
    return true;

  // bonded device with identity address known
  if (!p_dev_rec->ble.identity_address_with_type.bda.IsEmpty()) {
    return true;
  }

  // Public address, Random Static, or Random Non-Resolvable Address known
  if (p_dev_rec->ble.ble_addr_type == BLE_ADDR_PUBLIC ||
      !BTM_BLE_IS_RESOLVE_BDA(address)) {
    return true;
  }

  // Only Resolvable Private Address (RPA) is known, we don't allow it into
  // the background connection procedure.
  return false;
}

/*******************************************************************************
 *
 * Function         btm_add_dev_to_controller
 *
 * Description      This function load the device into controller acceptlist
 ******************************************************************************/
static bool btm_add_dev_to_controller(bool to_add, const RawAddress& bd_addr) {
  tBTM_SEC_DEV_REC* p_dev_rec = btm_find_dev(bd_addr);

  if (p_dev_rec != NULL && p_dev_rec->device_type & BT_DEVICE_TYPE_BLE) {
    if (to_add) {
      if (!p_dev_rec->ble.identity_address_with_type.bda.IsEmpty()) {
        LOG_DEBUG(
            "Adding known device record into acceptlist with identity "
            "device:%s",
            p_dev_rec->ble.identity_address_with_type.bda.ToString().c_str());
        background_connection_add(
            p_dev_rec->ble.identity_address_with_type.type,
            p_dev_rec->ble.identity_address_with_type.bda);
      } else {
        LOG_DEBUG(
            "Adding known device record into acceptlist without identity "
            "device:%s",
            bd_addr.ToString().c_str());
        background_connection_add(p_dev_rec->ble.ble_addr_type, bd_addr);

        if (p_dev_rec->ble.ble_addr_type == BLE_ADDR_RANDOM &&
            BTM_BLE_IS_RESOLVE_BDA(bd_addr)) {
          LOG(INFO) << __func__ << " addig RPA into acceptlist";
        }
      }

      p_dev_rec->ble.in_controller_list |= BTM_ACCEPTLIST_BIT;
    } else {
      if (!p_dev_rec->ble.identity_address_with_type.bda.IsEmpty()) {
        LOG_DEBUG(
            "Removing known device record into acceptlist with identity "
            "device:%s",
            p_dev_rec->ble.identity_address_with_type.bda.ToString().c_str());
        background_connection_remove(
            p_dev_rec->ble.identity_address_with_type.bda);
      } else {
        LOG_DEBUG(
            "Removing known device record into acceptlist without identity "
            "device:%s",
            bd_addr.ToString().c_str());
        background_connection_remove(bd_addr);

        if (p_dev_rec->ble.ble_addr_type == BLE_ADDR_RANDOM &&
            BTM_BLE_IS_RESOLVE_BDA(bd_addr)) {
          LOG(INFO) << __func__ << " removing RPA from acceptlist";
        }
      }

      p_dev_rec->ble.in_controller_list &= ~BTM_ACCEPTLIST_BIT;
    }
  } else {
    /* not a known device, i.e. attempt to connect to device never seen before
     */
    if (to_add)
      background_connection_add(BLE_ADDR_PUBLIC, bd_addr);
    else
      background_connection_remove(bd_addr);
  }

  return true;
}

/** Acceptlist add complete */
static void wl_add_complete(uint8_t* p_data, uint16_t /* evt_len */) {
  uint8_t status;
  STREAM_TO_UINT8(status, p_data);
  VLOG(2) << __func__ << ": status=" << loghex(status);
}

/** Acceptlist element remove complete */
static void wl_remove_complete(uint8_t* p_data, uint16_t /* evt_len */) {
  uint8_t status;
  STREAM_TO_UINT8(status, p_data);
  VLOG(2) << __func__ << ": status=" << loghex(status);
}

/*******************************************************************************
 *
 * Function         btm_execute_wl_dev_operation
 *
 * Description      execute the pending acceptlist device operation (loading or
 *                                                                  removing)
 ******************************************************************************/
static bool btm_execute_wl_dev_operation(void) {
  // handle removals first to avoid filling up controller's acceptlist
  for (auto map_it = background_connections.begin();
       map_it != background_connections.end();) {
    BackgroundConnection* connection = &map_it->second;
    if (connection->pending_removal) {
      btsnd_hcic_ble_remove_from_acceptlist(
          connection->addr_type_in_wl, connection->address,
          base::BindOnce(&wl_remove_complete));
      map_it = background_connections.erase(map_it);
    } else
      ++map_it;
  }
  for (auto& map_el : background_connections) {
    BackgroundConnection* connection = &map_el.second;
    const bool connected =
        BTM_IsAclConnectionUp(connection->address, BT_TRANSPORT_LE);
    if (!connection->in_controller_wl && !connected) {
      btsnd_hcic_ble_add_acceptlist(connection->addr_type, connection->address,
                                    base::BindOnce(&wl_add_complete));
      connection->in_controller_wl = true;
      connection->addr_type_in_wl = connection->addr_type;
    } else if (connection->in_controller_wl && connected) {
      /* Bluetooth Core 4.2 as well as ESR08 disallows more than one
         connection between two LE addresses. Not all controllers handle this
         correctly, therefore we must make sure connected devices are not in
         the acceptlist when bg connection attempt is active. */
      btsnd_hcic_ble_remove_from_acceptlist(
          connection->addr_type_in_wl, connection->address,
          base::BindOnce(&wl_remove_complete));
      connection->in_controller_wl = false;
    }
  }
  return true;
}

/*******************************************************************************
 *
 * Function         btm_ble_acceptlist_init
 *
 * Description      Initialize acceptlist size
 *
 ******************************************************************************/
void btm_ble_acceptlist_init(uint8_t acceptlist_size) {
  BTM_TRACE_DEBUG("%s acceptlist_size = %d", __func__, acceptlist_size);
}

uint8_t BTM_GetAcceptlistSize() {
  const controller_t* controller = controller_get_interface();
  uint8_t max_size = controller->get_ble_acceptlist_size();

  if (!controller->supports_ble()) {
    return 0;
  }
  return max_size > BTA_GATTC_KNOWN_SR_MAX ?
         BTA_GATTC_KNOWN_SR_MAX : max_size;
}

bool BTM_SetLeConnectionModeToFast() {
  VLOG(2) << __func__;
  tBTM_BLE_CB* p_cb = &btm_cb.ble_ctr_cb;
  if ((p_cb->scan_int == BTM_BLE_SCAN_PARAM_UNDEF &&
       p_cb->scan_win == BTM_BLE_SCAN_PARAM_UNDEF) ||
      (p_cb->scan_int == BTM_BLE_SCAN_SLOW_INT_1 &&
       p_cb->scan_win == BTM_BLE_SCAN_SLOW_WIN_1)) {
    p_cb->scan_int = BTM_BLE_SCAN_FAST_INT;
    p_cb->scan_win = BTM_BLE_SCAN_FAST_WIN;
    return true;
  }
  return false;
}

void BTM_SetLeConnectionModeToSlow() {
  VLOG(2) << __func__;
  tBTM_BLE_CB* p_cb = &btm_cb.ble_ctr_cb;
  if ((p_cb->scan_int == BTM_BLE_SCAN_PARAM_UNDEF &&
       p_cb->scan_win == BTM_BLE_SCAN_PARAM_UNDEF) ||
      (p_cb->scan_int == BTM_BLE_SCAN_FAST_INT &&
       p_cb->scan_win == BTM_BLE_SCAN_FAST_WIN)) {
    p_cb->scan_int = BTM_BLE_SCAN_SLOW_INT_1;
    p_cb->scan_win = BTM_BLE_SCAN_SLOW_WIN_1;
  }
}

/** This function is to start auto connection procedure */
static bool btm_ble_start_auto_conn() {
  tBTM_BLE_CB* p_cb = &btm_cb.ble_ctr_cb;

  BTM_TRACE_EVENT("%s", __func__);

  uint16_t scan_int = (p_cb->scan_int == BTM_BLE_SCAN_PARAM_UNDEF)
                          ? BTM_BLE_SCAN_SLOW_INT_1
                          : p_cb->scan_int;
  uint16_t scan_win = (p_cb->scan_win == BTM_BLE_SCAN_PARAM_UNDEF)
                          ? BTM_BLE_SCAN_SLOW_WIN_1
                          : p_cb->scan_win;
  uint8_t own_addr_type = p_cb->addr_mgnt_cb.own_addr_type;
  uint8_t peer_addr_type = BLE_ADDR_PUBLIC;

  uint8_t phy = PHY_LE_1M;
  if (controller_get_interface()->supports_ble_2m_phy()) phy |= PHY_LE_2M;
  if (controller_get_interface()->supports_ble_coded_phy()) phy |= PHY_LE_CODED;

  if (!btm_ble_topology_check(BTM_BLE_STATE_INIT)) {
    LOG(INFO) << "initate background connection fail, topology limitation";
    return false;
  }

  if (!btm_cb.ble_ctr_cb.is_connection_state_idle() ||
      !background_connections_pending()) {
    return false;
  }

  p_cb->wl_state |= BTM_BLE_ACCEPTLIST_INIT;

  btm_execute_wl_dev_operation();

  btm_ble_enable_resolving_list_for_platform(BTM_BLE_RL_INIT);
  if (btm_cb.ble_ctr_cb.rl_state != BTM_BLE_RL_IDLE &&
      controller_get_interface()->supports_ble_privacy()) {
    own_addr_type |= BLE_ADDR_TYPE_ID_BIT;
    peer_addr_type |= BLE_ADDR_TYPE_ID_BIT;
  }

  btm_send_hci_create_connection(
      scan_int,                            /* uint16_t scan_int      */
      scan_win,                            /* uint16_t scan_win      */
      0x01,                                /* uint8_t acceptlist     */
      peer_addr_type,                      /* uint8_t addr_type_peer */
      RawAddress::kEmpty,                  /* BD_ADDR bda_peer     */
      own_addr_type,                       /* uint8_t addr_type_own */
      BTM_BLE_CONN_INT_MIN_DEF,            /* uint16_t conn_int_min  */
      BTM_BLE_CONN_INT_MAX_DEF,            /* uint16_t conn_int_max  */
      BTM_BLE_CONN_PERIPHERAL_LATENCY_DEF, /* uint16_t conn_latency  */
      BTM_BLE_CONN_TIMEOUT_DEF,            /* uint16_t conn_timeout  */
      0,                                   /* uint16_t min_len       */
      0,                                   /* uint16_t max_len       */
      phy);
  return true;
}

/** This function is to stop auto connection procedure */
static bool btm_ble_stop_auto_conn() {
  BTM_TRACE_EVENT("%s", __func__);

  if (!btm_cb.ble_ctr_cb.is_connection_state_connecting()) {
    BTM_TRACE_DEBUG("%s not in auto conn state, cannot stop", __func__);
    return false;
  }

  btm_ble_create_conn_cancel();

  btm_cb.ble_ctr_cb.wl_state &= ~BTM_BLE_ACCEPTLIST_INIT;
  return true;
}

/*******************************************************************************
 *
 * Function         btm_ble_suspend_bg_conn
 *
 * Description      This function is to suspend an active background connection
 *                  procedure.
 *
 * Parameters       none.
 *
 * Returns          none.
 *
 ******************************************************************************/
bool btm_ble_suspend_bg_conn(void) {
  BTM_TRACE_EVENT("%s", __func__);
  return btm_ble_stop_auto_conn();
}

/*******************************************************************************
 *
 * Function         btm_ble_resume_bg_conn
 *
 * Description      This function is to resume a background auto connection
 *                  procedure.
 *
 * Parameters       none.
 *
 * Returns          none.
 *
 ******************************************************************************/
bool btm_ble_resume_bg_conn(void) { return btm_ble_start_auto_conn(); }

/** Adds the device into acceptlist. Returns false if acceptlist is full and
 * device can't be added, true otherwise. */
bool BTM_AcceptlistAdd(const RawAddress& address) {
  VLOG(1) << __func__ << ": " << address;

  if (background_connections_count() ==
      controller_get_interface()->get_ble_acceptlist_size()) {
    LOG_ERROR("Unable to add device to acceptlist since it is full");
    return false;
  }

  if (btm_cb.ble_ctr_cb.wl_state & BTM_BLE_ACCEPTLIST_INIT) {
    btm_ble_stop_auto_conn();
  }
  btm_add_dev_to_controller(true, address);
  btm_ble_resume_bg_conn();
  return true;
}

/** Removes the device from acceptlist */
void BTM_AcceptlistRemove(const RawAddress& address) {
  VLOG(1) << __func__ << ": " << address;
  if (btm_cb.ble_ctr_cb.wl_state & BTM_BLE_ACCEPTLIST_INIT) {
    btm_ble_stop_auto_conn();
  }
  btm_add_dev_to_controller(false, address);
  btm_ble_resume_bg_conn();
}

/** clear acceptlist complete */
void wl_clear_complete(uint8_t* p_data, uint16_t /* evt_len */) {
  uint8_t status;
  STREAM_TO_UINT8(status, p_data);
  VLOG(2) << __func__ << ": status=" << loghex(status);
}

/** Clear the acceptlist, end any pending acceptlist connections */
void BTM_AcceptlistClear() {
  VLOG(1) << __func__;
  if (!controller_get_interface()->supports_ble()) return;
  btm_ble_stop_auto_conn();
  btsnd_hcic_ble_clear_acceptlist(base::BindOnce(&wl_clear_complete));
  background_connections_clear();
}
