/******************************************************************************
 *
 *  Copyright 2000-2012 Broadcom Corporation
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

/*****************************************************************************
 *
 *  Name:          btm_acl.cc
 *
 *  Description:   This file contains functions that handle ACL connections.
 *                 This includes operations such as hold and sniff modes,
 *                 supported packet types.
 *
 *                 This module contains both internal and external (API)
 *                 functions. External (API) functions are distinguishable
 *                 by their names beginning with uppercase BTM.
 *
 *
 *****************************************************************************/

#define LOG_TAG "btm_acl"

#include <cstdint>

#include "bta/include/bta_dm_acl.h"
#include "bta/sys/bta_sys.h"
#include "btif/include/btif_acl.h"
#include "common/metrics.h"
#include "device/include/controller.h"
#include "device/include/interop.h"
#include "include/l2cap_hci_link_interface.h"
#include "main/shim/acl_api.h"
#include "main/shim/btm_api.h"
#include "main/shim/shim.h"
#include "osi/include/log.h"
#include "stack/acl/acl.h"
#include "stack/btm/btm_dev.h"
#include "stack/btm/btm_int_types.h"
#include "stack/btm/btm_sec.h"
#include "stack/gatt/connection_manager.h"
#include "stack/include/acl_api.h"
#include "stack/include/acl_hci_link_interface.h"
#include "stack/include/btm_api.h"
#include "stack/include/btu.h"
#include "stack/include/hcimsgs.h"
#include "stack/include/l2cap_acl_interface.h"
#include "types/raw_address.h"

struct StackAclBtmAcl {
  tACL_CONN* acl_allocate_connection();
  tACL_CONN* acl_get_connection_from_handle(uint16_t handle);
  tACL_CONN* btm_bda_to_acl(const RawAddress& bda, tBT_TRANSPORT transport);
  tBTM_STATUS btm_set_packet_types(tACL_CONN* p, uint16_t pkt_types);
  void btm_establish_continue(tACL_CONN* p_acl_cb);
  void btm_read_remote_features(uint16_t handle);
  void btm_set_default_link_policy(uint16_t settings);
  void btm_acl_role_changed(uint8_t hci_status, const RawAddress& bd_addr,
                            uint8_t new_role);
};

namespace {
StackAclBtmAcl internal_;
}

#define BTM_MAX_SW_ROLE_FAILED_ATTEMPTS 3

/* Define masks for supported and exception 2.0 ACL packet types
 */
#define BTM_ACL_SUPPORTED_PKTS_MASK                                           \
  (HCI_PKT_TYPES_MASK_DM1 | HCI_PKT_TYPES_MASK_DH1 | HCI_PKT_TYPES_MASK_DM3 | \
   HCI_PKT_TYPES_MASK_DH3 | HCI_PKT_TYPES_MASK_DM5 | HCI_PKT_TYPES_MASK_DH5)

#define BTM_ACL_EXCEPTION_PKTS_MASK                            \
  (HCI_PKT_TYPES_MASK_NO_2_DH1 | HCI_PKT_TYPES_MASK_NO_3_DH1 | \
   HCI_PKT_TYPES_MASK_NO_2_DH3 | HCI_PKT_TYPES_MASK_NO_3_DH3 | \
   HCI_PKT_TYPES_MASK_NO_2_DH5 | HCI_PKT_TYPES_MASK_NO_3_DH5)

#define BTM_EPR_AVAILABLE(p)                                        \
  ((HCI_ATOMIC_ENCRYPT_SUPPORTED((p)->peer_lmp_feature_pages[0]) && \
    controller_get_interface()->supports_encryption_pause())        \
       ? true                                                       \
       : false)

extern tBTM_CB btm_cb;

static void btm_acl_chk_peer_pkt_type_support(tACL_CONN* p,
                                              uint16_t* p_pkt_type);
static void btm_cont_rswitch(tACL_CONN* p, tBTM_SEC_DEV_REC* p_dev_rec);
static void btm_read_automatic_flush_timeout_timeout(void* data);
static void btm_read_failed_contact_counter_timeout(void* data);
static void btm_read_remote_ext_features(uint16_t handle, uint8_t page_number);
static void btm_read_rssi_timeout(void* data);
static void btm_read_tx_power_timeout(void* data);
static void btm_process_remote_ext_features(tACL_CONN* p_acl_cb,
                                            uint8_t num_read_pages);
static void btm_sec_set_peer_sec_caps(tACL_CONN* p_acl_cb,
                                      tBTM_SEC_DEV_REC* p_dev_rec);
static bool acl_is_role_master(const RawAddress& bda, tBT_TRANSPORT transport);
static void btm_set_link_policy(tACL_CONN* conn, uint16_t policy);

/* 3 seconds timeout waiting for responses */
#define BTM_DEV_REPLY_TIMEOUT_MS (3 * 1000)

/*******************************************************************************
 *
 * Function         btm_acl_init
 *
 * Description      This function is called at BTM startup to initialize
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_acl_init(void) {
  BTM_TRACE_DEBUG("btm_acl_init");
  /* Initialize nonzero defaults */
  btm_cb.acl_cb_.btm_def_link_super_tout = HCI_DEFAULT_INACT_TOUT;
  btm_cb.acl_cb_.acl_disc_reason = 0xff;

  btm_cb.acl_cb_.btm_acl_pkt_types_supported =
      HCI_PKT_TYPES_MASK_DH1 + HCI_PKT_TYPES_MASK_DM1 + HCI_PKT_TYPES_MASK_DH3 +
      HCI_PKT_TYPES_MASK_DM3 + HCI_PKT_TYPES_MASK_DH5 + HCI_PKT_TYPES_MASK_DM5;
}

void BTM_acl_after_controller_started() {
  internal_.btm_set_default_link_policy(
      HCI_ENABLE_MASTER_SLAVE_SWITCH | HCI_ENABLE_HOLD_MODE |
      HCI_ENABLE_SNIFF_MODE | HCI_ENABLE_PARK_MODE);

  const controller_t* controller = controller_get_interface();

  /* Create ACL supported packet types mask */
  btm_cb.acl_cb_.btm_acl_pkt_types_supported =
      (HCI_PKT_TYPES_MASK_DH1 + HCI_PKT_TYPES_MASK_DM1);

  if (controller->supports_3_slot_packets())
    btm_cb.acl_cb_.btm_acl_pkt_types_supported |=
        (HCI_PKT_TYPES_MASK_DH3 + HCI_PKT_TYPES_MASK_DM3);

  if (controller->supports_5_slot_packets())
    btm_cb.acl_cb_.btm_acl_pkt_types_supported |=
        (HCI_PKT_TYPES_MASK_DH5 + HCI_PKT_TYPES_MASK_DM5);

  /* Add in EDR related ACL types */
  if (!controller->supports_classic_2m_phy()) {
    btm_cb.acl_cb_.btm_acl_pkt_types_supported |=
        (HCI_PKT_TYPES_MASK_NO_2_DH1 + HCI_PKT_TYPES_MASK_NO_2_DH3 +
         HCI_PKT_TYPES_MASK_NO_2_DH5);
  }

  if (!controller->supports_classic_3m_phy()) {
    btm_cb.acl_cb_.btm_acl_pkt_types_supported |=
        (HCI_PKT_TYPES_MASK_NO_3_DH1 + HCI_PKT_TYPES_MASK_NO_3_DH3 +
         HCI_PKT_TYPES_MASK_NO_3_DH5);
  }

  /* Check to see if 3 and 5 slot packets are available */
  if (controller->supports_classic_2m_phy() ||
      controller->supports_classic_3m_phy()) {
    if (!controller->supports_3_slot_edr_packets())
      btm_cb.acl_cb_.btm_acl_pkt_types_supported |=
          (HCI_PKT_TYPES_MASK_NO_2_DH3 + HCI_PKT_TYPES_MASK_NO_3_DH3);

    if (!controller->supports_5_slot_edr_packets())
      btm_cb.acl_cb_.btm_acl_pkt_types_supported |=
          (HCI_PKT_TYPES_MASK_NO_2_DH5 + HCI_PKT_TYPES_MASK_NO_3_DH5);
  }

  BTM_TRACE_DEBUG("Local supported ACL packet types: 0x%04x",
                  btm_cb.acl_cb_.btm_acl_pkt_types_supported);
}

/*******************************************************************************
 *
 * Function        btm_bda_to_acl
 *
 * Description     This function returns the FIRST acl_db entry for the passed
 *                 BDA.
 *
 * Parameters      bda : BD address of the remote device
 *                 transport : Physical transport used for ACL connection
 *                 (BR/EDR or LE)
 *
 * Returns         Returns pointer to the ACL DB for the requested BDA if found.
 *                 NULL if not found.
 *
 ******************************************************************************/
tACL_CONN* StackAclBtmAcl::btm_bda_to_acl(const RawAddress& bda,
                                          tBT_TRANSPORT transport) {
  tACL_CONN* p = &btm_cb.acl_cb_.acl_db[0];
  uint16_t xx;
  for (xx = 0; xx < MAX_L2CAP_LINKS; xx++, p++) {
    if ((p->in_use) && p->remote_addr == bda && p->transport == transport) {
      BTM_TRACE_DEBUG("btm_bda_to_acl found");
      return (p);
    }
  }

  /* If here, no BD Addr found */
  return ((tACL_CONN*)NULL);
}

/*******************************************************************************
 *
 * Function         btm_handle_to_acl_index
 *
 * Description      This function returns the FIRST acl_db entry for the passed
 *                  hci_handle.
 *
 * Returns          index to the acl_db or MAX_L2CAP_LINKS.
 *
 ******************************************************************************/
uint8_t btm_handle_to_acl_index(uint16_t hci_handle) {
  tACL_CONN* p = &btm_cb.acl_cb_.acl_db[0];
  uint8_t xx;
  BTM_TRACE_DEBUG("btm_handle_to_acl_index");
  for (xx = 0; xx < MAX_L2CAP_LINKS; xx++, p++) {
    if ((p->in_use) && (p->hci_handle == hci_handle)) {
      break;
    }
  }

  /* If here, no BD Addr found */
  return (xx);
}

tACL_CONN* StackAclBtmAcl::acl_get_connection_from_handle(uint16_t hci_handle) {
  uint8_t index = btm_handle_to_acl_index(hci_handle);
  if (index >= MAX_L2CAP_LINKS) return nullptr;
  return &btm_cb.acl_cb_.acl_db[index];
}

/*******************************************************************************
 *
 * Function         btm_ble_get_acl_remote_addr
 *
 * Description      This function reads the active remote address used for the
 *                  connection.
 *
 * Returns          success return true, otherwise false.
 *
 ******************************************************************************/
static bool btm_ble_get_acl_remote_addr(tBTM_SEC_DEV_REC* p_dev_rec,
                                        RawAddress& conn_addr,
                                        tBLE_ADDR_TYPE* p_addr_type) {
  bool st = true;

  if (p_dev_rec == NULL) {
    BTM_TRACE_ERROR("%s can not find device with matching address", __func__);
    return false;
  }

  switch (p_dev_rec->ble.active_addr_type) {
    case tBTM_SEC_BLE::BTM_BLE_ADDR_PSEUDO:
      conn_addr = p_dev_rec->bd_addr;
      *p_addr_type = p_dev_rec->ble.ble_addr_type;
      break;

    case tBTM_SEC_BLE::BTM_BLE_ADDR_RRA:
      conn_addr = p_dev_rec->ble.cur_rand_addr;
      *p_addr_type = BLE_ADDR_RANDOM;
      break;

    case tBTM_SEC_BLE::BTM_BLE_ADDR_STATIC:
      conn_addr = p_dev_rec->ble.identity_address_with_type.bda;
      *p_addr_type = p_dev_rec->ble.identity_address_with_type.type;
      break;

    default:
      BTM_TRACE_ERROR("Unknown active address: %d",
                      p_dev_rec->ble.active_addr_type);
      st = false;
      break;
  }

  return st;
}

void btm_acl_process_sca_cmpl_pkt(uint8_t len, uint8_t* data) {
  uint16_t handle;
  uint8_t acl_idx;
  uint8_t sca;
  uint8_t status;
  tACL_CONN* p;

  STREAM_TO_UINT8(status, data);

  if (status != HCI_SUCCESS) {
    BTM_TRACE_DEBUG("%s, Peer SCA Command complete failed 0x%02x\n", __func__,
                    status);
    return;
  }

  STREAM_TO_UINT16(handle, data);
  STREAM_TO_UINT8(sca, data);

  acl_idx = btm_handle_to_acl_index(handle);
  if (acl_idx >= MAX_L2CAP_LINKS) {
    BTM_TRACE_DEBUG(
        "%s, Peer SCA Command complete: failed to find handle 0x%04x\n",
        __func__, handle);
    return;
  }

  p = &btm_cb.acl_cb_.acl_db[acl_idx];
  p->sca = sca;

  BTM_TRACE_DEBUG(
      "%s, Peer SCA Command complete: handle: 0x%04x, sca: 0x%02x\n", __func__,
      handle, sca);
}

/*******************************************************************************
 *
 * Function         btm_acl_created
 *
 * Description      This function is called by L2CAP when an ACL connection
 *                  is created.
 *
 * Returns          void
 *
 ******************************************************************************/
void acl_initialize_power_mode(const tACL_CONN& p_acl) {
  tBTM_PM_MCB* p_db =
      &btm_cb.acl_cb_.pm_mode_db[btm_handle_to_acl_index(p_acl.hci_handle)];
  memset(p_db, 0, sizeof(tBTM_PM_MCB));
  p_db->state = BTM_PM_ST_ACTIVE;
}

tACL_CONN* StackAclBtmAcl::acl_allocate_connection() {
  tACL_CONN* p_acl = &btm_cb.acl_cb_.acl_db[0];
  for (uint8_t xx = 0; xx < MAX_L2CAP_LINKS; xx++, p_acl++) {
    if (!p_acl->in_use) {
      return p_acl;
    }
  }
  return nullptr;
}

void btm_acl_created(const RawAddress& bda, uint16_t hci_handle,
                     uint8_t link_role, tBT_TRANSPORT transport) {

  tACL_CONN* p_acl = internal_.btm_bda_to_acl(bda, transport);
  if (p_acl != (tACL_CONN*)NULL) {
    p_acl->hci_handle = hci_handle;
    p_acl->link_role = link_role;
    p_acl->transport = transport;
    btm_set_link_policy(p_acl, btm_cb.acl_cb_.btm_def_link_policy);
    LOG_WARN(
        "Unable to create duplicate acl when one already exists handle:%hu"
        " role:%s transport:%s",
        hci_handle, RoleText(link_role).c_str(),
        BtTransportText(transport).c_str());
    return;
  }

  p_acl = internal_.acl_allocate_connection();
  if (p_acl == nullptr) {
    LOG_ERROR("Unable to find available acl resource to continue");
    return;
  }

  LOG_DEBUG("Acl allocated handle:%hu role:%s transport:%s", hci_handle,
            RoleText(link_role).c_str(), BtTransportText(transport).c_str());

  p_acl->in_use = true;
  p_acl->hci_handle = hci_handle;
  p_acl->link_role = link_role;
  p_acl->link_up_issued = false;
  p_acl->remote_addr = bda;
  p_acl->sca = 0xFF;
  btm_set_link_policy(p_acl, btm_cb.acl_cb_.btm_def_link_policy);

  p_acl->transport = transport;
  if (transport == BT_TRANSPORT_LE) {
    btm_ble_refresh_local_resolvable_private_addr(
        bda, btm_cb.ble_ctr_cb.addr_mgnt_cb.private_addr);
  }
  p_acl->switch_role_failed_attempts = 0;
  p_acl->reset_switch_role();

  acl_initialize_power_mode(*p_acl);

  /* if BR/EDR do something more */
  if (transport == BT_TRANSPORT_BR_EDR) {
    btsnd_hcic_read_rmt_clk_offset(hci_handle);
    btsnd_hcic_rmt_ver_req(hci_handle);
  }

  tBTM_SEC_DEV_REC* p_dev_rec = btm_find_dev_by_handle(hci_handle);
  if (p_dev_rec == nullptr) {
    LOG_ERROR("Unable to find security device record hci_handle:%hu",
              hci_handle);
    // TODO Release acl resource
    return;
  }

  if (transport != BT_TRANSPORT_LE) {
    /* If remote features already known, copy them and continue connection
     * setup */
    if ((p_dev_rec->num_read_pages) &&
        (p_dev_rec->num_read_pages <= (HCI_EXT_FEATURES_PAGE_MAX + 1))) {
      memcpy(p_acl->peer_lmp_feature_pages, p_dev_rec->feature_pages,
             (HCI_FEATURE_BYTES_PER_PAGE * p_dev_rec->num_read_pages));
      // TODO We do not need to store the pages read here
      p_acl->num_read_pages = p_dev_rec->num_read_pages;

      const uint8_t req_pend = (p_dev_rec->sm4 & BTM_SM4_REQ_PEND);

      /* Store the Peer Security Capabilites (in SM4 and rmt_sec_caps) */
      btm_sec_set_peer_sec_caps(p_acl, p_dev_rec);

      if (req_pend) {
        /* Request for remaining Security Features (if any) */
        l2cu_resubmit_pending_sec_req(&p_dev_rec->bd_addr);
      }
      internal_.btm_establish_continue(p_acl);
      return;
    }
  }

  if (transport == BT_TRANSPORT_LE) {
    btm_ble_get_acl_remote_addr(p_dev_rec, p_acl->active_remote_addr,
                                &p_acl->active_remote_addr_type);

    if (controller_get_interface()
            ->supports_ble_peripheral_initiated_feature_exchange() ||
        link_role == HCI_ROLE_MASTER) {
      btsnd_hcic_ble_read_remote_feat(p_acl->hci_handle);
    } else {
      internal_.btm_establish_continue(p_acl);
    }
  }
}

void btm_acl_update_conn_addr(uint16_t conn_handle, const RawAddress& address) {
  uint8_t idx = btm_handle_to_acl_index(conn_handle);
  if (idx != MAX_L2CAP_LINKS) {
    btm_cb.acl_cb_.acl_db[idx].conn_addr = address;
  }
}

/*******************************************************************************
 *
 * Function         btm_acl_removed
 *
 * Description      This function is called by L2CAP when an ACL connection
 *                  is removed. Since only L2CAP creates ACL links, we use
 *                  the L2CAP link index as our index into the control blocks.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_acl_removed(const RawAddress& bda, tBT_TRANSPORT transport) {
  tACL_CONN* p_acl = internal_.btm_bda_to_acl(bda, transport);
  if (p_acl == nullptr) {
    LOG_WARN("%s Tried to remove acl that does not exist", __func__);
    return;
  }
  p_acl->in_use = false;

  /* Only notify if link up has had a chance to be issued */
  if (p_acl->link_up_issued) {
    p_acl->link_up_issued = false;
    BTA_dm_acl_down(bda, transport);
  }

  LOG_DEBUG(
      "%s acl hci_handle=%d transport=%d connectable_mode=0x%0x link_role=%d",
      __func__, p_acl->hci_handle, p_acl->transport,
      btm_cb.ble_ctr_cb.inq_var.connectable_mode, p_acl->link_role);

  tBTM_SEC_DEV_REC* p_dev_rec = btm_find_dev(bda);
  if (p_dev_rec == nullptr) {
    LOG_WARN("%s Device security record not found", __func__);
  } else {
    if (p_acl->transport == BT_TRANSPORT_LE) {
      p_dev_rec->sec_flags &= ~(BTM_SEC_LE_ENCRYPTED | BTM_SEC_ROLE_SWITCHED);
      if ((p_dev_rec->sec_flags & BTM_SEC_LE_LINK_KEY_KNOWN) == 0) {
        p_dev_rec->sec_flags &=
            ~(BTM_SEC_LE_LINK_KEY_AUTHED | BTM_SEC_LE_AUTHENTICATED);
      }
    } else {
      p_dev_rec->sec_flags &=
          ~(BTM_SEC_AUTHENTICATED | BTM_SEC_ENCRYPTED | BTM_SEC_ROLE_SWITCHED);
    }
  }
  memset(p_acl, 0, sizeof(tACL_CONN));
}

/*******************************************************************************
 *
 * Function         btm_acl_device_down
 *
 * Description      This function is called when the local device is deemed
 *                  to be down. It notifies L2CAP of the failure.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_acl_device_down(void) {
  tACL_CONN* p = &btm_cb.acl_cb_.acl_db[0];
  uint16_t xx;
  BTM_TRACE_DEBUG("btm_acl_device_down");
  for (xx = 0; xx < MAX_L2CAP_LINKS; xx++, p++) {
    if (p->in_use) {
      BTM_TRACE_DEBUG("hci_handle=%d HCI_ERR_HW_FAILURE ", p->hci_handle);
      l2c_link_hci_disc_comp(p->hci_handle, HCI_ERR_HW_FAILURE);
    }
  }
}

void btm_acl_set_paging(bool value) { btm_cb.is_paging = value; }

void btm_acl_update_inquiry_status(uint8_t status) {
  btm_cb.is_inquiry = status == BTM_INQUIRY_STARTED;
  BTIF_dm_report_inquiry_status_change(status);
}

tBTM_STATUS BTM_GetRole(const RawAddress& remote_bd_addr, uint8_t* p_role) {
  if (p_role == nullptr) {
    return BTM_ILLEGAL_VALUE;
  }
  *p_role = HCI_ROLE_UNKNOWN;

  tACL_CONN* p_acl =
      internal_.btm_bda_to_acl(remote_bd_addr, BT_TRANSPORT_BR_EDR);
  if (p_acl == nullptr) {
    return BTM_UNKNOWN_ADDR;
  }
  *p_role = p_acl->link_role;
  return BTM_SUCCESS;
}

/*******************************************************************************
 *
 * Function         BTM_SwitchRole
 *
 * Description      This function is called to switch role between master and
 *                  slave.  If role is already set it will do nothing.
 *
 * Returns          BTM_SUCCESS if already in specified role.
 *                  BTM_CMD_STARTED if command issued to controller.
 *                  BTM_NO_RESOURCES if couldn't allocate memory to issue
 *                                   command
 *                  BTM_UNKNOWN_ADDR if no active link with bd addr specified
 *                  BTM_MODE_UNSUPPORTED if local device does not support role
 *                                       switching
 *                  BTM_BUSY if the previous command is not completed
 *
 ******************************************************************************/
tBTM_STATUS BTM_SwitchRole(const RawAddress& remote_bd_addr, uint8_t new_role) {
  if (!controller_get_interface()->supports_master_slave_role_switch()) {
    LOG_INFO("Local controller does not support role switching");
    return BTM_MODE_UNSUPPORTED;
  }

  tACL_CONN* p_acl =
      internal_.btm_bda_to_acl(remote_bd_addr, BT_TRANSPORT_BR_EDR);
  if (p_acl == nullptr) {
    LOG_INFO("Unable to find device for classic role switch");
    return BTM_UNKNOWN_ADDR;
  }

  if (p_acl->link_role == new_role) {
    LOG_INFO("Requested role is already in effect");
    return BTM_SUCCESS;
  }

  if (interop_match_addr(INTEROP_DISABLE_ROLE_SWITCH, &remote_bd_addr)) {
    LOG_INFO("Remote device is on list preventing role switch");
    return BTM_DEV_BLACKLISTED;
  }

  if (BTM_IsScoActiveByBdaddr(remote_bd_addr)) {
    LOG_INFO("An active SCO to device prevents role switch at this time");
    return BTM_NO_RESOURCES;
  }

  if (!p_acl->is_switch_role_idle()) {
    LOG_DEBUG("Role switch is already progress");
    return BTM_BUSY;
  }

  tBTM_PM_MODE pwr_mode;
  if (!BTM_ReadPowerMode(p_acl->remote_addr, &pwr_mode)) {
    LOG_WARN(
        "Unable to find device to read current power mode prior to role "
        "switch");
    return BTM_UNKNOWN_ADDR;
  };

  if (pwr_mode == BTM_PM_MD_PARK || pwr_mode == BTM_PM_MD_SNIFF) {
    tBTM_PM_PWR_MD settings;
    memset((void*)&settings, 0, sizeof(settings));
    settings.mode = BTM_PM_MD_ACTIVE;
    tBTM_STATUS status =
        BTM_SetPowerMode(BTM_PM_SET_ONLY_ID, p_acl->remote_addr, &settings);
    if (status != BTM_CMD_STARTED) {
      LOG_WARN("Unable to set power mode before attempting switch");
      return BTM_WRONG_MODE;
    }
    p_acl->set_switch_role_changing();
  }
  /* some devices do not support switch while encryption is on */
  else {
    tBTM_SEC_DEV_REC* p_dev_rec = btm_find_dev(remote_bd_addr);
    if ((p_dev_rec != NULL) &&
        ((p_dev_rec->sec_flags & BTM_SEC_ENCRYPTED) != 0) &&
        !BTM_EPR_AVAILABLE(p_acl)) {
      /* bypass turning off encryption if change link key is already doing it */
      p_acl->set_encryption_off();
      p_acl->set_switch_role_encryption_off();
    } else {
      btsnd_hcic_switch_role(remote_bd_addr, new_role);
      p_acl->set_switch_role_in_progress();
      if (p_dev_rec) p_dev_rec->rs_disc_pending = BTM_SEC_RS_PENDING;
    }
  }

  return BTM_CMD_STARTED;
}

/*******************************************************************************
 *
 * Function         btm_acl_encrypt_change
 *
 * Description      This function is when encryption of the connection is
 *                  completed by the LM.  Checks to see if a role switch or
 *                  change of link key was active and initiates or continues
 *                  process if needed.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_acl_encrypt_change(uint16_t handle, uint8_t status,
                            uint8_t encr_enable) {
  tACL_CONN* p;
  uint8_t xx;
  tBTM_SEC_DEV_REC* p_dev_rec;

  BTM_TRACE_DEBUG("btm_acl_encrypt_change handle=%d status=%d encr_enabl=%d",
                  handle, status, encr_enable);
  xx = btm_handle_to_acl_index(handle);
  /* don't assume that we can never get a bad hci_handle */
  if (xx < MAX_L2CAP_LINKS)
    p = &btm_cb.acl_cb_.acl_db[xx];
  else
    return;

  /* Process Role Switch if active */
  if (p->is_switch_role_encryption_off()) {
    /* if encryption turn off failed we still will try to switch role */
    if (encr_enable) {
      p->set_encryption_idle();
      p->reset_switch_role();
    } else {
      p->set_encryption_switching();
      p->set_switch_role_switching();
    }

    btsnd_hcic_switch_role(p->remote_addr, (uint8_t)!p->link_role);
    p_dev_rec = btm_find_dev(p->remote_addr);
    if (p_dev_rec != NULL) p_dev_rec->rs_disc_pending = BTM_SEC_RS_PENDING;

  }
  /* Finished enabling Encryption after role switch */
  else if (p->is_switch_role_encryption_on()) {
    p->reset_switch_role();
    p->set_encryption_idle();
    auto new_role = btm_cb.acl_cb_.switch_role_ref_data.role;
    auto hci_status = btm_cb.acl_cb_.switch_role_ref_data.hci_status;
    BTA_dm_report_role_change(
        btm_cb.acl_cb_.switch_role_ref_data.remote_bd_addr, new_role,
        hci_status);

    BTM_TRACE_DEBUG("%s: Role Switch Event: new_role 0x%02x, HCI Status 0x%02x",
                    __func__, new_role, hci_status);

    /* If a disconnect is pending, issue it now that role switch has completed
     */
    p_dev_rec = btm_find_dev(p->remote_addr);
    if (p_dev_rec != NULL) {
      if (p_dev_rec->rs_disc_pending == BTM_SEC_DISC_PENDING) {
        BTM_TRACE_WARNING(
            "btm_acl_encrypt_change -> Issuing delayed HCI_Disconnect!!!");
        btsnd_hcic_disconnect(p_dev_rec->hci_handle, HCI_ERR_PEER_USER);
      }
      BTM_TRACE_ERROR(
          "btm_acl_encrypt_change: tBTM_SEC_DEV:0x%x rs_disc_pending=%d",
          PTR_TO_UINT(p_dev_rec), p_dev_rec->rs_disc_pending);
      p_dev_rec->rs_disc_pending = BTM_SEC_RS_NOT_PENDING; /* reset flag */
    }
  }
}

void check_link_policy(uint16_t* settings) {
  const controller_t* controller = controller_get_interface();

  if ((*settings & HCI_ENABLE_MASTER_SLAVE_SWITCH) &&
      (!controller->supports_role_switch())) {
    *settings &= (~HCI_ENABLE_MASTER_SLAVE_SWITCH);
    BTM_TRACE_API("switch not supported (settings: 0x%04x)", *settings);
  }
  if ((*settings & HCI_ENABLE_HOLD_MODE) &&
      (!controller->supports_hold_mode())) {
    *settings &= (~HCI_ENABLE_HOLD_MODE);
    BTM_TRACE_API("hold not supported (settings: 0x%04x)", *settings);
  }
  if ((*settings & HCI_ENABLE_SNIFF_MODE) &&
      (!controller->supports_sniff_mode())) {
    *settings &= (~HCI_ENABLE_SNIFF_MODE);
    BTM_TRACE_API("sniff not supported (settings: 0x%04x)", *settings);
  }
  if ((*settings & HCI_ENABLE_PARK_MODE) &&
      (!controller->supports_park_mode())) {
    *settings &= (~HCI_ENABLE_PARK_MODE);
    BTM_TRACE_API("park not supported (settings: 0x%04x)", *settings);
  }
}

static void btm_set_link_policy(tACL_CONN* conn, uint16_t policy) {
  conn->link_policy = policy;
  check_link_policy(&conn->link_policy);
  btsnd_hcic_write_policy_set(conn->hci_handle, conn->link_policy);
}

static void btm_toggle_policy_on_for(const RawAddress& peer_addr,
                                     uint16_t flag) {
  auto conn = internal_.btm_bda_to_acl(peer_addr, BT_TRANSPORT_BR_EDR);
  if (!conn) {
    return;
  }
  btm_set_link_policy(conn, conn->link_policy | flag);
}

static void btm_toggle_policy_off_for(const RawAddress& peer_addr,
                                      uint16_t flag) {
  auto conn = internal_.btm_bda_to_acl(peer_addr, BT_TRANSPORT_BR_EDR);
  if (!conn) {
    return;
  }
  btm_set_link_policy(conn, conn->link_policy & ~flag);
}

bool BTM_is_sniff_allowed_for(const RawAddress& peer_addr) {
  auto conn = internal_.btm_bda_to_acl(peer_addr, BT_TRANSPORT_BR_EDR);
  if (!conn) {
    return false;
  }
  return conn->link_policy & HCI_ENABLE_SNIFF_MODE;
}

void BTM_unblock_sniff_mode_for(const RawAddress& peer_addr) {
  btm_toggle_policy_on_for(peer_addr, HCI_ENABLE_SNIFF_MODE);
}

void BTM_block_sniff_mode_for(const RawAddress& peer_addr) {
  btm_toggle_policy_off_for(peer_addr, HCI_ENABLE_SNIFF_MODE);
}

void BTM_unblock_role_switch_for(const RawAddress& peer_addr) {
  btm_toggle_policy_on_for(peer_addr, HCI_ENABLE_MASTER_SLAVE_SWITCH);
}

void BTM_block_role_switch_for(const RawAddress& peer_addr) {
  btm_toggle_policy_off_for(peer_addr, HCI_ENABLE_MASTER_SLAVE_SWITCH);
}

void StackAclBtmAcl::btm_set_default_link_policy(uint16_t settings) {
  check_link_policy(&settings);
  btm_cb.acl_cb_.btm_def_link_policy = settings;
  btsnd_hcic_write_def_policy_set(settings);
}

void BTM_default_unblock_role_switch() {
  internal_.btm_set_default_link_policy(btm_cb.acl_cb_.btm_def_link_policy |
                                        HCI_ENABLE_MASTER_SLAVE_SWITCH);
}

void BTM_default_block_role_switch() {
  internal_.btm_set_default_link_policy(btm_cb.acl_cb_.btm_def_link_policy &
                                        ~HCI_ENABLE_MASTER_SLAVE_SWITCH);
}

/*******************************************************************************
 *
 * Function         btm_read_remote_version_complete
 *
 * Description      This function is called when the command complete message
 *                  is received from the HCI for the remote version info.
 *
 * Returns          void
 *
 ******************************************************************************/
static void btm_process_remote_version_complete(uint8_t status, uint16_t handle,
                                                uint8_t lmp_version,
                                                uint16_t manufacturer,
                                                uint16_t lmp_subversion) {
  tACL_CONN* p_acl_cb = internal_.acl_get_connection_from_handle(handle);
  if (p_acl_cb == nullptr) {
    LOG_WARN("Received remote version complete for unknown device");
    return;
  }

  if (status == HCI_SUCCESS) {
    p_acl_cb->lmp_version = lmp_version;
    p_acl_cb->manufacturer = manufacturer;
    p_acl_cb->lmp_subversion = lmp_subversion;

    if (p_acl_cb->transport == BT_TRANSPORT_BR_EDR) {
      internal_.btm_read_remote_features(p_acl_cb->hci_handle);
    }
    bluetooth::common::LogRemoteVersionInfo(handle, status, lmp_version,
                                            manufacturer, lmp_subversion);
  } else {
    bluetooth::common::LogRemoteVersionInfo(handle, status, 0, 0, 0);
  }

  if (p_acl_cb->transport == BT_TRANSPORT_LE) {
    l2cble_notify_le_connection(p_acl_cb->remote_addr);
    l2cble_use_preferred_conn_params(p_acl_cb->remote_addr);
  }
}

void btm_read_remote_version_complete(uint8_t* p) {
  uint8_t status;
  uint16_t handle;
  uint8_t lmp_version;
  uint16_t manufacturer;
  uint16_t lmp_subversion;

  STREAM_TO_UINT8(status, p);
  STREAM_TO_UINT16(handle, p);
  STREAM_TO_UINT8(lmp_version, p);
  STREAM_TO_UINT16(manufacturer, p);
  STREAM_TO_UINT16(lmp_subversion, p);

  btm_process_remote_version_complete(status, handle, lmp_version, manufacturer,
                                      lmp_subversion);
}

/*******************************************************************************
 *
 * Function         btm_process_remote_ext_features
 *
 * Description      Local function called to process all extended features pages
 *                  read from a remote device.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_process_remote_ext_features(tACL_CONN* p_acl_cb,
                                     uint8_t num_read_pages) {
  uint16_t handle = p_acl_cb->hci_handle;
  tBTM_SEC_DEV_REC* p_dev_rec = btm_find_dev_by_handle(handle);
  uint8_t page_idx;

  BTM_TRACE_DEBUG("btm_process_remote_ext_features");

  /* Make sure we have the record to save remote features information */
  if (p_dev_rec == NULL) {
    /* Get a new device; might be doing dedicated bonding */
    p_dev_rec = btm_find_or_alloc_dev(p_acl_cb->remote_addr);
  }

  p_acl_cb->num_read_pages = num_read_pages;
  p_dev_rec->num_read_pages = num_read_pages;

  /* Move the pages to placeholder */
  for (page_idx = 0; page_idx < num_read_pages; page_idx++) {
    if (page_idx > HCI_EXT_FEATURES_PAGE_MAX) {
      BTM_TRACE_ERROR("%s: page=%d unexpected", __func__, page_idx);
      break;
    }
    memcpy(p_dev_rec->feature_pages[page_idx],
           p_acl_cb->peer_lmp_feature_pages[page_idx],
           HCI_FEATURE_BYTES_PER_PAGE);
  }

  if (!(p_dev_rec->sec_flags & BTM_SEC_NAME_KNOWN) ||
      p_dev_rec->is_originator) {
    BTM_TRACE_DEBUG("%s: Calling Next Security Procedure", __func__);
    uint8_t status = btm_sec_execute_procedure(p_dev_rec);
    if (status != BTM_CMD_STARTED) {
      BTM_TRACE_ERROR("%s: Security procedure not started! status %d", __func__,
                      status);
      btm_sec_dev_rec_cback_event(p_dev_rec, status, false);
    }
  }
  const uint8_t req_pend = (p_dev_rec->sm4 & BTM_SM4_REQ_PEND);

  /* Store the Peer Security Capabilites (in SM4 and rmt_sec_caps) */
  btm_sec_set_peer_sec_caps(p_acl_cb, p_dev_rec);

  BTM_TRACE_API("%s: pend:%d", __func__, req_pend);
  if (req_pend) {
    /* Request for remaining Security Features (if any) */
    l2cu_resubmit_pending_sec_req(&p_dev_rec->bd_addr);
  }
}

/*******************************************************************************
 *
 * Function         btm_read_remote_features
 *
 * Description      Local function called to send a read remote supported
 *                  features/remote extended features page[0].
 *
 * Returns          void
 *
 ******************************************************************************/
void StackAclBtmAcl::btm_read_remote_features(uint16_t handle) {
  uint8_t acl_idx;
  tACL_CONN* p_acl_cb;

  BTM_TRACE_DEBUG("btm_read_remote_features() handle: %d", handle);

  acl_idx = btm_handle_to_acl_index(handle);
  if (acl_idx >= MAX_L2CAP_LINKS) {
    BTM_TRACE_ERROR("btm_read_remote_features handle=%d invalid", handle);
    return;
  }

  p_acl_cb = &btm_cb.acl_cb_.acl_db[acl_idx];
  p_acl_cb->num_read_pages = 0;
  memset(p_acl_cb->peer_lmp_feature_pages, 0,
         sizeof(p_acl_cb->peer_lmp_feature_pages));

  /* first send read remote supported features HCI command */
  /* because we don't know whether the remote support extended feature command
   */
  btsnd_hcic_rmt_features_req(handle);
}

/*******************************************************************************
 *
 * Function         btm_read_remote_ext_features
 *
 * Description      Local function called to send a read remote extended
 *                  features
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_read_remote_ext_features(uint16_t handle, uint8_t page_number) {
  BTM_TRACE_DEBUG("btm_read_remote_ext_features() handle: %d page: %d", handle,
                  page_number);

  btsnd_hcic_rmt_ext_features(handle, page_number);
}

/*******************************************************************************
 *
 * Function         btm_read_remote_features_complete
 *
 * Description      This function is called when the remote supported features
 *                  complete event is received from the HCI.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_read_remote_features_complete(uint8_t* p) {
  tACL_CONN* p_acl_cb;
  uint8_t status;
  uint16_t handle;
  uint8_t acl_idx;

  BTM_TRACE_DEBUG("btm_read_remote_features_complete");
  STREAM_TO_UINT8(status, p);

  if (status != HCI_SUCCESS) {
    BTM_TRACE_ERROR("btm_read_remote_features_complete failed (status 0x%02x)",
                    status);
    return;
  }

  STREAM_TO_UINT16(handle, p);

  acl_idx = btm_handle_to_acl_index(handle);
  if (acl_idx >= MAX_L2CAP_LINKS) {
    BTM_TRACE_ERROR("btm_read_remote_features_complete handle=%d invalid",
                    handle);
    return;
  }

  p_acl_cb = &btm_cb.acl_cb_.acl_db[acl_idx];

  /* Copy the received features page */
  STREAM_TO_ARRAY(p_acl_cb->peer_lmp_feature_pages[0], p,
                  HCI_FEATURE_BYTES_PER_PAGE);

  if ((HCI_LMP_EXTENDED_SUPPORTED(p_acl_cb->peer_lmp_feature_pages[0])) &&
      (controller_get_interface()
           ->supports_reading_remote_extended_features())) {
    /* if the remote controller has extended features and local controller
       supports HCI_Read_Remote_Extended_Features command then start reading
       these feature starting with extended features page 1 */
    BTM_TRACE_DEBUG("Start reading remote extended features");
    btm_read_remote_ext_features(handle, 1);
    return;
  }

  /* Remote controller has no extended features. Process remote controller
     supported features (features page 0). */
  btm_process_remote_ext_features(p_acl_cb, 1);

  /* Continue with HCI connection establishment */
  internal_.btm_establish_continue(p_acl_cb);
}

/*******************************************************************************
 *
 * Function         btm_read_remote_ext_features_complete
 *
 * Description      This function is called when the remote extended features
 *                  complete event is received from the HCI.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_read_remote_ext_features_complete(uint8_t* p, uint8_t evt_len) {
  tACL_CONN* p_acl_cb;
  uint8_t page_num, max_page;
  uint16_t handle;
  uint8_t acl_idx;

  BTM_TRACE_DEBUG("btm_read_remote_ext_features_complete");

  if (evt_len < HCI_EXT_FEATURES_SUCCESS_EVT_LEN) {
    android_errorWriteLog(0x534e4554, "141552859");
    BTM_TRACE_ERROR(
        "btm_read_remote_ext_features_complete evt length too short. length=%d",
        evt_len);
    return;
  }

  ++p;
  STREAM_TO_UINT16(handle, p);
  STREAM_TO_UINT8(page_num, p);
  STREAM_TO_UINT8(max_page, p);

  /* Validate parameters */
  acl_idx = btm_handle_to_acl_index(handle);
  if (acl_idx >= MAX_L2CAP_LINKS) {
    BTM_TRACE_ERROR("btm_read_remote_ext_features_complete handle=%d invalid",
                    handle);
    return;
  }

  if (max_page > HCI_EXT_FEATURES_PAGE_MAX) {
    BTM_TRACE_ERROR("btm_read_remote_ext_features_complete page=%d unknown",
                    max_page);
    return;
  }

  if (page_num > HCI_EXT_FEATURES_PAGE_MAX) {
    android_errorWriteLog(0x534e4554, "141552859");
    BTM_TRACE_ERROR("btm_read_remote_ext_features_complete num_page=%d invalid",
                    page_num);
    return;
  }

  if (page_num > max_page) {
    BTM_TRACE_WARNING(
        "btm_read_remote_ext_features_complete num_page=%d, max_page=%d "
        "invalid",
        page_num, max_page);
  }

  p_acl_cb = &btm_cb.acl_cb_.acl_db[acl_idx];

  /* Copy the received features page */
  STREAM_TO_ARRAY(p_acl_cb->peer_lmp_feature_pages[page_num], p,
                  HCI_FEATURE_BYTES_PER_PAGE);

  /* If there is the next remote features page and
   * we have space to keep this page data - read this page */
  if ((page_num < max_page) && (page_num < HCI_EXT_FEATURES_PAGE_MAX)) {
    page_num++;
    BTM_TRACE_DEBUG("BTM reads next remote extended features page (%d)",
                    page_num);
    btm_read_remote_ext_features(handle, page_num);
    return;
  }

  /* Reading of remote feature pages is complete */
  BTM_TRACE_DEBUG("BTM reached last remote extended features page (%d)",
                  page_num);

  /* Process the pages */
  btm_process_remote_ext_features(p_acl_cb, (uint8_t)(page_num + 1));

  /* Continue with HCI connection establishment */
  internal_.btm_establish_continue(p_acl_cb);
}

/*******************************************************************************
 *
 * Function         btm_read_remote_ext_features_failed
 *
 * Description      This function is called when the remote extended features
 *                  complete event returns a failed status.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_read_remote_ext_features_failed(uint8_t status, uint16_t handle) {
  tACL_CONN* p_acl_cb;
  uint8_t acl_idx;

  BTM_TRACE_WARNING(
      "btm_read_remote_ext_features_failed (status 0x%02x) for handle %d",
      status, handle);

  acl_idx = btm_handle_to_acl_index(handle);
  if (acl_idx >= MAX_L2CAP_LINKS) {
    BTM_TRACE_ERROR("btm_read_remote_ext_features_failed handle=%d invalid",
                    handle);
    return;
  }

  p_acl_cb = &btm_cb.acl_cb_.acl_db[acl_idx];

  /* Process supported features only */
  btm_process_remote_ext_features(p_acl_cb, 1);

  /* Continue HCI connection establishment */
  internal_.btm_establish_continue(p_acl_cb);
}

/*******************************************************************************
 *
 * Function         btm_establish_continue
 *
 * Description      This function is called when the command complete message
 *                  is received from the HCI for the read local link policy
 *                  request.
 *
 * Returns          void
 *
 ******************************************************************************/
void StackAclBtmAcl::btm_establish_continue(tACL_CONN* p_acl_cb) {
  BTM_TRACE_DEBUG("btm_establish_continue");
  if (p_acl_cb->transport == BT_TRANSPORT_BR_EDR) {
    /* For now there are a some devices that do not like sending */
    /* commands events and data at the same time. */
    /* Set the packet types to the default allowed by the device */
    internal_.btm_set_packet_types(p_acl_cb,
                                   btm_cb.acl_cb_.btm_acl_pkt_types_supported);
    btm_set_link_policy(p_acl_cb, btm_cb.acl_cb_.btm_def_link_policy);
  }
  if (p_acl_cb->link_up_issued) {
    BTM_TRACE_ERROR("%s: Already link is up ", __func__);
    return;
  }
  p_acl_cb->link_up_issued = true;

  BTA_dm_acl_up(p_acl_cb->remote_addr, p_acl_cb->transport);
}

void btm_establish_continue_from_address(const RawAddress& bda,
                                         tBT_TRANSPORT transport) {
  tACL_CONN* p_acl = internal_.btm_bda_to_acl(bda, transport);
  if (p_acl == nullptr) {
    BTM_TRACE_ERROR("%s Unable to find acl control block to continue",
                    __func__);
    return;
  }
  internal_.btm_establish_continue(p_acl);
}

/*******************************************************************************
 *
 * Function         BTM_SetDefaultLinkSuperTout
 *
 * Description      Set the default value for HCI "Write Link Supervision
 *                                                 Timeout"
 *                  command to use when an ACL link is created.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTM_SetDefaultLinkSuperTout(uint16_t timeout) {
  BTM_TRACE_DEBUG("BTM_SetDefaultLinkSuperTout");
  btm_cb.acl_cb_.btm_def_link_super_tout = timeout;
}

/*******************************************************************************
 *
 * Function         BTM_GetLinkSuperTout
 *
 * Description      Read the link supervision timeout value of the connection
 *
 * Returns          status of the operation
 *
 ******************************************************************************/
tBTM_STATUS BTM_GetLinkSuperTout(const RawAddress& remote_bda,
                                 uint16_t* p_timeout) {
  tACL_CONN* p = internal_.btm_bda_to_acl(remote_bda, BT_TRANSPORT_BR_EDR);

  BTM_TRACE_DEBUG("BTM_GetLinkSuperTout");
  if (p != (tACL_CONN*)NULL) {
    *p_timeout = p->link_super_tout;
    return (BTM_SUCCESS);
  }
  /* If here, no BD Addr found */
  return (BTM_UNKNOWN_ADDR);
}

/*******************************************************************************
 *
 * Function         BTM_SetLinkSuperTout
 *
 * Description      Create and send HCI "Write Link Supervision Timeout" command
 *
 * Returns          status of the operation
 *
 ******************************************************************************/
tBTM_STATUS BTM_SetLinkSuperTout(const RawAddress& remote_bda,
                                 uint16_t timeout) {
  tACL_CONN* p = internal_.btm_bda_to_acl(remote_bda, BT_TRANSPORT_BR_EDR);

  BTM_TRACE_DEBUG("BTM_SetLinkSuperTout");
  if (p != (tACL_CONN*)NULL) {
    p->link_super_tout = timeout;

    /* Only send if current role is Master; 2.0 spec requires this */
    if (p->link_role == HCI_ROLE_MASTER) {
      btsnd_hcic_write_link_super_tout(LOCAL_BR_EDR_CONTROLLER_ID,
                                       p->hci_handle, timeout);
      return (BTM_CMD_STARTED);
    } else {
      return (BTM_SUCCESS);
    }
  }

  /* If here, no BD Addr found */
  return (BTM_UNKNOWN_ADDR);
}

/*******************************************************************************
 *
 * Function         BTM_IsAclConnectionUp
 *
 * Description      This function is called to check if an ACL connection exists
 *                  to a specific remote BD Address.
 *
 * Returns          true if connection is up, else false.
 *
 ******************************************************************************/
bool BTM_IsAclConnectionUp(const RawAddress& remote_bda,
                           tBT_TRANSPORT transport) {
  tACL_CONN* p;

  VLOG(2) << __func__ << " RemBdAddr: " << remote_bda;

  p = internal_.btm_bda_to_acl(remote_bda, transport);
  if (p != (tACL_CONN*)NULL) {
    return (true);
  }

  /* If here, no BD Addr found */
  return (false);
}

bool BTM_IsAclConnectionUpAndHandleValid(const RawAddress& remote_bda,
                                         tBT_TRANSPORT transport) {
  tACL_CONN* p_acl = internal_.btm_bda_to_acl(remote_bda, transport);
  if (p_acl == nullptr) {
    return false;
  }
  return p_acl->hci_handle != HCI_INVALID_HANDLE;
}

bool BTM_IsAclConnectionUpFromHandle(uint16_t hci_handle) {
  return internal_.acl_get_connection_from_handle(hci_handle) != nullptr;
}

/*******************************************************************************
 *
 * Function         BTM_GetNumAclLinks
 *
 * Description      This function is called to count the number of
 *                  ACL links that are active.
 *
 * Returns          uint16_t Number of active ACL links
 *
 ******************************************************************************/
uint16_t BTM_GetNumAclLinks(void) {
  uint16_t num_acl = 0;

  for (uint16_t i = 0; i < MAX_L2CAP_LINKS; ++i) {
    if (btm_cb.acl_cb_.acl_db[i].in_use) ++num_acl;
  }

  return num_acl;
}

/*******************************************************************************
 *
 * Function         btm_get_acl_disc_reason_code
 *
 * Description      This function is called to get the disconnection reason code
 *                  returned by the HCI at disconnection complete event.
 *
 * Returns          true if connection is up, else false.
 *
 ******************************************************************************/
uint16_t btm_get_acl_disc_reason_code(void) {
  uint8_t res = btm_cb.acl_cb_.acl_disc_reason;
  LOG_WARN("%s This API should require an address for per ACL basis", __func__);
  return res;
}

/*******************************************************************************
 *
 * Function         BTM_GetHCIConnHandle
 *
 * Description      This function is called to get the handle for an ACL
 *                  connection to a specific remote BD Address.
 *
 * Returns          the handle of the connection, or HCI_INVALID_HANDLE if none.
 *
 ******************************************************************************/
uint16_t BTM_GetHCIConnHandle(const RawAddress& remote_bda,
                              tBT_TRANSPORT transport) {
  if (bluetooth::shim::is_gd_shim_enabled()) {
    return bluetooth::shim::BTM_GetHCIConnHandle(remote_bda, transport);
  }

  tACL_CONN* p;
  BTM_TRACE_DEBUG("BTM_GetHCIConnHandle");
  p = internal_.btm_bda_to_acl(remote_bda, transport);
  if (p != (tACL_CONN*)NULL) {
    return (p->hci_handle);
  }

  /* If here, no BD Addr found */
  return HCI_INVALID_HANDLE;
}

/*******************************************************************************
 *
 * Function         BTM_RequestPeerSCA
 *
 * Description      This function is called to request sleep clock accuracy
 *                  from peer device
 *
 ******************************************************************************/
void BTM_RequestPeerSCA(const RawAddress& remote_bda, tBT_TRANSPORT transport) {
  tACL_CONN* p;
  BTM_TRACE_DEBUG("BTM_RequestPeerSCA");
  p = internal_.btm_bda_to_acl(remote_bda, transport);
  if (p == (tACL_CONN*)NULL) {
    BTM_TRACE_DEBUG("BTM_RequestPeerSCA: no connection");
    return;
  }

  btsnd_hcic_req_peer_sca(p->hci_handle);
}

/*******************************************************************************
 *
 * Function         BTM_GetPeerSCA
 *
 * Description      This function is called to get peer sleep clock accuracy
 *
 * Returns          SCA or 0xFF if SCA was never previously requested, request
 *                  is not supported by peer device or ACL does not exist
 *
 ******************************************************************************/
uint8_t BTM_GetPeerSCA(const RawAddress& remote_bda, tBT_TRANSPORT transport) {
  tACL_CONN* p;
  BTM_TRACE_DEBUG("BTM_GetPeerSCA");
  p = internal_.btm_bda_to_acl(remote_bda, transport);
  if (p != (tACL_CONN*)NULL) {
    return (p->sca);
  }

  /* If here, no BD Addr found */
  return (0xFF);
}

/*******************************************************************************
 *
 * Function         btm_process_clk_off_comp_evt
 *
 * Description      This function is called when clock offset command completes.
 *
 * Input Parms      hci_handle - connection handle associated with the change
 *                  clock offset
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_process_clk_off_comp_evt(uint16_t hci_handle, uint16_t clock_offset) {
  uint8_t xx;
  BTM_TRACE_DEBUG("btm_process_clk_off_comp_evt");
  /* Look up the connection by handle and set the current mode */
  xx = btm_handle_to_acl_index(hci_handle);
  if (xx < MAX_L2CAP_LINKS)
    btm_cb.acl_cb_.acl_db[xx].clock_offset = clock_offset;
}

/*******************************************************************************
 *
 * Function         btm_blacklist_role_change_device
 *
 * Description      This function is used to blacklist the device if the role
 *                  switch fails for maximum number of times. It also removes
 *                  the device from the black list if the role switch succeeds.
 *
 * Input Parms      bd_addr - remote BD addr
 *                  hci_status - role switch status
 *
 * Returns          void
 *
 *******************************************************************************/
void btm_blacklist_role_change_device(const RawAddress& bd_addr,
                                      uint8_t hci_status) {
  tACL_CONN* p = internal_.btm_bda_to_acl(bd_addr, BT_TRANSPORT_BR_EDR);
  tBTM_SEC_DEV_REC* p_dev_rec = btm_find_dev(bd_addr);

  if (!p || !p_dev_rec) {
    return;
  }
  if (hci_status == HCI_SUCCESS) {
    p->switch_role_failed_attempts = 0;
    return;
  }

  /* check for carkits */
  const uint32_t cod_audio_device =
      (BTM_COD_SERVICE_AUDIO | BTM_COD_MAJOR_AUDIO) << 8;
  const uint32_t cod =
      ((p_dev_rec->dev_class[0] << 16) | (p_dev_rec->dev_class[1] << 8) |
       p_dev_rec->dev_class[2]) &
      0xffffff;
  if ((hci_status != HCI_SUCCESS) &&
      (p->is_switch_role_switching_or_in_progress()) &&
      ((cod & cod_audio_device) == cod_audio_device) &&
      (!interop_match_addr(INTEROP_DYNAMIC_ROLE_SWITCH, &bd_addr))) {
    p->switch_role_failed_attempts++;
    if (p->switch_role_failed_attempts == BTM_MAX_SW_ROLE_FAILED_ATTEMPTS) {
      BTM_TRACE_WARNING(
          "%s: Device %s blacklisted for role switching - "
          "multiple role switch failed attempts: %u",
          __func__, bd_addr.ToString().c_str(), p->switch_role_failed_attempts);
      interop_database_add(INTEROP_DYNAMIC_ROLE_SWITCH, &bd_addr, 3);
    }
  }
}

/*******************************************************************************
 *
 * Function         btm_acl_role_changed
 *
 * Description      This function is called whan a link's master/slave role
 *                  change event or command status event (with error) is
 *                  received. It updates the link control block, and calls the
 *                  registered callback with status and role (if registered).
 *
 * Returns          void
 *
 ******************************************************************************/
void StackAclBtmAcl::btm_acl_role_changed(uint8_t hci_status,
                                          const RawAddress& bd_addr,
                                          uint8_t new_role) {
  tACL_CONN* p_acl = internal_.btm_bda_to_acl(bd_addr, BT_TRANSPORT_BR_EDR);
  if (p_acl == nullptr) {
    BTM_TRACE_WARNING("%s: Unsolicited role change for unknown ACL", __func__);
    return;
  }

  tBTM_ROLE_SWITCH_CMPL* p_switch_role = &btm_cb.acl_cb_.switch_role_ref_data;
  BTM_TRACE_DEBUG("%s: peer %s hci_status:0x%x new_role:%d", __func__,
                  bd_addr.ToString().c_str(), hci_status, new_role);

  p_switch_role->hci_status = hci_status;
  if (hci_status == HCI_SUCCESS) {
    p_switch_role->role = new_role;
    p_switch_role->remote_bd_addr = bd_addr;

    /* Update cached value */
    p_acl->link_role = new_role;

    /* Reload LSTO: link supervision timeout is reset in the LM after a role
     * switch */
    if (new_role == HCI_ROLE_MASTER) {
      BTM_SetLinkSuperTout(p_acl->remote_addr, p_acl->link_super_tout);
    }
  } else {
    new_role = p_acl->link_role;
  }

  /* Check if any SCO req is pending for role change */
  btm_sco_chk_pend_rolechange(p_acl->hci_handle);

  /* if switching state is switching we need to turn encryption on */
  /* if idle, we did not change encryption */
  if (p_acl->is_switch_role_switching()) {
    p_acl->set_encryption_on();
    p_acl->set_switch_role_encryption_on();
    return;
  }

  /* Set the switch_role_state to IDLE since the reply received from HCI */
  /* regardless of its result either success or failed. */
  if (p_acl->is_switch_role_in_progress()) {
    p_acl->set_encryption_idle();
    p_acl->reset_switch_role();
  }

  BTA_dm_report_role_change(bd_addr, new_role, hci_status);
  BTM_TRACE_DEBUG(
      "%s: peer %s Role Switch Event: new_role 0x%02x, HCI Status 0x%02x, ",
      __func__, bd_addr.ToString().c_str(), new_role, hci_status);

  /* If a disconnect is pending, issue it now that role switch has completed */
  tBTM_SEC_DEV_REC* p_dev_rec = btm_find_dev(bd_addr);
  if (p_dev_rec != nullptr) {
    if (p_dev_rec->rs_disc_pending == BTM_SEC_DISC_PENDING) {
      BTM_TRACE_WARNING("%s peer %s Issuing delayed HCI_Disconnect!!!",
                        __func__, bd_addr.ToString().c_str());
      btsnd_hcic_disconnect(p_dev_rec->hci_handle, HCI_ERR_PEER_USER);
    }
    BTM_TRACE_ERROR("%s: peer %s rs_disc_pending=%d", __func__,
                    bd_addr.ToString().c_str(), p_dev_rec->rs_disc_pending);
    p_dev_rec->rs_disc_pending = BTM_SEC_RS_NOT_PENDING; /* reset flag */
  }
}

void btm_acl_role_changed(uint8_t hci_status, const RawAddress& bd_addr,
                          uint8_t new_role) {
  if (hci_status == HCI_SUCCESS) {
    l2c_link_role_changed(&bd_addr, new_role, hci_status);
  } else {
    l2c_link_role_changed(nullptr, HCI_ROLE_UNKNOWN,
                          HCI_ERR_COMMAND_DISALLOWED);
  }
  internal_.btm_acl_role_changed(hci_status, bd_addr, new_role);
}

/*******************************************************************************
 *
 * Function         btm_set_packet_types
 *
 * Description      This function sets the packet types used for a specific
 *                  ACL connection. It is called internally by btm_acl_created
 *                  or by an application/profile by BTM_SetPacketTypes.
 *
 * Returns          status of the operation
 *
 ******************************************************************************/
tBTM_STATUS StackAclBtmAcl::btm_set_packet_types(tACL_CONN* p,
                                                 uint16_t pkt_types) {
  uint16_t temp_pkt_types;
  BTM_TRACE_DEBUG("btm_set_packet_types");
  /* Save in the ACL control blocks, types that we support */
  temp_pkt_types = (pkt_types & BTM_ACL_SUPPORTED_PKTS_MASK &
                    btm_cb.acl_cb_.btm_acl_pkt_types_supported);

  /* OR in any exception packet types if at least 2.0 version of spec */
  temp_pkt_types |= ((pkt_types & BTM_ACL_EXCEPTION_PKTS_MASK) |
                     (btm_cb.acl_cb_.btm_acl_pkt_types_supported &
                      BTM_ACL_EXCEPTION_PKTS_MASK));

  /* Exclude packet types not supported by the peer */
  btm_acl_chk_peer_pkt_type_support(p, &temp_pkt_types);

  BTM_TRACE_DEBUG("SetPacketType Mask -> 0x%04x", temp_pkt_types);

  btsnd_hcic_change_conn_type(p->hci_handle, temp_pkt_types);
  p->pkt_types_mask = temp_pkt_types;

  return (BTM_CMD_STARTED);
}

void btm_set_packet_types_from_address(const RawAddress& bd_addr,
                                       tBT_TRANSPORT transport,
                                       uint16_t pkt_types) {
  tACL_CONN* p_acl_cb = internal_.btm_bda_to_acl(bd_addr, transport);
  if (p_acl_cb == nullptr) {
    BTM_TRACE_ERROR("%s Unable to find acl for address", __func__);
    return;
  }
  tBTM_STATUS status = internal_.btm_set_packet_types(p_acl_cb, pkt_types);
  if (status != BTM_CMD_STARTED) {
    BTM_TRACE_ERROR("%s unable to set packet types from address", __func__);
  }
}

/*******************************************************************************
 *
 * Function         BTM_GetMaxPacketSize
 *
 * Returns          Returns maximum packet size that can be used for current
 *                  connection, 0 if connection is not established
 *
 ******************************************************************************/
uint16_t BTM_GetMaxPacketSize(const RawAddress& addr) {
  tACL_CONN* p = internal_.btm_bda_to_acl(addr, BT_TRANSPORT_BR_EDR);
  uint16_t pkt_types = 0;
  uint16_t pkt_size = 0;
  BTM_TRACE_DEBUG("BTM_GetMaxPacketSize");
  if (p != NULL) {
    pkt_types = p->pkt_types_mask;
  } else {
    /* Special case for when info for the local device is requested */
    if (addr == *controller_get_interface()->get_address()) {
      pkt_types = btm_cb.acl_cb_.btm_acl_pkt_types_supported;
    }
  }

  if (pkt_types) {
    if (!(pkt_types & HCI_PKT_TYPES_MASK_NO_3_DH5))
      pkt_size = HCI_EDR3_DH5_PACKET_SIZE;
    else if (!(pkt_types & HCI_PKT_TYPES_MASK_NO_2_DH5))
      pkt_size = HCI_EDR2_DH5_PACKET_SIZE;
    else if (!(pkt_types & HCI_PKT_TYPES_MASK_NO_3_DH3))
      pkt_size = HCI_EDR3_DH3_PACKET_SIZE;
    else if (pkt_types & HCI_PKT_TYPES_MASK_DH5)
      pkt_size = HCI_DH5_PACKET_SIZE;
    else if (!(pkt_types & HCI_PKT_TYPES_MASK_NO_2_DH3))
      pkt_size = HCI_EDR2_DH3_PACKET_SIZE;
    else if (pkt_types & HCI_PKT_TYPES_MASK_DM5)
      pkt_size = HCI_DM5_PACKET_SIZE;
    else if (pkt_types & HCI_PKT_TYPES_MASK_DH3)
      pkt_size = HCI_DH3_PACKET_SIZE;
    else if (pkt_types & HCI_PKT_TYPES_MASK_DM3)
      pkt_size = HCI_DM3_PACKET_SIZE;
    else if (!(pkt_types & HCI_PKT_TYPES_MASK_NO_3_DH1))
      pkt_size = HCI_EDR3_DH1_PACKET_SIZE;
    else if (!(pkt_types & HCI_PKT_TYPES_MASK_NO_2_DH1))
      pkt_size = HCI_EDR2_DH1_PACKET_SIZE;
    else if (pkt_types & HCI_PKT_TYPES_MASK_DH1)
      pkt_size = HCI_DH1_PACKET_SIZE;
    else if (pkt_types & HCI_PKT_TYPES_MASK_DM1)
      pkt_size = HCI_DM1_PACKET_SIZE;
  }

  return (pkt_size);
}

/*******************************************************************************
 *
 * Function         BTM_ReadRemoteVersion
 *
 * Returns          If connected report peer device info
 *
 ******************************************************************************/
tBTM_STATUS BTM_ReadRemoteVersion(const RawAddress& addr, uint8_t* lmp_version,
                                  uint16_t* manufacturer,
                                  uint16_t* lmp_sub_version) {
  tACL_CONN* p = internal_.btm_bda_to_acl(addr, BT_TRANSPORT_BR_EDR);
  BTM_TRACE_DEBUG("BTM_ReadRemoteVersion");
  if (p == NULL) return (BTM_UNKNOWN_ADDR);

  if (lmp_version) *lmp_version = p->lmp_version;

  if (manufacturer) *manufacturer = p->manufacturer;

  if (lmp_sub_version) *lmp_sub_version = p->lmp_subversion;

  return (BTM_SUCCESS);
}

/*******************************************************************************
 *
 * Function         BTM_ReadRemoteFeatures
 *
 * Returns          pointer to the remote supported features mask (8 bytes)
 *
 ******************************************************************************/
uint8_t* BTM_ReadRemoteFeatures(const RawAddress& addr) {
  tACL_CONN* p = internal_.btm_bda_to_acl(addr, BT_TRANSPORT_BR_EDR);
  BTM_TRACE_DEBUG("BTM_ReadRemoteFeatures");
  if (p == NULL) {
    return (NULL);
  }

  return (p->peer_lmp_feature_pages[0]);
}

/*******************************************************************************
 *
 * Function         BTM_ReadRSSI
 *
 * Description      This function is called to read the link policy settings.
 *                  The address of link policy results are returned in the
 *                  callback.
 *                  (tBTM_RSSI_RESULT)
 *
 * Returns          BTM_CMD_STARTED if successfully initiated or error code
 *
 ******************************************************************************/
tBTM_STATUS BTM_ReadRSSI(const RawAddress& remote_bda, tBTM_CMPL_CB* p_cb) {
  tACL_CONN* p = NULL;
  tBT_DEVICE_TYPE dev_type;
  tBLE_ADDR_TYPE addr_type;

  /* If someone already waiting on the version, do not allow another */
  if (btm_cb.devcb.p_rssi_cmpl_cb) return (BTM_BUSY);

  BTM_ReadDevInfo(remote_bda, &dev_type, &addr_type);

  if (dev_type & BT_DEVICE_TYPE_BLE) {
    p = internal_.btm_bda_to_acl(remote_bda, BT_TRANSPORT_LE);
  }

  if (p == NULL && dev_type & BT_DEVICE_TYPE_BREDR) {
    p = internal_.btm_bda_to_acl(remote_bda, BT_TRANSPORT_BR_EDR);
  }

  if (p) {
    btm_cb.devcb.p_rssi_cmpl_cb = p_cb;
    alarm_set_on_mloop(btm_cb.devcb.read_rssi_timer, BTM_DEV_REPLY_TIMEOUT_MS,
                       btm_read_rssi_timeout, NULL);

    btsnd_hcic_read_rssi(p->hci_handle);
    return (BTM_CMD_STARTED);
  }

  /* If here, no BD Addr found */
  return (BTM_UNKNOWN_ADDR);
}

/*******************************************************************************
 *
 * Function         BTM_ReadFailedContactCounter
 *
 * Description      This function is called to read the failed contact counter.
 *                  The result is returned in the callback.
 *                  (tBTM_FAILED_CONTACT_COUNTER_RESULT)
 *
 * Returns          BTM_CMD_STARTED if successfully initiated or error code
 *
 ******************************************************************************/
tBTM_STATUS BTM_ReadFailedContactCounter(const RawAddress& remote_bda,
                                         tBTM_CMPL_CB* p_cb) {
  tACL_CONN* p;
  tBT_TRANSPORT transport = BT_TRANSPORT_BR_EDR;
  tBT_DEVICE_TYPE dev_type;
  tBLE_ADDR_TYPE addr_type;

  /* If someone already waiting on the result, do not allow another */
  if (btm_cb.devcb.p_failed_contact_counter_cmpl_cb) return (BTM_BUSY);

  BTM_ReadDevInfo(remote_bda, &dev_type, &addr_type);
  if (dev_type == BT_DEVICE_TYPE_BLE) transport = BT_TRANSPORT_LE;

  p = internal_.btm_bda_to_acl(remote_bda, transport);
  if (p != (tACL_CONN*)NULL) {
    btm_cb.devcb.p_failed_contact_counter_cmpl_cb = p_cb;
    alarm_set_on_mloop(btm_cb.devcb.read_failed_contact_counter_timer,
                       BTM_DEV_REPLY_TIMEOUT_MS,
                       btm_read_failed_contact_counter_timeout, NULL);

    btsnd_hcic_read_failed_contact_counter(p->hci_handle);
    return (BTM_CMD_STARTED);
  }

  /* If here, no BD Addr found */
  return (BTM_UNKNOWN_ADDR);
}

/*******************************************************************************
 *
 * Function         BTM_ReadAutomaticFlushTimeout
 *
 * Description      This function is called to read the automatic flush timeout.
 *                  The result is returned in the callback.
 *                  (tBTM_AUTOMATIC_FLUSH_TIMEOUT_RESULT)
 *
 * Returns          BTM_CMD_STARTED if successfully initiated or error code
 *
 ******************************************************************************/
tBTM_STATUS BTM_ReadAutomaticFlushTimeout(const RawAddress& remote_bda,
                                          tBTM_CMPL_CB* p_cb) {
  tACL_CONN* p;
  tBT_TRANSPORT transport = BT_TRANSPORT_BR_EDR;
  tBT_DEVICE_TYPE dev_type;
  tBLE_ADDR_TYPE addr_type;

  /* If someone already waiting on the result, do not allow another */
  if (btm_cb.devcb.p_automatic_flush_timeout_cmpl_cb) return (BTM_BUSY);

  BTM_ReadDevInfo(remote_bda, &dev_type, &addr_type);
  if (dev_type == BT_DEVICE_TYPE_BLE) transport = BT_TRANSPORT_LE;

  p = internal_.btm_bda_to_acl(remote_bda, transport);
  if (!p) return BTM_UNKNOWN_ADDR;

  btm_cb.devcb.p_automatic_flush_timeout_cmpl_cb = p_cb;
  alarm_set_on_mloop(btm_cb.devcb.read_automatic_flush_timeout_timer,
                     BTM_DEV_REPLY_TIMEOUT_MS,
                     btm_read_automatic_flush_timeout_timeout, nullptr);

  btsnd_hcic_read_automatic_flush_timeout(p->hci_handle);
  return BTM_CMD_STARTED;
}

/*******************************************************************************
 *
 * Function         BTM_ReadTxPower
 *
 * Description      This function is called to read the current
 *                  TX power of the connection. The tx power level results
 *                  are returned in the callback.
 *                  (tBTM_RSSI_RESULT)
 *
 * Returns          BTM_CMD_STARTED if successfully initiated or error code
 *
 ******************************************************************************/
tBTM_STATUS BTM_ReadTxPower(const RawAddress& remote_bda,
                            tBT_TRANSPORT transport, tBTM_CMPL_CB* p_cb) {
  tACL_CONN* p;
#define BTM_READ_RSSI_TYPE_CUR 0x00
#define BTM_READ_RSSI_TYPE_MAX 0X01

  VLOG(2) << __func__ << ": RemBdAddr: " << remote_bda;

  /* If someone already waiting on the version, do not allow another */
  if (btm_cb.devcb.p_tx_power_cmpl_cb) return (BTM_BUSY);

  p = internal_.btm_bda_to_acl(remote_bda, transport);
  if (p != (tACL_CONN*)NULL) {
    btm_cb.devcb.p_tx_power_cmpl_cb = p_cb;
    alarm_set_on_mloop(btm_cb.devcb.read_tx_power_timer,
                       BTM_DEV_REPLY_TIMEOUT_MS, btm_read_tx_power_timeout,
                       NULL);

    if (p->transport == BT_TRANSPORT_LE) {
      btm_cb.devcb.read_tx_pwr_addr = remote_bda;
      btsnd_hcic_ble_read_adv_chnl_tx_power();
    } else {
      btsnd_hcic_read_tx_power(p->hci_handle, BTM_READ_RSSI_TYPE_CUR);
    }

    return (BTM_CMD_STARTED);
  }

  /* If here, no BD Addr found */
  return (BTM_UNKNOWN_ADDR);
}

/*******************************************************************************
 *
 * Function         btm_read_tx_power_timeout
 *
 * Description      Callback when reading the tx power times out.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_read_tx_power_timeout(UNUSED_ATTR void* data) {
  tBTM_CMPL_CB* p_cb = btm_cb.devcb.p_tx_power_cmpl_cb;
  btm_cb.devcb.p_tx_power_cmpl_cb = NULL;
  if (p_cb) (*p_cb)((void*)NULL);
}

/*******************************************************************************
 *
 * Function         btm_read_tx_power_complete
 *
 * Description      This function is called when the command complete message
 *                  is received from the HCI for the read tx power request.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_read_tx_power_complete(uint8_t* p, bool is_ble) {
  tBTM_CMPL_CB* p_cb = btm_cb.devcb.p_tx_power_cmpl_cb;
  tBTM_TX_POWER_RESULT result;
  tACL_CONN* p_acl_cb = &btm_cb.acl_cb_.acl_db[0];

  BTM_TRACE_DEBUG("%s", __func__);
  alarm_cancel(btm_cb.devcb.read_tx_power_timer);
  btm_cb.devcb.p_tx_power_cmpl_cb = NULL;

  /* If there was a registered callback, call it */
  if (p_cb) {
    STREAM_TO_UINT8(result.hci_status, p);

    if (result.hci_status == HCI_SUCCESS) {
      result.status = BTM_SUCCESS;

      if (!is_ble) {
        uint16_t handle;
        STREAM_TO_UINT16(handle, p);
        STREAM_TO_UINT8(result.tx_power, p);

        /* Search through the list of active channels for the correct BD Addr */
        for (uint16_t index = 0; index < MAX_L2CAP_LINKS; index++, p_acl_cb++) {
          if ((p_acl_cb->in_use) && (handle == p_acl_cb->hci_handle)) {
            result.rem_bda = p_acl_cb->remote_addr;
            break;
          }
        }
      } else {
        STREAM_TO_UINT8(result.tx_power, p);
        result.rem_bda = btm_cb.devcb.read_tx_pwr_addr;
      }
      BTM_TRACE_DEBUG("BTM TX power Complete: tx_power %d, hci status 0x%02x",
                      result.tx_power, result.hci_status);
    } else {
      result.status = BTM_ERR_PROCESSING;
    }

    (*p_cb)(&result);
  }
}

/*******************************************************************************
 *
 * Function         btm_read_rssi_timeout
 *
 * Description      Callback when reading the RSSI times out.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_read_rssi_timeout(UNUSED_ATTR void* data) {
  tBTM_RSSI_RESULT result;
  tBTM_CMPL_CB* p_cb = btm_cb.devcb.p_rssi_cmpl_cb;
  btm_cb.devcb.p_rssi_cmpl_cb = NULL;
  result.status = BTM_DEVICE_TIMEOUT;
  if (p_cb) (*p_cb)(&result);
}

/*******************************************************************************
 *
 * Function         btm_read_rssi_complete
 *
 * Description      This function is called when the command complete message
 *                  is received from the HCI for the read rssi request.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_read_rssi_complete(uint8_t* p) {
  tBTM_CMPL_CB* p_cb = btm_cb.devcb.p_rssi_cmpl_cb;
  tBTM_RSSI_RESULT result;
  tACL_CONN* p_acl_cb = &btm_cb.acl_cb_.acl_db[0];

  BTM_TRACE_DEBUG("%s", __func__);
  alarm_cancel(btm_cb.devcb.read_rssi_timer);
  btm_cb.devcb.p_rssi_cmpl_cb = NULL;

  /* If there was a registered callback, call it */
  if (p_cb) {
    STREAM_TO_UINT8(result.hci_status, p);

    if (result.hci_status == HCI_SUCCESS) {
      uint16_t handle;
      result.status = BTM_SUCCESS;

      STREAM_TO_UINT16(handle, p);

      STREAM_TO_UINT8(result.rssi, p);
      BTM_TRACE_DEBUG("BTM RSSI Complete: rssi %d, hci status 0x%02x",
                      result.rssi, result.hci_status);

      /* Search through the list of active channels for the correct BD Addr */
      for (uint16_t index = 0; index < MAX_L2CAP_LINKS; index++, p_acl_cb++) {
        if ((p_acl_cb->in_use) && (handle == p_acl_cb->hci_handle)) {
          result.rem_bda = p_acl_cb->remote_addr;
          break;
        }
      }
    } else {
      result.status = BTM_ERR_PROCESSING;
    }

    (*p_cb)(&result);
  }
}

/*******************************************************************************
 *
 * Function         btm_read_failed_contact_counter_timeout
 *
 * Description      Callback when reading the failed contact counter times out.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_read_failed_contact_counter_timeout(UNUSED_ATTR void* data) {
  tBTM_FAILED_CONTACT_COUNTER_RESULT result;
  tBTM_CMPL_CB* p_cb = btm_cb.devcb.p_failed_contact_counter_cmpl_cb;
  btm_cb.devcb.p_failed_contact_counter_cmpl_cb = NULL;
  result.status = BTM_DEVICE_TIMEOUT;
  if (p_cb) (*p_cb)(&result);
}

/*******************************************************************************
 *
 * Function         btm_read_failed_contact_counter_complete
 *
 * Description      This function is called when the command complete message
 *                  is received from the HCI for the read failed contact
 *                  counter request.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_read_failed_contact_counter_complete(uint8_t* p) {
  tBTM_CMPL_CB* p_cb = btm_cb.devcb.p_failed_contact_counter_cmpl_cb;
  tBTM_FAILED_CONTACT_COUNTER_RESULT result;
  tACL_CONN* p_acl_cb = &btm_cb.acl_cb_.acl_db[0];

  BTM_TRACE_DEBUG("%s", __func__);
  alarm_cancel(btm_cb.devcb.read_failed_contact_counter_timer);
  btm_cb.devcb.p_failed_contact_counter_cmpl_cb = NULL;

  /* If there was a registered callback, call it */
  if (p_cb) {
    uint16_t handle;
    STREAM_TO_UINT8(result.hci_status, p);

    if (result.hci_status == HCI_SUCCESS) {
      result.status = BTM_SUCCESS;

      STREAM_TO_UINT16(handle, p);

      STREAM_TO_UINT16(result.failed_contact_counter, p);
      BTM_TRACE_DEBUG(
          "BTM Failed Contact Counter Complete: counter %u, hci status 0x%02x",
          result.failed_contact_counter, result.hci_status);

      /* Search through the list of active channels for the correct BD Addr */
      for (uint16_t index = 0; index < MAX_L2CAP_LINKS; index++, p_acl_cb++) {
        if ((p_acl_cb->in_use) && (handle == p_acl_cb->hci_handle)) {
          result.rem_bda = p_acl_cb->remote_addr;
          break;
        }
      }
    } else {
      result.status = BTM_ERR_PROCESSING;
    }

    (*p_cb)(&result);
  }
}

/*******************************************************************************
 *
 * Function         btm_read_automatic_flush_timeout_timeout
 *
 * Description      Callback when reading the automatic flush timeout times out.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_read_automatic_flush_timeout_timeout(UNUSED_ATTR void* data) {
  tBTM_AUTOMATIC_FLUSH_TIMEOUT_RESULT result;
  tBTM_CMPL_CB* p_cb = btm_cb.devcb.p_automatic_flush_timeout_cmpl_cb;
  btm_cb.devcb.p_automatic_flush_timeout_cmpl_cb = nullptr;
  result.status = BTM_DEVICE_TIMEOUT;
  if (p_cb) (*p_cb)(&result);
}

/*******************************************************************************
 *
 * Function         btm_read_automatic_flush_timeout_complete
 *
 * Description      This function is called when the command complete message
 *                  is received from the HCI for the read automatic flush
 *                  timeout request.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_read_automatic_flush_timeout_complete(uint8_t* p) {
  tBTM_CMPL_CB* p_cb = btm_cb.devcb.p_automatic_flush_timeout_cmpl_cb;
  tBTM_AUTOMATIC_FLUSH_TIMEOUT_RESULT result;
  tACL_CONN* p_acl_cb = &btm_cb.acl_cb_.acl_db[0];

  BTM_TRACE_DEBUG("%s", __func__);
  alarm_cancel(btm_cb.devcb.read_automatic_flush_timeout_timer);
  btm_cb.devcb.p_automatic_flush_timeout_cmpl_cb = nullptr;

  /* If there was a registered callback, call it */
  if (p_cb) {
    uint16_t handle;
    STREAM_TO_UINT8(result.hci_status, p);

    if (result.hci_status == HCI_SUCCESS) {
      result.status = BTM_SUCCESS;

      STREAM_TO_UINT16(handle, p);

      STREAM_TO_UINT16(result.automatic_flush_timeout, p);
      BTM_TRACE_DEBUG(
          "BTM Automatic Flush Timeout Complete: timeout %u, hci status 0x%02x",
          result.automatic_flush_timeout, result.hci_status);

      /* Search through the list of active channels for the correct BD Addr */
      for (uint16_t index = 0; index < MAX_L2CAP_LINKS; index++, p_acl_cb++) {
        if ((p_acl_cb->in_use) && (handle == p_acl_cb->hci_handle)) {
          result.rem_bda = p_acl_cb->remote_addr;
          break;
        }
      }
    } else {
      result.status = BTM_ERR_PROCESSING;
    }

    (*p_cb)(&result);
  }
}

/*******************************************************************************
 *
 * Function         btm_read_link_quality_timeout
 *
 * Description      Callback when reading the link quality times out.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_read_link_quality_timeout(UNUSED_ATTR void* data) {
  tBTM_CMPL_CB* p_cb = btm_cb.devcb.p_link_qual_cmpl_cb;
  btm_cb.devcb.p_link_qual_cmpl_cb = NULL;
  if (p_cb) (*p_cb)((void*)NULL);
}

/*******************************************************************************
 *
 * Function         btm_read_link_quality_complete
 *
 * Description      This function is called when the command complete message
 *                  is received from the HCI for the read link quality.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_read_link_quality_complete(uint8_t* p) {
  tBTM_CMPL_CB* p_cb = btm_cb.devcb.p_link_qual_cmpl_cb;
  tBTM_LINK_QUALITY_RESULT result;
  tACL_CONN* p_acl_cb = &btm_cb.acl_cb_.acl_db[0];

  BTM_TRACE_DEBUG("%s", __func__);
  alarm_cancel(btm_cb.devcb.read_link_quality_timer);
  btm_cb.devcb.p_link_qual_cmpl_cb = NULL;

  /* If there was a registered callback, call it */
  if (p_cb) {
    STREAM_TO_UINT8(result.hci_status, p);

    if (result.hci_status == HCI_SUCCESS) {
      uint16_t handle;
      result.status = BTM_SUCCESS;

      STREAM_TO_UINT16(handle, p);

      STREAM_TO_UINT8(result.link_quality, p);
      BTM_TRACE_DEBUG(
          "BTM Link Quality Complete: Link Quality %d, hci status 0x%02x",
          result.link_quality, result.hci_status);

      /* Search through the list of active channels for the correct BD Addr */
      for (uint16_t index = 0; index < MAX_L2CAP_LINKS; index++, p_acl_cb++) {
        if ((p_acl_cb->in_use) && (handle == p_acl_cb->hci_handle)) {
          result.rem_bda = p_acl_cb->remote_addr;
          break;
        }
      }
    } else {
      result.status = BTM_ERR_PROCESSING;
    }

    (*p_cb)(&result);
  }
}

/*******************************************************************************
 *
 * Function         btm_remove_acl
 *
 * Description      This function is called to disconnect an ACL connection
 *
 * Returns          BTM_SUCCESS if successfully initiated, otherwise
 *                  BTM_NO_RESOURCES.
 *
 ******************************************************************************/
tBTM_STATUS btm_remove_acl(const RawAddress& bd_addr, tBT_TRANSPORT transport) {
  uint16_t hci_handle = BTM_GetHCIConnHandle(bd_addr, transport);
  tBTM_STATUS status = BTM_SUCCESS;

  BTM_TRACE_DEBUG("btm_remove_acl");
  tBTM_SEC_DEV_REC* p_dev_rec = btm_find_dev(bd_addr);

  /* Role Switch is pending, postpone until completed */
  if (p_dev_rec && (p_dev_rec->rs_disc_pending == BTM_SEC_RS_PENDING)) {
    p_dev_rec->rs_disc_pending = BTM_SEC_DISC_PENDING;
  } else /* otherwise can disconnect right away */
  {
    if (hci_handle != HCI_INVALID_HANDLE && p_dev_rec &&
        p_dev_rec->sec_state != BTM_SEC_STATE_DISCONNECTING) {
      btsnd_hcic_disconnect(hci_handle, HCI_ERR_PEER_USER);
    } else {
      status = BTM_UNKNOWN_ADDR;
    }
  }

  return status;
}

/*******************************************************************************
 *
 * Function         BTM_SetTraceLevel
 *
 * Description      This function sets the trace level for BTM.  If called with
 *                  a value of 0xFF, it simply returns the current trace level.
 *
 * Returns          The new or current trace level
 *
 ******************************************************************************/
uint8_t BTM_SetTraceLevel(uint8_t new_level) {
  BTM_TRACE_DEBUG("BTM_SetTraceLevel");
  if (new_level != 0xFF) btm_cb.trace_level = new_level;

  return (btm_cb.trace_level);
}

/*******************************************************************************
 *
 * Function         btm_cont_rswitch
 *
 * Description      This function is called to continue processing an active
 *                  role switch. It first disables encryption if enabled and
 *                  EPR is not supported
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_cont_rswitch(tACL_CONN* p, tBTM_SEC_DEV_REC* p_dev_rec) {
  BTM_TRACE_DEBUG("btm_cont_rswitch");
  /* Check to see if encryption needs to be turned off if pending
     change of link key or role switch */
  if (p->is_switch_role_mode_change()) {
    /* Must turn off Encryption first if necessary */
    /* Some devices do not support switch or change of link key while encryption
     * is on */
    if (p_dev_rec != NULL &&
        ((p_dev_rec->sec_flags & BTM_SEC_ENCRYPTED) != 0) &&
        !BTM_EPR_AVAILABLE(p)) {
      p->set_encryption_off();
      if (p->is_switch_role_mode_change()) {
        p->set_switch_role_encryption_off();
      }
    } else /* Encryption not used or EPR supported, continue with switch
              and/or change of link key */
    {
      if (p->is_switch_role_mode_change()) {
        p->set_switch_role_in_progress();
        if (p_dev_rec) p_dev_rec->rs_disc_pending = BTM_SEC_RS_PENDING;
        btsnd_hcic_switch_role(p->remote_addr, (uint8_t)!p->link_role);
      }
    }
  }
}

void btm_cont_rswitch_from_handle(uint16_t hci_handle) {
  BTM_TRACE_DEBUG("%s", __func__);
  tACL_CONN* p_acl = internal_.acl_get_connection_from_handle(hci_handle);
  if (p_acl == nullptr) {
    BTM_TRACE_ERROR("%s role switch received but with no active ACL", __func__);
    return;
  }
  btm_cont_rswitch(p_acl, btm_find_dev(p_acl->remote_addr));
}

/*******************************************************************************
 *
 * Function         btm_acl_resubmit_page
 *
 * Description      send pending page request
 *
 ******************************************************************************/
void btm_acl_resubmit_page(void) {
  tBTM_SEC_DEV_REC* p_dev_rec;
  BT_HDR* p_buf;
  uint8_t* pp;
  BTM_TRACE_DEBUG("btm_acl_resubmit_page");
  /* If there were other page request schedule can start the next one */
  p_buf = (BT_HDR*)fixed_queue_try_dequeue(btm_cb.page_queue);
  if (p_buf != NULL) {
    /* skip 3 (2 bytes opcode and 1 byte len) to get to the bd_addr
     * for both create_conn and rmt_name */
    pp = (uint8_t*)(p_buf + 1) + p_buf->offset + 3;

    RawAddress bda;
    STREAM_TO_BDADDR(bda, pp);

    p_dev_rec = btm_find_or_alloc_dev(bda);

    btm_cb.connecting_bda = p_dev_rec->bd_addr;
    memcpy(btm_cb.connecting_dc, p_dev_rec->dev_class, DEV_CLASS_LEN);

    btu_hcif_send_cmd(LOCAL_BR_EDR_CONTROLLER_ID, p_buf);
  } else {
    btm_cb.paging = false;
  }
}

/*******************************************************************************
 *
 * Function         btm_acl_reset_paging
 *
 * Description      set paging to false and free the page queue - called at
 *                  hci_reset
 *
 ******************************************************************************/
void btm_acl_reset_paging(void) {
  BT_HDR* p;
  BTM_TRACE_DEBUG("btm_acl_reset_paging");
  /* If we sent reset we are definitely not paging any more */
  while ((p = (BT_HDR*)fixed_queue_try_dequeue(btm_cb.page_queue)) != NULL)
    osi_free(p);

  btm_cb.paging = false;
}

/*******************************************************************************
 *
 * Function         btm_acl_paging
 *
 * Description      send a paging command or queue it in btm_cb
 *
 ******************************************************************************/
void btm_acl_paging(BT_HDR* p, const RawAddress& bda) {
  tBTM_SEC_DEV_REC* p_dev_rec;

  if (!BTM_IsAclConnectionUp(bda, BT_TRANSPORT_BR_EDR)) {
    VLOG(1) << "connecting_bda: " << btm_cb.connecting_bda;
    if (btm_cb.paging && bda == btm_cb.connecting_bda) {
      fixed_queue_enqueue(btm_cb.page_queue, p);
    } else {
      p_dev_rec = btm_find_or_alloc_dev(bda);
      btm_cb.connecting_bda = p_dev_rec->bd_addr;
      memcpy(btm_cb.connecting_dc, p_dev_rec->dev_class, DEV_CLASS_LEN);

      btu_hcif_send_cmd(LOCAL_BR_EDR_CONTROLLER_ID, p);
    }

    btm_cb.paging = true;
  } else /* ACL is already up */
  {
    btu_hcif_send_cmd(LOCAL_BR_EDR_CONTROLLER_ID, p);
  }
}

/*******************************************************************************
 *
 * Function         btm_acl_notif_conn_collision
 *
 * Description      Send connection collision event to upper layer if registered
 *
 *
 ******************************************************************************/
void btm_acl_notif_conn_collision(const RawAddress& bda) {
  do_in_main_thread(FROM_HERE, base::Bind(bta_sys_notify_collision, bda));
}

/*******************************************************************************
 *
 * Function         btm_acl_chk_peer_pkt_type_support
 *
 * Description      Check if peer supports requested packets
 *
 ******************************************************************************/
void btm_acl_chk_peer_pkt_type_support(tACL_CONN* p, uint16_t* p_pkt_type) {
  /* 3 and 5 slot packets? */
  if (!HCI_3_SLOT_PACKETS_SUPPORTED(p->peer_lmp_feature_pages[0]))
    *p_pkt_type &= ~(HCI_PKT_TYPES_MASK_DH3 + HCI_PKT_TYPES_MASK_DM3);

  if (!HCI_5_SLOT_PACKETS_SUPPORTED(p->peer_lmp_feature_pages[0]))
    *p_pkt_type &= ~(HCI_PKT_TYPES_MASK_DH5 + HCI_PKT_TYPES_MASK_DM5);

  /* 2 and 3 MPS support? */
  if (!HCI_EDR_ACL_2MPS_SUPPORTED(p->peer_lmp_feature_pages[0]))
    /* Not supported. Add 'not_supported' mask for all 2MPS packet types */
    *p_pkt_type |= (HCI_PKT_TYPES_MASK_NO_2_DH1 + HCI_PKT_TYPES_MASK_NO_2_DH3 +
                    HCI_PKT_TYPES_MASK_NO_2_DH5);

  if (!HCI_EDR_ACL_3MPS_SUPPORTED(p->peer_lmp_feature_pages[0]))
    /* Not supported. Add 'not_supported' mask for all 3MPS packet types */
    *p_pkt_type |= (HCI_PKT_TYPES_MASK_NO_3_DH1 + HCI_PKT_TYPES_MASK_NO_3_DH3 +
                    HCI_PKT_TYPES_MASK_NO_3_DH5);

  /* EDR 3 and 5 slot support? */
  if (HCI_EDR_ACL_2MPS_SUPPORTED(p->peer_lmp_feature_pages[0]) ||
      HCI_EDR_ACL_3MPS_SUPPORTED(p->peer_lmp_feature_pages[0])) {
    if (!HCI_3_SLOT_EDR_ACL_SUPPORTED(p->peer_lmp_feature_pages[0]))
      /* Not supported. Add 'not_supported' mask for all 3-slot EDR packet types
       */
      *p_pkt_type |=
          (HCI_PKT_TYPES_MASK_NO_2_DH3 + HCI_PKT_TYPES_MASK_NO_3_DH3);

    if (!HCI_5_SLOT_EDR_ACL_SUPPORTED(p->peer_lmp_feature_pages[0]))
      /* Not supported. Add 'not_supported' mask for all 5-slot EDR packet types
       */
      *p_pkt_type |=
          (HCI_PKT_TYPES_MASK_NO_2_DH5 + HCI_PKT_TYPES_MASK_NO_3_DH5);
  }
}

bool lmp_version_below(const RawAddress& bda, uint8_t version) {
  tACL_CONN* acl = internal_.btm_bda_to_acl(bda, BT_TRANSPORT_LE);
  if (acl == NULL || acl->lmp_version == 0) {
    BTM_TRACE_WARNING("%s cannot retrieve LMP version...", __func__);
    return false;
  }
  BTM_TRACE_WARNING("%s LMP version %d < %d", __func__, acl->lmp_version,
                    version);
  return acl->lmp_version < version;
}

bool acl_is_role_master(const RawAddress& bda, tBT_TRANSPORT transport) {
  tACL_CONN* p = internal_.btm_bda_to_acl(bda, BT_TRANSPORT_BR_EDR);
  if (p == nullptr) {
    return false;
  }
  return (p->link_role == HCI_ROLE_MASTER);
}

bool acl_br_edr_is_role_master(const RawAddress& bda) {
  return acl_is_role_master(bda, BT_TRANSPORT_BR_EDR);
}

bool acl_ble_is_role_master(const RawAddress& bda) {
  return acl_is_role_master(bda, BT_TRANSPORT_LE);
}

bool BTM_BLE_IS_RESOLVE_BDA(const RawAddress& x) {
  return ((x.address)[0] & BLE_RESOLVE_ADDR_MASK) == BLE_RESOLVE_ADDR_MSB;
}

bool acl_refresh_remote_address(const tBTM_SEC_DEV_REC* p_sec_rec,
                                const RawAddress& bda, tBLE_ADDR_TYPE rra_type,
                                const RawAddress& rpa) {
  tACL_CONN* p_acl = internal_.btm_bda_to_acl(bda, BT_TRANSPORT_LE);
  if (p_acl == nullptr) {
    LOG_WARN("%s Unable to refresh remote address", __func__);
    return false;
  }

  if (rra_type == tBTM_SEC_BLE::BTM_BLE_ADDR_PSEUDO) {
    /* use identity address, resolvable_private_addr is empty */
    if (rpa.IsEmpty()) {
      p_acl->active_remote_addr_type =
          p_sec_rec->ble.identity_address_with_type.type;
      p_acl->active_remote_addr = p_sec_rec->ble.identity_address_with_type.bda;
    } else {
      p_acl->active_remote_addr_type = BLE_ADDR_RANDOM;
      p_acl->active_remote_addr = rpa;
    }
  } else {
    p_acl->active_remote_addr_type = rra_type;
    p_acl->active_remote_addr = rpa;
  }

  BTM_TRACE_DEBUG("%s active_remote_addr_type: %d ", __func__,
                  p_acl->active_remote_addr_type);
  return true;
}

bool acl_peer_supports_ble_connection_parameters_request(
    const RawAddress& remote_bda) {
  tACL_CONN* p_acl = internal_.btm_bda_to_acl(remote_bda, BT_TRANSPORT_LE);
  if (p_acl == nullptr) {
    return false;
  }
  return HCI_LE_CONN_PARAM_REQ_SUPPORTED(p_acl->peer_le_features);
}

bool acl_peer_supports_sniff_subrating(const RawAddress& remote_bda) {
  tACL_CONN* p_acl = internal_.btm_bda_to_acl(remote_bda, BT_TRANSPORT_LE);
  if (p_acl == nullptr) {
    return false;
  }
  return HCI_SNIFF_SUB_RATE_SUPPORTED(p_acl->peer_le_features);
}

/*******************************************************************************
 *
 * Function         BTM_ReadConnectionAddr
 *
 * Description      This function is called to get the local device address
 *                  information.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTM_ReadConnectionAddr(const RawAddress& remote_bda,
                            RawAddress& local_conn_addr,
                            tBLE_ADDR_TYPE* p_addr_type) {
  if (bluetooth::shim::is_gd_shim_enabled()) {
    return bluetooth::shim::BTM_ReadConnectionAddr(remote_bda, local_conn_addr,
                                                   p_addr_type);
  }
  tACL_CONN* p_acl = internal_.btm_bda_to_acl(remote_bda, BT_TRANSPORT_LE);

  if (p_acl == NULL) {
    BTM_TRACE_ERROR("No connection exist!");
    return;
  }
  local_conn_addr = p_acl->conn_addr;
  *p_addr_type = p_acl->conn_addr_type;

  BTM_TRACE_DEBUG("BTM_ReadConnectionAddr address type: %d addr: 0x%02x",
                  p_acl->conn_addr_type, p_acl->conn_addr.address[0]);
}

/*******************************************************************************
 *
 * Function         BTM_IsBleConnection
 *
 * Description      This function is called to check if the connection handle
 *                  for an LE link
 *
 * Returns          true if connection is LE link, otherwise false.
 *
 ******************************************************************************/
bool BTM_IsBleConnection(uint16_t hci_handle) {
  if (bluetooth::shim::is_gd_shim_enabled()) {
    ASSERT_LOG(false, "This should not be invoked from code path");
  }

  uint8_t index = btm_handle_to_acl_index(hci_handle);
  if (index >= MAX_L2CAP_LINKS) return false;

  tACL_CONN* p = &btm_cb.acl_cb_.acl_db[index];
  return (p->transport == BT_TRANSPORT_LE);
}

const RawAddress acl_address_from_handle(uint16_t hci_handle) {
  uint8_t index = btm_handle_to_acl_index(hci_handle);
  if (index >= MAX_L2CAP_LINKS) {
    return RawAddress::kEmpty;
  }
  return btm_cb.acl_cb_.acl_db[index].remote_addr;
}

tBTM_PM_MCB* acl_power_mode_from_handle(uint16_t hci_handle) {
  uint8_t index = btm_handle_to_acl_index(hci_handle);
  if (index >= MAX_L2CAP_LINKS) {
    return nullptr;
  }
  return &btm_cb.acl_cb_.pm_mode_db[index];
}

/*******************************************************************************
 *
 * Function         btm_pm_find_acl_ind
 *
 * Description      This function initializes the control block of an ACL link.
 *                  It is called when an ACL connection is created.
 *
 * Returns          void
 *
 ******************************************************************************/
int btm_pm_find_acl_ind(const RawAddress& remote_bda) {
  tACL_CONN* p = &btm_cb.acl_cb_.acl_db[0];
  uint8_t xx;

  for (xx = 0; xx < MAX_L2CAP_LINKS; xx++, p++) {
    if (p->in_use && p->remote_addr == remote_bda &&
        p->transport == BT_TRANSPORT_BR_EDR) {
      LOG_VERBOSE("btm_pm_find_acl_ind ind:%d", xx);
      break;
    }
  }
  return xx;
}

/*******************************************************************************
 *
 * Function         btm_ble_refresh_local_resolvable_private_addr
 *
 * Description      This function refresh the currently used resolvable private
 *                  address for the active link to the remote device
 *
 ******************************************************************************/
void btm_ble_refresh_local_resolvable_private_addr(
    const RawAddress& pseudo_addr, const RawAddress& local_rpa) {
  tACL_CONN* p = internal_.btm_bda_to_acl(pseudo_addr, BT_TRANSPORT_LE);

  if (p != NULL) {
    if (btm_cb.ble_ctr_cb.privacy_mode != BTM_PRIVACY_NONE) {
      p->conn_addr_type = BLE_ADDR_RANDOM;
      if (!local_rpa.IsEmpty())
        p->conn_addr = local_rpa;
      else
        p->conn_addr = btm_cb.ble_ctr_cb.addr_mgnt_cb.private_addr;
    } else {
      p->conn_addr_type = BLE_ADDR_PUBLIC;
      p->conn_addr = *controller_get_interface()->get_address();
    }
  }
}

/*******************************************************************************
 *
 * Function         btm_sec_set_peer_sec_caps
 *
 * Description      This function is called to set sm4 and rmt_sec_caps fields
 *                  based on the available peer device features.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_sec_set_peer_sec_caps(tACL_CONN* p_acl_cb,
                               tBTM_SEC_DEV_REC* p_dev_rec) {
  if ((btm_cb.security_mode == BTM_SEC_MODE_SP ||
       btm_cb.security_mode == BTM_SEC_MODE_SC) &&
      HCI_SSP_HOST_SUPPORTED(p_acl_cb->peer_lmp_feature_pages[1])) {
    p_dev_rec->sm4 = BTM_SM4_TRUE;
    p_dev_rec->remote_supports_secure_connections =
        (HCI_SC_HOST_SUPPORTED(p_acl_cb->peer_lmp_feature_pages[1]));
  } else {
    p_dev_rec->sm4 = BTM_SM4_KNOWN;
    p_dev_rec->remote_supports_secure_connections = false;
  }

  BTM_TRACE_API("%s: sm4: 0x%02x, rmt_support_for_secure_connections %d",
                __func__, p_dev_rec->sm4,
                p_dev_rec->remote_supports_secure_connections);

  if (p_dev_rec->remote_features_needed) {
    BTM_TRACE_EVENT(
        "%s: Now device in SC Only mode, waiting for peer remote features!",
        __func__);
    btm_io_capabilities_req(p_dev_rec->bd_addr);
    p_dev_rec->remote_features_needed = false;
  }
}

bool sco_peer_supports_esco_2m_phy(uint16_t hci_handle) {
  tACL_CONN* p_acl = internal_.acl_get_connection_from_handle(hci_handle);
  if (p_acl == nullptr) {
    return false;
  }
  return HCI_EDR_ESCO_2MPS_SUPPORTED(p_acl->peer_lmp_feature_pages[0]);
}

bool sco_peer_supports_esco_3m_phy(uint16_t hci_handle) {
  tACL_CONN* p_acl = internal_.acl_get_connection_from_handle(hci_handle);
  if (p_acl == nullptr) {
    return false;
  }
  return HCI_EDR_ESCO_3MPS_SUPPORTED(p_acl->peer_lmp_feature_pages[0]);
}

bool acl_is_switch_role_idle(const RawAddress& bd_addr,
                             tBT_TRANSPORT transport) {
  tACL_CONN* p_acl = internal_.btm_bda_to_acl(bd_addr, transport);
  if (p_acl == nullptr) {
    return false;
  }
  return p_acl->is_switch_role_idle();
}

/*******************************************************************************
 *
 * Function       BTM_ReadRemoteConnectionAddr
 *
 * Description    This function is read the remote device address currently used
 *
 * Parameters     pseudo_addr: pseudo random address available
 *                conn_addr:connection address used
 *                p_addr_type : BD Address type, Public or Random of the address
 *                              used
 *
 * Returns        bool, true if connection to remote device exists, else false
 *
 ******************************************************************************/
bool BTM_ReadRemoteConnectionAddr(const RawAddress& pseudo_addr,
                                  RawAddress& conn_addr,
                                  tBLE_ADDR_TYPE* p_addr_type) {
  if (bluetooth::shim::is_gd_shim_enabled()) {
    return bluetooth::shim::BTM_ReadRemoteConnectionAddr(pseudo_addr, conn_addr,
                                                         p_addr_type);
  }
  bool st = true;
  tACL_CONN* p_acl = internal_.btm_bda_to_acl(pseudo_addr, BT_TRANSPORT_LE);

  if (p_acl == NULL) {
    BTM_TRACE_ERROR(
        "BTM_ReadRemoteConnectionAddr can not find connection"
        " with matching address");
    return false;
  }

  conn_addr = p_acl->active_remote_addr;
  *p_addr_type = p_acl->active_remote_addr_type;
  return st;
}

uint8_t acl_link_role(const RawAddress& bd_addr, tBT_TRANSPORT transport) {
  tACL_CONN* p_acl = internal_.btm_bda_to_acl(bd_addr, transport);
  if (p_acl == nullptr) {
    return HCI_ROLE_UNKNOWN;
  }
  return p_acl->link_role;
}

uint8_t acl_link_role_from_handle(uint16_t handle) {
  tACL_CONN* p_acl = internal_.acl_get_connection_from_handle(handle);
  if (p_acl == nullptr) {
    return HCI_ROLE_UNKNOWN;
  }
  return p_acl->link_role;
}

bool acl_is_transport_le_from_handle(uint16_t handle) {
  tACL_CONN* p_acl = internal_.acl_get_connection_from_handle(handle);
  if (p_acl == nullptr) {
    return false;
  }
  return p_acl->transport == BT_TRANSPORT_LE;
}

tBT_TRANSPORT acl_get_transport_from_handle(uint16_t handle) {
  tACL_CONN* p_acl = internal_.acl_get_connection_from_handle(handle);
  if (p_acl == nullptr) {
    return BT_TRANSPORT_INVALID;
  }
  return p_acl->transport;
}

uint16_t acl_get_hci_handle_for_hcif(const RawAddress& bd_addr,
                                     tBT_TRANSPORT transport) {
  tACL_CONN* p_acl = internal_.btm_bda_to_acl(bd_addr, transport);
  if (p_acl == nullptr) {
    return HCI_INVALID_HANDLE;
  }
  return p_acl->hci_handle;
}

bool acl_peer_supports_ble_packet_extension(uint16_t hci_handle) {
  tACL_CONN* p_acl = internal_.acl_get_connection_from_handle(hci_handle);
  if (p_acl == nullptr) {
    return false;
  }
  return HCI_LE_DATA_LEN_EXT_SUPPORTED(p_acl->peer_lmp_feature_pages[0]);
}

bool acl_peer_supports_ble_2m_phy(uint16_t hci_handle) {
  tACL_CONN* p_acl = internal_.acl_get_connection_from_handle(hci_handle);
  if (p_acl == nullptr) {
    return false;
  }
  return HCI_LE_2M_PHY_SUPPORTED(p_acl->peer_le_features);
}

bool acl_peer_supports_ble_coded_phy(uint16_t hci_handle) {
  tACL_CONN* p_acl = internal_.acl_get_connection_from_handle(hci_handle);
  if (p_acl == nullptr) {
    return false;
  }
  return HCI_LE_CODED_PHY_SUPPORTED(p_acl->peer_le_features);
}

uint16_t acl_get_link_supervision_timeout() {
  return btm_cb.acl_cb_.btm_def_link_super_tout;
}

uint8_t acl_get_disconnect_reason() { return btm_cb.acl_cb_.acl_disc_reason; }

void acl_set_disconnect_reason(uint8_t acl_disc_reason) {
  btm_cb.acl_cb_.acl_disc_reason = acl_disc_reason;
}

bool acl_is_role_switch_allowed() {
  return btm_cb.acl_cb_.btm_def_link_policy & HCI_ENABLE_MASTER_SLAVE_SWITCH;
}

uint16_t acl_get_supported_packet_types() {
  return btm_cb.acl_cb_.btm_acl_pkt_types_supported;
}

bool acl_set_peer_le_features_from_handle(uint16_t hci_handle,
                                          const uint8_t* p) {
  tACL_CONN* p_acl = internal_.acl_get_connection_from_handle(hci_handle);
  if (p_acl == nullptr) {
    return false;
  }
  STREAM_TO_ARRAY(p_acl->peer_le_features, p, BD_FEATURES_LEN);
  return true;
}

void btm_acl_connected(const RawAddress& bda, uint16_t handle, uint8_t status,
                       uint8_t enc_mode) {
  btm_sec_connected(bda, handle, status, enc_mode);
  btm_acl_set_paging(false);
  l2c_link_hci_conn_comp(status, handle, bda);
}

constexpr uint16_t kDefaultPacketTypes =
    HCI_PKT_TYPES_MASK_DM1 | HCI_PKT_TYPES_MASK_DH1 | HCI_PKT_TYPES_MASK_DM3 |
    HCI_PKT_TYPES_MASK_DH3 | HCI_PKT_TYPES_MASK_DM5 | HCI_PKT_TYPES_MASK_DH5;

void acl_create_classic_connection(const RawAddress& bd_addr,
                                   bool there_are_high_priority_channels,
                                   bool is_bonding) {
  if (bluetooth::shim::is_gd_acl_enabled()) {
    bluetooth::shim::ACL_CreateClassicConnection(bd_addr);
  }

  const bool controller_supports_role_switch =
      controller_get_interface()->supports_role_switch();
  const bool acl_allows_role_switch = acl_is_role_switch_allowed();

  /* FW team says that we can participant in 4 piconets
   * typically 3 piconet + 1 for scanning.
   * We can enhance the code to count the number of piconets later. */
  uint8_t allow_role_switch = HCI_CR_CONN_NOT_ALLOW_SWITCH;
  if (((acl_allows_role_switch && (BTM_GetNumAclLinks() < 3)) ||
       (is_bonding && !there_are_high_priority_channels &&
        controller_supports_role_switch)))
    allow_role_switch = HCI_CR_CONN_ALLOW_SWITCH;

  /* Check with the BT manager if details about remote device are known */
  uint8_t page_scan_rep_mode{HCI_PAGE_SCAN_REP_MODE_R1};
  uint8_t page_scan_mode{HCI_MANDATARY_PAGE_SCAN_MODE};
  uint16_t clock_offset = BTM_GetClockOffset(bd_addr);

  tBTM_INQ_INFO* p_inq_info = BTM_InqDbRead(bd_addr);
  if (p_inq_info != nullptr &&
      (p_inq_info->results.inq_result_type & BTM_INQ_RESULT_BR)) {
    page_scan_rep_mode = p_inq_info->results.page_scan_rep_mode;
    page_scan_mode = p_inq_info->results.page_scan_mode;
    clock_offset = p_inq_info->results.clock_offset;
  }

  btsnd_hcic_create_conn(bd_addr, kDefaultPacketTypes, page_scan_rep_mode,
                         page_scan_mode, clock_offset, allow_role_switch);
  btm_acl_set_paging(true);
}

void btm_acl_connection_request(const RawAddress& bda, uint8_t* dc) {
  btm_sec_conn_req(bda, dc);
  l2c_link_hci_conn_req(bda);
}

void acl_accept_connection_request(const RawAddress& bd_addr, uint8_t role) {
  btsnd_hcic_accept_conn(bd_addr, role);
}

void acl_reject_connection_request(const RawAddress& bd_addr, uint8_t reason) {
  btsnd_hcic_reject_conn(bd_addr, reason);
}

void acl_disconnect(const RawAddress& bd_addr, tBT_TRANSPORT transport,
                    uint8_t reason) {
  tACL_CONN* p_acl = internal_.btm_bda_to_acl(bd_addr, transport);
  if (p_acl == nullptr) {
    LOG_WARN("%s Acl disconnect request for unknown device", __func__);
    return;
  }
  p_acl->disconnect_reason = reason;
  btsnd_hcic_disconnect(p_acl->hci_handle, reason);
}

constexpr uint16_t kDataPacketEventBrEdr = (BT_EVT_TO_LM_HCI_ACL);
constexpr uint16_t kDataPacketEventBle =
    (BT_EVT_TO_LM_HCI_ACL | LOCAL_BLE_CONTROLLER_ID);

void acl_send_data_packet_br_edr([[maybe_unused]] const RawAddress& bd_addr,
                                 BT_HDR* p_buf) {
  bte_main_hci_send(p_buf, kDataPacketEventBrEdr);
}

void acl_send_data_packet_ble([[maybe_unused]] const RawAddress& bd_addr,
                              BT_HDR* p_buf) {
  bte_main_hci_send(p_buf, kDataPacketEventBle);
}

void acl_write_automatic_flush_timeout(const RawAddress& bd_addr,
                                       uint16_t flush_timeout_in_ticks) {
  tACL_CONN* p_acl = internal_.btm_bda_to_acl(bd_addr, BT_TRANSPORT_BR_EDR);
  if (p_acl == nullptr) {
    LOG_ERROR("%s Unknown peer ACL", __func__);
    return;
  }
  if (p_acl->flush_timeout_in_ticks == flush_timeout_in_ticks) {
    LOG_INFO(
        "%s Ignoring since cached value is same as requested flush_timeout:%hd",
        __func__, flush_timeout_in_ticks);
    return;
  }
  flush_timeout_in_ticks &= HCI_MAX_AUTOMATIC_FLUSH_TIMEOUT;
  p_acl->flush_timeout_in_ticks = flush_timeout_in_ticks;
  btsnd_hcic_write_auto_flush_tout(p_acl->hci_handle, flush_timeout_in_ticks);
}

bool acl_create_le_connection(const RawAddress& bd_addr) {
  return connection_manager::direct_connect_add(CONN_MGR_ID_L2CAP, bd_addr);
}

void acl_cancel_le_connection(const RawAddress& bd_addr) {
  connection_manager::direct_connect_remove(CONN_MGR_ID_L2CAP, bd_addr);
}
