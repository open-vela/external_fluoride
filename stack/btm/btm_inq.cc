/******************************************************************************
 *
 *  Copyright 1999-2014 Broadcom Corporation
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
 *  This file contains functions that handle inquiries. These include
 *  setting discoverable mode, controlling the mode of the Baseband, and
 *  maintaining a small database of inquiry responses, with API for people
 *  to browse it.
 *
 ******************************************************************************/

#define LOG_TAG "bluetooth"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common/time_util.h"
#include "device/include/controller.h"
#include "osi/include/log.h"
#include "osi/include/osi.h"

#include "advertise_data_parser.h"
#include "bt_common.h"
#include "bt_types.h"
#include "btm_api.h"
#include "btm_int.h"
#include "btu.h"
#include "hcidefs.h"
#include "main/shim/btm_api.h"
#include "main/shim/shim.h"
#include "stack/btm/btm_ble_int.h"
#include "stack/btm/btm_int_types.h"
#include "stack/include/acl_api.h"
#include "stack/include/btm_ble_api.h"
#include "stack/include/hcimsgs.h"
#include "stack/include/inq_hci_link_interface.h"

extern tBTM_CB btm_cb;

extern void btm_inq_remote_name_timer_timeout(void* data);
extern tBTM_STATUS btm_ble_read_remote_name(const RawAddress& remote_bda,
                                            tBTM_CMPL_CB* p_cb);
extern bool btm_ble_cancel_remote_name(const RawAddress& remote_bda);
extern tBTM_STATUS btm_ble_set_discoverability(uint16_t combined_mode);
extern tBTM_STATUS btm_ble_set_connectability(uint16_t combined_mode);

extern tBTM_STATUS btm_ble_start_inquiry(uint8_t duration);
extern void btm_ble_stop_inquiry(void);

using bluetooth::Uuid;

/* 3 second timeout waiting for responses */
#define BTM_INQ_REPLY_TIMEOUT_MS (3 * 1000)

/* TRUE to enable DEBUG traces for btm_inq */
#ifndef BTM_INQ_DEBUG
#define BTM_INQ_DEBUG FALSE
#endif

#define BTIF_DM_DEFAULT_INQ_MAX_DURATION 10

/******************************************************************************/
/*               L O C A L    D A T A    D E F I N I T I O N S                */
/******************************************************************************/
static const LAP general_inq_lap = {0x9e, 0x8b, 0x33};
static const LAP limited_inq_lap = {0x9e, 0x8b, 0x00};

const uint16_t BTM_EIR_UUID_LKUP_TBL[BTM_EIR_MAX_SERVICES] = {
    UUID_SERVCLASS_SERVICE_DISCOVERY_SERVER,
    /*    UUID_SERVCLASS_BROWSE_GROUP_DESCRIPTOR,   */
    /*    UUID_SERVCLASS_PUBLIC_BROWSE_GROUP,       */
    UUID_SERVCLASS_SERIAL_PORT, UUID_SERVCLASS_LAN_ACCESS_USING_PPP,
    UUID_SERVCLASS_DIALUP_NETWORKING, UUID_SERVCLASS_IRMC_SYNC,
    UUID_SERVCLASS_OBEX_OBJECT_PUSH, UUID_SERVCLASS_OBEX_FILE_TRANSFER,
    UUID_SERVCLASS_IRMC_SYNC_COMMAND, UUID_SERVCLASS_HEADSET,
    UUID_SERVCLASS_CORDLESS_TELEPHONY, UUID_SERVCLASS_AUDIO_SOURCE,
    UUID_SERVCLASS_AUDIO_SINK, UUID_SERVCLASS_AV_REM_CTRL_TARGET,
    /*    UUID_SERVCLASS_ADV_AUDIO_DISTRIBUTION,    */
    UUID_SERVCLASS_AV_REMOTE_CONTROL,
    /*    UUID_SERVCLASS_VIDEO_CONFERENCING,        */
    UUID_SERVCLASS_INTERCOM, UUID_SERVCLASS_FAX,
    UUID_SERVCLASS_HEADSET_AUDIO_GATEWAY,
    /*    UUID_SERVCLASS_WAP,                       */
    /*    UUID_SERVCLASS_WAP_CLIENT,                */
    UUID_SERVCLASS_PANU, UUID_SERVCLASS_NAP, UUID_SERVCLASS_GN,
    UUID_SERVCLASS_DIRECT_PRINTING,
    /*    UUID_SERVCLASS_REFERENCE_PRINTING,        */
    UUID_SERVCLASS_IMAGING, UUID_SERVCLASS_IMAGING_RESPONDER,
    UUID_SERVCLASS_IMAGING_AUTO_ARCHIVE, UUID_SERVCLASS_IMAGING_REF_OBJECTS,
    UUID_SERVCLASS_HF_HANDSFREE, UUID_SERVCLASS_AG_HANDSFREE,
    UUID_SERVCLASS_DIR_PRT_REF_OBJ_SERVICE,
    /*    UUID_SERVCLASS_REFLECTED_UI,              */
    UUID_SERVCLASS_BASIC_PRINTING, UUID_SERVCLASS_PRINTING_STATUS,
    UUID_SERVCLASS_HUMAN_INTERFACE, UUID_SERVCLASS_CABLE_REPLACEMENT,
    UUID_SERVCLASS_HCRP_PRINT, UUID_SERVCLASS_HCRP_SCAN,
    /*    UUID_SERVCLASS_COMMON_ISDN_ACCESS,        */
    /*    UUID_SERVCLASS_VIDEO_CONFERENCING_GW,     */
    /*    UUID_SERVCLASS_UDI_MT,                    */
    /*    UUID_SERVCLASS_UDI_TA,                    */
    /*    UUID_SERVCLASS_VCP,                       */
    UUID_SERVCLASS_SAP, UUID_SERVCLASS_PBAP_PCE, UUID_SERVCLASS_PBAP_PSE,
    UUID_SERVCLASS_PHONE_ACCESS, UUID_SERVCLASS_HEADSET_HS,
    UUID_SERVCLASS_PNP_INFORMATION,
    /*    UUID_SERVCLASS_GENERIC_NETWORKING,        */
    /*    UUID_SERVCLASS_GENERIC_FILETRANSFER,      */
    /*    UUID_SERVCLASS_GENERIC_AUDIO,             */
    /*    UUID_SERVCLASS_GENERIC_TELEPHONY,         */
    /*    UUID_SERVCLASS_UPNP_SERVICE,              */
    /*    UUID_SERVCLASS_UPNP_IP_SERVICE,           */
    /*    UUID_SERVCLASS_ESDP_UPNP_IP_PAN,          */
    /*    UUID_SERVCLASS_ESDP_UPNP_IP_LAP,          */
    /*    UUID_SERVCLASS_ESDP_UPNP_IP_L2CAP,        */
    UUID_SERVCLASS_VIDEO_SOURCE, UUID_SERVCLASS_VIDEO_SINK,
    /*    UUID_SERVCLASS_VIDEO_DISTRIBUTION         */
    UUID_SERVCLASS_MESSAGE_ACCESS, UUID_SERVCLASS_MESSAGE_NOTIFICATION,
    UUID_SERVCLASS_HDP_SOURCE, UUID_SERVCLASS_HDP_SINK};

/******************************************************************************/
/*            L O C A L    F U N C T I O N     P R O T O T Y P E S            */
/******************************************************************************/
static void btm_clr_inq_db(const RawAddress* p_bda);
void btm_clr_inq_result_flt(void);
static void btm_inq_rmt_name_failed_cancelled(void);
static tBTM_STATUS btm_initiate_rem_name(const RawAddress& remote_bda,
                                         uint8_t origin, uint64_t timeout_ms,
                                         tBTM_CMPL_CB* p_cb);

static uint8_t btm_convert_uuid_to_eir_service(uint16_t uuid16);
void btm_set_eir_uuid(uint8_t* p_eir, tBTM_INQ_RESULTS* p_results);
static const uint8_t* btm_eir_get_uuid_list(uint8_t* p_eir, size_t eir_len,
                                            uint8_t uuid_size,
                                            uint8_t* p_num_uuid,
                                            uint8_t* p_uuid_list_type);

void SendRemoteNameRequest(const RawAddress& raw_address) {
  if (bluetooth::shim::is_gd_shim_enabled()) {
    return bluetooth::shim::SendRemoteNameRequest(raw_address);
  } else {
    btsnd_hcic_rmt_name_req(raw_address, HCI_PAGE_SCAN_REP_MODE_R1,
                            HCI_MANDATARY_PAGE_SCAN_MODE, 0);
  }
}
/*******************************************************************************
 *
 * Function         BTM_SetDiscoverability
 *
 * Description      This function is called to set the device into or out of
 *                  discoverable mode. Discoverable mode means inquiry
 *                  scans are enabled.  If a value of '0' is entered for window
 *                  or interval, the default values are used.
 *
 * Returns          BTM_SUCCESS if successful
 *                  BTM_BUSY if a setting of the filter is already in progress
 *                  BTM_NO_RESOURCES if couldn't get a memory pool buffer
 *                  BTM_ILLEGAL_VALUE if a bad parameter was detected
 *                  BTM_WRONG_MODE if the device is not up.
 *
 ******************************************************************************/
tBTM_STATUS BTM_SetDiscoverability(uint16_t inq_mode) {
  if (bluetooth::shim::is_gd_shim_enabled()) {
    return bluetooth::shim::BTM_SetDiscoverability(inq_mode, 0, 0);
  }

  uint8_t scan_mode = 0;
  uint16_t service_class;
  uint8_t* p_cod;
  uint8_t major, minor;
  DEV_CLASS cod;
  LAP temp_lap[2];
  bool is_limited;
  bool cod_limited;
  uint16_t window = BTM_DEFAULT_DISC_WINDOW;
  uint16_t interval = BTM_DEFAULT_DISC_INTERVAL;

  BTM_TRACE_API("BTM_SetDiscoverability");
  if (controller_get_interface()->supports_ble()) {
    if (btm_ble_set_discoverability((uint16_t)(inq_mode)) == BTM_SUCCESS) {
      btm_cb.btm_inq_vars.discoverable_mode &= (~BTM_BLE_DISCOVERABLE_MASK);
      btm_cb.btm_inq_vars.discoverable_mode |=
          (inq_mode & BTM_BLE_DISCOVERABLE_MASK);
    }
  }
  inq_mode &= ~BTM_BLE_DISCOVERABLE_MASK;

  /*** Check mode parameter ***/
  if (inq_mode > BTM_MAX_DISCOVERABLE) return (BTM_ILLEGAL_VALUE);

  /* Make sure the controller is active */
  if (!controller_get_interface()->get_is_ready()) return (BTM_DEV_RESET);

  /* If the window and/or interval is '0', set to default values */
  BTM_TRACE_API("BTM_SetDiscoverability: mode %d [NonDisc-0, Lim-1, Gen-2]",
                inq_mode);

  /* Set the IAC if needed */
  if (inq_mode != BTM_NON_DISCOVERABLE) {
    if (inq_mode & BTM_LIMITED_DISCOVERABLE) {
      /* Use the GIAC and LIAC codes for limited discoverable mode */
      memcpy(temp_lap[0], limited_inq_lap, LAP_LEN);
      memcpy(temp_lap[1], general_inq_lap, LAP_LEN);

      btsnd_hcic_write_cur_iac_lap(2, (LAP * const)temp_lap);
    } else {
      btsnd_hcic_write_cur_iac_lap(1, (LAP * const) & general_inq_lap);
    }

    scan_mode |= HCI_INQUIRY_SCAN_ENABLED;
  }

  /* Send down the inquiry scan window and period if changed */
  if ((window != btm_cb.btm_inq_vars.inq_scan_window) ||
      (interval != btm_cb.btm_inq_vars.inq_scan_period)) {
    btsnd_hcic_write_inqscan_cfg(interval, window);
    btm_cb.btm_inq_vars.inq_scan_window = window;
    btm_cb.btm_inq_vars.inq_scan_period = interval;
  }

  if (btm_cb.btm_inq_vars.connectable_mode & BTM_CONNECTABLE_MASK)
    scan_mode |= HCI_PAGE_SCAN_ENABLED;

  btsnd_hcic_write_scan_enable(scan_mode);
  btm_cb.btm_inq_vars.discoverable_mode &= (~BTM_DISCOVERABLE_MASK);
  btm_cb.btm_inq_vars.discoverable_mode |= inq_mode;

  /* Change the service class bit if mode has changed */
  p_cod = BTM_ReadDeviceClass();
  BTM_COD_SERVICE_CLASS(service_class, p_cod);
  is_limited = (inq_mode & BTM_LIMITED_DISCOVERABLE) ? true : false;
  cod_limited = (service_class & BTM_COD_SERVICE_LMTD_DISCOVER) ? true : false;
  if (is_limited ^ cod_limited) {
    BTM_COD_MINOR_CLASS(minor, p_cod);
    BTM_COD_MAJOR_CLASS(major, p_cod);
    if (is_limited)
      service_class |= BTM_COD_SERVICE_LMTD_DISCOVER;
    else
      service_class &= ~BTM_COD_SERVICE_LMTD_DISCOVER;

    FIELDS_TO_COD(cod, minor, major, service_class);
    (void)BTM_SetDeviceClass(cod);
  }

  return (BTM_SUCCESS);
}

void BTM_EnableInterlacedInquiryScan() {
  if (bluetooth::shim::is_gd_shim_enabled()) {
    bluetooth::shim::BTM_EnableInterlacedInquiryScan();
  }

  BTM_TRACE_API("BTM_EnableInterlacedInquiryScan");
  if (!controller_get_interface()->supports_interlaced_inquiry_scan() ||
      btm_cb.btm_inq_vars.inq_scan_type == BTM_SCAN_TYPE_INTERLACED) {
    return;
  }

  btsnd_hcic_write_inqscan_type(BTM_SCAN_TYPE_INTERLACED);
  btm_cb.btm_inq_vars.inq_scan_type = BTM_SCAN_TYPE_INTERLACED;
}

void BTM_EnableInterlacedPageScan() {
  if (bluetooth::shim::is_gd_shim_enabled()) {
    bluetooth::shim::BTM_EnableInterlacedPageScan();
    return;
  }

  BTM_TRACE_API("BTM_EnableInterlacedPageScan");
  if (!controller_get_interface()->supports_interlaced_inquiry_scan() ||
      btm_cb.btm_inq_vars.page_scan_type == BTM_SCAN_TYPE_INTERLACED) {
    return;
  }

  btsnd_hcic_write_pagescan_type(BTM_SCAN_TYPE_INTERLACED);
  btm_cb.btm_inq_vars.page_scan_type = BTM_SCAN_TYPE_INTERLACED;
}

/*******************************************************************************
 *
 * Function         BTM_SetInquiryMode
 *
 * Description      This function is called to set standard or with RSSI
 *                  mode of the inquiry for local device.
 *
 * Output Params:   mode - standard, with RSSI, extended
 *
 * Returns          BTM_SUCCESS if successful
 *                  BTM_NO_RESOURCES if couldn't get a memory pool buffer
 *                  BTM_ILLEGAL_VALUE if a bad parameter was detected
 *                  BTM_WRONG_MODE if the device is not up.
 *
 ******************************************************************************/
tBTM_STATUS BTM_SetInquiryMode(uint8_t mode) {
  if (bluetooth::shim::is_gd_shim_enabled()) {
    return bluetooth::shim::BTM_SetInquiryMode(mode);
  }

  const controller_t* controller = controller_get_interface();
  BTM_TRACE_API("BTM_SetInquiryMode");
  if (mode == BTM_INQ_RESULT_STANDARD) {
    /* mandatory mode */
  } else if (mode == BTM_INQ_RESULT_WITH_RSSI) {
    if (!controller->supports_rssi_with_inquiry_results())
      return (BTM_MODE_UNSUPPORTED);
  } else if (mode == BTM_INQ_RESULT_EXTENDED) {
    if (!controller->supports_extended_inquiry_response())
      return (BTM_MODE_UNSUPPORTED);
  } else
    return (BTM_ILLEGAL_VALUE);

  if (!BTM_IsDeviceUp()) return (BTM_WRONG_MODE);

  btsnd_hcic_write_inquiry_mode(mode);

  return (BTM_SUCCESS);
}

/*******************************************************************************
 *
 * Function         BTM_SetConnectability
 *
 * Description      This function is called to set the device into or out of
 *                  connectable mode. Discoverable mode means page scans are
 *                  enabled.
 *
 * Returns          BTM_SUCCESS if successful
 *                  BTM_ILLEGAL_VALUE if a bad parameter is detected
 *                  BTM_NO_RESOURCES if could not allocate a message buffer
 *                  BTM_WRONG_MODE if the device is not up.
 *
 ******************************************************************************/
tBTM_STATUS BTM_SetConnectability(uint16_t page_mode) {
  if (bluetooth::shim::is_gd_shim_enabled()) {
    return bluetooth::shim::BTM_SetConnectability(page_mode, 0, 0);
  }

  uint8_t scan_mode = 0;
  uint16_t window = BTM_DEFAULT_CONN_WINDOW;
  uint16_t interval = BTM_DEFAULT_CONN_INTERVAL;
  tBTM_INQUIRY_VAR_ST* p_inq = &btm_cb.btm_inq_vars;

  BTM_TRACE_API("BTM_SetConnectability");

  if (controller_get_interface()->supports_ble()) {
    if (btm_ble_set_connectability(page_mode) != BTM_SUCCESS) {
      return BTM_NO_RESOURCES;
    }
    p_inq->connectable_mode &= (~BTM_BLE_CONNECTABLE_MASK);
    p_inq->connectable_mode |= (page_mode & BTM_BLE_CONNECTABLE_MASK);
  }
  page_mode &= ~BTM_BLE_CONNECTABLE_MASK;

  /*** Check mode parameter ***/
  if (page_mode != BTM_NON_CONNECTABLE && page_mode != BTM_CONNECTABLE)
    return (BTM_ILLEGAL_VALUE);

  /* Make sure the controller is active */
  if (!controller_get_interface()->get_is_ready()) return (BTM_DEV_RESET);

  BTM_TRACE_API("BTM_SetConnectability: mode %d [NonConn-0, Conn-1]",
                page_mode);

  /*** Only check window and duration if mode is connectable ***/
  if (page_mode == BTM_CONNECTABLE) {
    scan_mode |= HCI_PAGE_SCAN_ENABLED;
  }

  if ((window != p_inq->page_scan_window) ||
      (interval != p_inq->page_scan_period)) {
    p_inq->page_scan_window = window;
    p_inq->page_scan_period = interval;
    btsnd_hcic_write_pagescan_cfg(interval, window);
  }

  /* Keep the inquiry scan as previouosly set */
  if (p_inq->discoverable_mode & BTM_DISCOVERABLE_MASK)
    scan_mode |= HCI_INQUIRY_SCAN_ENABLED;

  btsnd_hcic_write_scan_enable(scan_mode);
  p_inq->connectable_mode &= (~BTM_CONNECTABLE_MASK);
  p_inq->connectable_mode |= page_mode;
  return (BTM_SUCCESS);
}

/*******************************************************************************
 *
 * Function         BTM_IsInquiryActive
 *
 * Description      This function returns a bit mask of the current inquiry
 *                  state
 *
 * Returns          BTM_INQUIRY_INACTIVE if inactive (0)
 *                  BTM_GENERAL_INQUIRY_ACTIVE if a general inquiry is active
 *
 ******************************************************************************/
uint16_t BTM_IsInquiryActive(void) {
  if (bluetooth::shim::is_gd_shim_enabled()) {
    return bluetooth::shim::BTM_IsInquiryActive();
  }

  BTM_TRACE_API("BTM_IsInquiryActive");

  return (btm_cb.btm_inq_vars.inq_active);
}

/*******************************************************************************
 *
 * Function         BTM_CancelInquiry
 *
 * Description      This function cancels an inquiry if active
 *
 ******************************************************************************/
void BTM_CancelInquiry(void) {
  if (bluetooth::shim::is_gd_shim_enabled()) {
    bluetooth::shim::BTM_CancelInquiry();
    return;
  }

  tBTM_INQUIRY_VAR_ST* p_inq = &btm_cb.btm_inq_vars;
  BTM_TRACE_API("BTM_CancelInquiry called");

  CHECK(BTM_IsDeviceUp());

  /* Only cancel if not in periodic mode, otherwise the caller should call
   * BTM_CancelPeriodicMode */
  if ((p_inq->inq_active & BTM_INQUIRY_ACTIVE_MASK) != 0) {
    p_inq->inq_active = BTM_INQUIRY_INACTIVE;
    p_inq->state = BTM_INQ_INACTIVE_STATE;
    p_inq->p_inq_results_cb = NULL; /* Do not notify caller anymore */
    p_inq->p_inq_cmpl_cb = NULL;    /* Do not notify caller anymore */

    if ((p_inq->inqparms.mode & BTM_BR_INQUIRY_MASK) != 0) {
      bluetooth::legacy::hci::GetInterface().InquiryCancel();
    }
    if ((p_inq->inqparms.mode & BTM_BLE_INQUIRY_MASK) != 0)
      btm_ble_stop_inquiry();

    p_inq->inq_counter++;
    btm_clr_inq_result_flt();
  }
}

/*******************************************************************************
 *
 * Function         BTM_StartInquiry
 *
 * Description      This function is called to start an inquiry.
 *
 * Parameters:      p_inqparms - pointer to the inquiry information
 *                      mode - GENERAL or LIMITED inquiry, BR/LE bit mask
 *                             seperately
 *                      duration - length in 1.28 sec intervals (If '0', the
 *                                 inquiry is CANCELLED)
 *                      filter_cond_type - BTM_CLR_INQUIRY_FILTER,
 *                                         BTM_FILTER_COND_DEVICE_CLASS, or
 *                                         BTM_FILTER_COND_BD_ADDR
 *                      filter_cond - value for the filter (based on
 *                                                          filter_cond_type)
 *
 *                  p_results_cb   - Pointer to the callback routine which gets
 *                                called upon receipt of an inquiry result. If
 *                                this field is NULL, the application is not
 *                                notified.
 *
 *                  p_cmpl_cb   - Pointer to the callback routine which gets
 *                                called upon completion.  If this field is
 *                                NULL, the application is not notified when
 *                                completed.
 * Returns          tBTM_STATUS
 *                  BTM_CMD_STARTED if successfully initiated
 *                  BTM_BUSY if already in progress
 *                  BTM_ILLEGAL_VALUE if parameter(s) are out of range
 *                  BTM_NO_RESOURCES if could not allocate resources to start
 *                                   the command
 *                  BTM_WRONG_MODE if the device is not up.
 *
 ******************************************************************************/
tBTM_STATUS BTM_StartInquiry(tBTM_INQ_RESULTS_CB* p_results_cb,
                             tBTM_CMPL_CB* p_cmpl_cb) {
  tBTM_INQUIRY_VAR_ST* p_inq = &btm_cb.btm_inq_vars;

  if (bluetooth::shim::is_gd_shim_enabled()) {
    return bluetooth::shim::BTM_StartInquiry(p_results_cb, p_cmpl_cb);
  }

  /* Only one active inquiry is allowed in this implementation.
     Also do not allow an inquiry if the inquiry filter is being updated */
  if (p_inq->inq_active) {
    LOG(ERROR) << __func__ << ": BTM_BUSY";
    return (BTM_BUSY);
  }

  /*** Make sure the device is ready ***/
  if (!BTM_IsDeviceUp()) {
    LOG(ERROR) << __func__ << ": adapter is not up";
    return BTM_WRONG_MODE;
  }

  /* Save the inquiry parameters to be used upon the completion of
   * setting/clearing the inquiry filter */
  p_inq->inqparms = {};
  p_inq->inqparms.mode = BTM_GENERAL_INQUIRY | BTM_BLE_GENERAL_INQUIRY;
  p_inq->inqparms.duration = BTIF_DM_DEFAULT_INQ_MAX_DURATION;

  /* Initialize the inquiry variables */
  p_inq->state = BTM_INQ_ACTIVE_STATE;
  p_inq->p_inq_cmpl_cb = p_cmpl_cb;
  p_inq->p_inq_results_cb = p_results_cb;
  p_inq->inq_cmpl_info.num_resp = 0; /* Clear the results counter */
  p_inq->inq_active = p_inq->inqparms.mode;

  BTM_TRACE_DEBUG("BTM_StartInquiry: p_inq->inq_active = 0x%02x",
                  p_inq->inq_active);

  if (controller_get_interface()->supports_ble()) {
    btm_ble_start_inquiry(p_inq->inqparms.duration);
  }
  p_inq->inqparms.mode &= ~BTM_BLE_INQUIRY_MASK;

  btm_acl_update_inquiry_status(BTM_INQUIRY_STARTED);

  if (p_inq->inq_active & BTM_SSP_INQUIRY_ACTIVE) {
    btm_process_inq_complete(BTM_NO_RESOURCES, BTM_GENERAL_INQUIRY);
    return BTM_CMD_STARTED;
  }

  btm_clr_inq_result_flt();

  /* Allocate memory to hold bd_addrs responding */
  p_inq->p_bd_db = (tINQ_BDADDR*)osi_calloc(BT_DEFAULT_BUFFER_SIZE);
  p_inq->max_bd_entries =
      (uint16_t)(BT_DEFAULT_BUFFER_SIZE / sizeof(tINQ_BDADDR));

  bluetooth::legacy::hci::GetInterface().StartInquiry(
      general_inq_lap, p_inq->inqparms.duration, 0);
  return BTM_CMD_STARTED;
}

/*******************************************************************************
 *
 * Function         BTM_ReadRemoteDeviceName
 *
 * Description      This function initiates a remote device HCI command to the
 *                  controller and calls the callback when the process has
 *                  completed.
 *
 * Input Params:    remote_bda      - device address of name to retrieve
 *                  p_cb            - callback function called when
 *                                    BTM_CMD_STARTED is returned.
 *                                    A pointer to tBTM_REMOTE_DEV_NAME is
 *                                    passed to the callback.
 *
 * Returns
 *                  BTM_CMD_STARTED is returned if the request was successfully
 *                                  sent to HCI.
 *                  BTM_BUSY if already in progress
 *                  BTM_UNKNOWN_ADDR if device address is bad
 *                  BTM_NO_RESOURCES if could not allocate resources to start
 *                                   the command
 *                  BTM_WRONG_MODE if the device is not up.
 *
 ******************************************************************************/
tBTM_STATUS BTM_ReadRemoteDeviceName(const RawAddress& remote_bda,
                                     tBTM_CMPL_CB* p_cb,
                                     tBT_TRANSPORT transport) {
  if (bluetooth::shim::is_gd_shim_enabled()) {
    return bluetooth::shim::BTM_ReadRemoteDeviceName(remote_bda, p_cb,
                                                     transport);
  }

  VLOG(1) << __func__ << ": bd addr " << remote_bda;
  /* Use LE transport when LE is the only available option */
  if (transport == BT_TRANSPORT_LE) {
    return btm_ble_read_remote_name(remote_bda, p_cb);
  }
  /* Use classic transport for BR/EDR and Dual Mode devices */
  return btm_initiate_rem_name(remote_bda, BTM_RMT_NAME_EXT,
                               BTM_EXT_RMT_NAME_TIMEOUT_MS, p_cb);
}

/*******************************************************************************
 *
 * Function         BTM_CancelRemoteDeviceName
 *
 * Description      This function initiates the cancel request for the specified
 *                  remote device.
 *
 * Input Params:    None
 *
 * Returns
 *                  BTM_CMD_STARTED is returned if the request was successfully
 *                                  sent to HCI.
 *                  BTM_NO_RESOURCES if could not allocate resources to start
 *                                   the command
 *                  BTM_WRONG_MODE if there is not an active remote name
 *                                 request.
 *
 ******************************************************************************/
tBTM_STATUS BTM_CancelRemoteDeviceName(void) {
  tBTM_INQUIRY_VAR_ST* p_inq = &btm_cb.btm_inq_vars;

  BTM_TRACE_API("BTM_CancelRemoteDeviceName()");

  /* Make sure there is not already one in progress */
  if (p_inq->remname_active) {
    if (BTM_UseLeLink(p_inq->remname_bda)) {
      /* Cancel remote name request for LE device, and process remote name
       * callback. */
      btm_inq_rmt_name_failed_cancelled();
    } else
      btsnd_hcic_rmt_name_req_cancel(p_inq->remname_bda);
    return (BTM_CMD_STARTED);
  } else
    return (BTM_WRONG_MODE);
}

/*******************************************************************************
 *
 * Function         BTM_InqDbRead
 *
 * Description      This function looks through the inquiry database for a match
 *                  based on Bluetooth Device Address. This is the application's
 *                  interface to get the inquiry details of a specific BD
 *                  address.
 *
 * Returns          pointer to entry, or NULL if not found
 *
 ******************************************************************************/
tBTM_INQ_INFO* BTM_InqDbRead(const RawAddress& p_bda) {
  VLOG(1) << __func__ << ": bd addr " << p_bda;

  tINQ_DB_ENT* p_ent = btm_inq_db_find(p_bda);
  if (!p_ent) return NULL;

  return &p_ent->inq_info;
}

/*******************************************************************************
 *
 * Function         BTM_InqDbFirst
 *
 * Description      This function looks through the inquiry database for the
 *                  first used entry, and returns that. This is used in
 *                  conjunction with
 *                  BTM_InqDbNext by applications as a way to walk through the
 *                  inquiry database.
 *
 * Returns          pointer to first in-use entry, or NULL if DB is empty
 *
 ******************************************************************************/
tBTM_INQ_INFO* BTM_InqDbFirst(void) {
  uint16_t xx;
  tINQ_DB_ENT* p_ent = btm_cb.btm_inq_vars.inq_db;

  for (xx = 0; xx < BTM_INQ_DB_SIZE; xx++, p_ent++) {
    if (p_ent->in_use) return (&p_ent->inq_info);
  }

  /* If here, no used entry found */
  return ((tBTM_INQ_INFO*)NULL);
}

/*******************************************************************************
 *
 * Function         BTM_InqDbNext
 *
 * Description      This function looks through the inquiry database for the
 *                  next used entry, and returns that.  If the input parameter
 *                  is NULL, the first entry is returned.
 *
 * Returns          pointer to next in-use entry, or NULL if no more found.
 *
 ******************************************************************************/
tBTM_INQ_INFO* BTM_InqDbNext(tBTM_INQ_INFO* p_cur) {
  tINQ_DB_ENT* p_ent;
  uint16_t inx;

  if (p_cur) {
    p_ent = (tINQ_DB_ENT*)((uint8_t*)p_cur - offsetof(tINQ_DB_ENT, inq_info));
    inx = (uint16_t)((p_ent - btm_cb.btm_inq_vars.inq_db) + 1);

    for (p_ent = &btm_cb.btm_inq_vars.inq_db[inx]; inx < BTM_INQ_DB_SIZE;
         inx++, p_ent++) {
      if (p_ent->in_use) return (&p_ent->inq_info);
    }

    /* If here, more entries found */
    return ((tBTM_INQ_INFO*)NULL);
  } else
    return (BTM_InqDbFirst());
}

/*******************************************************************************
 *
 * Function         BTM_ClearInqDb
 *
 * Description      This function is called to clear out a device or all devices
 *                  from the inquiry database.
 *
 * Parameter        p_bda - (input) BD_ADDR ->  Address of device to clear
 *                                              (NULL clears all entries)
 *
 * Returns          BTM_BUSY if an inquiry, get remote name, or event filter
 *                          is active, otherwise BTM_SUCCESS
 *
 ******************************************************************************/
tBTM_STATUS BTM_ClearInqDb(const RawAddress* p_bda) {
  tBTM_INQUIRY_VAR_ST* p_inq = &btm_cb.btm_inq_vars;

  /* If an inquiry or remote name is in progress return busy */
  if (p_inq->inq_active != BTM_INQUIRY_INACTIVE) return (BTM_BUSY);

  btm_clr_inq_db(p_bda);

  return (BTM_SUCCESS);
}

/*******************************************************************************
 *******************************************************************************
 *                                                                            **
 *                    BTM Internal Inquiry Functions                          **
 *                                                                            **
 *******************************************************************************
 ******************************************************************************/
/*******************************************************************************
 *
 * Function         btm_inq_db_reset
 *
 * Description      This function is called at at reset to clear the inquiry
 *                  database & pending callback.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_inq_db_reset(void) {
  tBTM_REMOTE_DEV_NAME rem_name;
  tBTM_INQUIRY_VAR_ST* p_inq = &btm_cb.btm_inq_vars;
  uint8_t num_responses;
  uint8_t temp_inq_active;

  /* If an inquiry or periodic inquiry is active, reset the mode to inactive */
  if (p_inq->inq_active != BTM_INQUIRY_INACTIVE) {
    temp_inq_active = p_inq->inq_active; /* Save so state can change BEFORE
                                                callback is called */
    p_inq->inq_active = BTM_INQUIRY_INACTIVE;

    /* If not a periodic inquiry, the complete callback must be called to notify
     * caller */
    if (temp_inq_active == BTM_GENERAL_INQUIRY_ACTIVE) {
      if (p_inq->p_inq_cmpl_cb) {
        num_responses = 0;
        (*p_inq->p_inq_cmpl_cb)(&num_responses);
      }
    }
  }

  /* Cancel a remote name request if active, and notify the caller (if waiting)
   */
  if (p_inq->remname_active) {
    alarm_cancel(p_inq->remote_name_timer);
    p_inq->remname_active = false;
    p_inq->remname_bda = RawAddress::kEmpty;

    if (p_inq->p_remname_cmpl_cb) {
      rem_name.status = BTM_DEV_RESET;

      (*p_inq->p_remname_cmpl_cb)(&rem_name);
      p_inq->p_remname_cmpl_cb = NULL;
    }
  }

  p_inq->state = BTM_INQ_INACTIVE_STATE;
  p_inq->p_inq_results_cb = NULL;
  btm_clr_inq_db(NULL); /* Clear out all the entries in the database */
  btm_clr_inq_result_flt();

  p_inq->discoverable_mode = BTM_NON_DISCOVERABLE;
  p_inq->connectable_mode = BTM_NON_CONNECTABLE;
  p_inq->page_scan_type = BTM_SCAN_TYPE_STANDARD;
  p_inq->inq_scan_type = BTM_SCAN_TYPE_STANDARD;

  p_inq->discoverable_mode |= BTM_BLE_NON_DISCOVERABLE;
  p_inq->connectable_mode |= BTM_BLE_NON_CONNECTABLE;
  return;
}

/*******************************************************************************
 *
 * Function         btm_inq_db_init
 *
 * Description      This function is called at startup to initialize the inquiry
 *                  database.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_inq_db_init(void) {
  alarm_free(btm_cb.btm_inq_vars.remote_name_timer);
  btm_cb.btm_inq_vars.remote_name_timer =
      alarm_new("btm_inq.remote_name_timer");
  btm_cb.btm_inq_vars.no_inc_ssp = BTM_NO_SSP_ON_INQUIRY;
}

void btm_inq_db_free(void) {
  alarm_free(btm_cb.btm_inq_vars.remote_name_timer);
}

/*******************************************************************************
 *
 * Function         btm_inq_stop_on_ssp
 *
 * Description      This function is called on incoming SSP
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_inq_stop_on_ssp(void) {
  uint8_t normal_active = (BTM_GENERAL_INQUIRY_ACTIVE);

#if (BTM_INQ_DEBUG == TRUE)
  BTM_TRACE_DEBUG(
      "btm_inq_stop_on_ssp: no_inc_ssp=%d inq_active:0x%x state:%d ",
      btm_cb.btm_inq_vars.no_inc_ssp, btm_cb.btm_inq_vars.inq_active,
      btm_cb.btm_inq_vars.state);
#endif
  if (btm_cb.btm_inq_vars.no_inc_ssp) {
    if (btm_cb.btm_inq_vars.state == BTM_INQ_ACTIVE_STATE) {
      if (btm_cb.btm_inq_vars.inq_active & normal_active) {
        /* can not call BTM_CancelInquiry() here. We need to report inquiry
         * complete evt */
        bluetooth::legacy::hci::GetInterface().InquiryCancel();
      }
    }
    /* do not allow inquiry to start */
    btm_cb.btm_inq_vars.inq_active |= BTM_SSP_INQUIRY_ACTIVE;
  }
}

/*******************************************************************************
 *
 * Function         btm_inq_clear_ssp
 *
 * Description      This function is called when pairing_state becomes idle
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_inq_clear_ssp(void) {
  btm_cb.btm_inq_vars.inq_active &= ~BTM_SSP_INQUIRY_ACTIVE;
}

/*******************************************************************************
 *
 * Function         btm_clr_inq_db
 *
 * Description      This function is called to clear out a device or all devices
 *                  from the inquiry database.
 *
 * Parameter        p_bda - (input) BD_ADDR ->  Address of device to clear
 *                                              (NULL clears all entries)
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_clr_inq_db(const RawAddress* p_bda) {
  tBTM_INQUIRY_VAR_ST* p_inq = &btm_cb.btm_inq_vars;
  tINQ_DB_ENT* p_ent = p_inq->inq_db;
  uint16_t xx;

#if (BTM_INQ_DEBUG == TRUE)
  BTM_TRACE_DEBUG("btm_clr_inq_db: inq_active:0x%x state:%d",
                  btm_cb.btm_inq_vars.inq_active, btm_cb.btm_inq_vars.state);
#endif
  for (xx = 0; xx < BTM_INQ_DB_SIZE; xx++, p_ent++) {
    if (p_ent->in_use) {
      /* If this is the specified BD_ADDR or clearing all devices */
      if (p_bda == NULL || (p_ent->inq_info.results.remote_bd_addr == *p_bda)) {
        p_ent->in_use = false;
      }
    }
  }
#if (BTM_INQ_DEBUG == TRUE)
  BTM_TRACE_DEBUG("inq_active:0x%x state:%d", btm_cb.btm_inq_vars.inq_active,
                  btm_cb.btm_inq_vars.state);
#endif
}

/*******************************************************************************
 *
 * Function         btm_clr_inq_result_flt
 *
 * Description      This function looks through the bdaddr database for a match
 *                  based on Bluetooth Device Address
 *
 * Returns          true if found, else false (new entry)
 *
 ******************************************************************************/
void btm_clr_inq_result_flt(void) {
  tBTM_INQUIRY_VAR_ST* p_inq = &btm_cb.btm_inq_vars;

  osi_free_and_reset((void**)&p_inq->p_bd_db);
  p_inq->num_bd_entries = 0;
  p_inq->max_bd_entries = 0;
}

/*******************************************************************************
 *
 * Function         btm_inq_find_bdaddr
 *
 * Description      This function looks through the bdaddr database for a match
 *                  based on Bluetooth Device Address
 *
 * Returns          true if found, else false (new entry)
 *
 ******************************************************************************/
bool btm_inq_find_bdaddr(const RawAddress& p_bda) {
  tBTM_INQUIRY_VAR_ST* p_inq = &btm_cb.btm_inq_vars;
  tINQ_BDADDR* p_db = &p_inq->p_bd_db[0];
  uint16_t xx;

  /* Don't bother searching, database doesn't exist or periodic mode */
  if (!p_db) return (false);

  for (xx = 0; xx < p_inq->num_bd_entries; xx++, p_db++) {
    if (p_db->bd_addr == p_bda && p_db->inq_count == p_inq->inq_counter)
      return (true);
  }

  if (xx < p_inq->max_bd_entries) {
    p_db->inq_count = p_inq->inq_counter;
    p_db->bd_addr = p_bda;
    p_inq->num_bd_entries++;
  }

  /* If here, New Entry */
  return (false);
}

/*******************************************************************************
 *
 * Function         btm_inq_db_find
 *
 * Description      This function looks through the inquiry database for a match
 *                  based on Bluetooth Device Address
 *
 * Returns          pointer to entry, or NULL if not found
 *
 ******************************************************************************/
tINQ_DB_ENT* btm_inq_db_find(const RawAddress& p_bda) {
  uint16_t xx;
  tINQ_DB_ENT* p_ent = btm_cb.btm_inq_vars.inq_db;

  for (xx = 0; xx < BTM_INQ_DB_SIZE; xx++, p_ent++) {
    if (p_ent->in_use && p_ent->inq_info.results.remote_bd_addr == p_bda)
      return (p_ent);
  }

  /* If here, not found */
  return (NULL);
}

/*******************************************************************************
 *
 * Function         btm_inq_db_new
 *
 * Description      This function looks through the inquiry database for an
 *                  unused entry. If no entry is free, it allocates the oldest
 *                  entry.
 *
 * Returns          pointer to entry
 *
 ******************************************************************************/
tINQ_DB_ENT* btm_inq_db_new(const RawAddress& p_bda) {
  uint16_t xx;
  tINQ_DB_ENT* p_ent = btm_cb.btm_inq_vars.inq_db;
  tINQ_DB_ENT* p_old = btm_cb.btm_inq_vars.inq_db;
  uint64_t ot = UINT64_MAX;

  for (xx = 0; xx < BTM_INQ_DB_SIZE; xx++, p_ent++) {
    if (!p_ent->in_use) {
      memset(p_ent, 0, sizeof(tINQ_DB_ENT));
      p_ent->inq_info.results.remote_bd_addr = p_bda;
      p_ent->in_use = true;

      return (p_ent);
    }

    if (p_ent->time_of_resp < ot) {
      p_old = p_ent;
      ot = p_ent->time_of_resp;
    }
  }

  /* If here, no free entry found. Return the oldest. */

  memset(p_old, 0, sizeof(tINQ_DB_ENT));
  p_old->inq_info.results.remote_bd_addr = p_bda;
  p_old->in_use = true;

  return (p_old);
}

/*******************************************************************************
 *
 * Function         btm_process_inq_results
 *
 * Description      This function is called when inquiry results are received
 *                  from the device. It updates the inquiry database. If the
 *                  inquiry database is full, the oldest entry is discarded.
 *
 * Parameters       inq_res_mode - BTM_INQ_RESULT_STANDARD
 *                                 BTM_INQ_RESULT_WITH_RSSI
 *                                 BTM_INQ_RESULT_EXTENDED
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_process_inq_results(uint8_t* p, uint8_t hci_evt_len,
                             uint8_t inq_res_mode) {
  uint8_t num_resp, xx;
  RawAddress bda;
  tINQ_DB_ENT* p_i;
  tBTM_INQ_RESULTS* p_cur = NULL;
  bool is_new = true;
  bool update = false;
  int8_t i_rssi;
  tBTM_INQUIRY_VAR_ST* p_inq = &btm_cb.btm_inq_vars;
  tBTM_INQ_RESULTS_CB* p_inq_results_cb = p_inq->p_inq_results_cb;
  uint8_t page_scan_rep_mode = 0;
  uint8_t page_scan_per_mode = 0;
  uint8_t page_scan_mode = 0;
  uint8_t rssi = 0;
  DEV_CLASS dc;
  uint16_t clock_offset;
  uint8_t* p_eir_data = NULL;

#if (BTM_INQ_DEBUG == TRUE)
  BTM_TRACE_DEBUG("btm_process_inq_results inq_active:0x%x state:%d",
                  btm_cb.btm_inq_vars.inq_active, btm_cb.btm_inq_vars.state);
#endif
  /* Only process the results if the BR inquiry is still active */
  if (!(p_inq->inq_active & BTM_BR_INQ_ACTIVE_MASK)) return;

  STREAM_TO_UINT8(num_resp, p);

  if (inq_res_mode == BTM_INQ_RESULT_EXTENDED) {
    if (num_resp > 1) {
      BTM_TRACE_ERROR("btm_process_inq_results() extended results (%d) > 1",
                      num_resp);
      return;
    }

    constexpr uint16_t extended_inquiry_result_size = 254;
    if (hci_evt_len - 1 != extended_inquiry_result_size) {
      android_errorWriteLog(0x534e4554, "141620271");
      BTM_TRACE_ERROR("%s: can't fit %d results in %d bytes", __func__,
                      num_resp, hci_evt_len);
      return;
    }
  } else if (inq_res_mode == BTM_INQ_RESULT_STANDARD ||
             inq_res_mode == BTM_INQ_RESULT_WITH_RSSI) {
    constexpr uint16_t inquiry_result_size = 14;
    if (hci_evt_len < num_resp * inquiry_result_size) {
      android_errorWriteLog(0x534e4554, "141620271");
      BTM_TRACE_ERROR("%s: can't fit %d results in %d bytes", __func__,
                      num_resp, hci_evt_len);
      return;
    }
  }

  for (xx = 0; xx < num_resp; xx++) {
    update = false;
    /* Extract inquiry results */
    STREAM_TO_BDADDR(bda, p);
    STREAM_TO_UINT8(page_scan_rep_mode, p);
    STREAM_TO_UINT8(page_scan_per_mode, p);

    if (inq_res_mode == BTM_INQ_RESULT_STANDARD) {
      STREAM_TO_UINT8(page_scan_mode, p);
    }

    STREAM_TO_DEVCLASS(dc, p);
    STREAM_TO_UINT16(clock_offset, p);
    if (inq_res_mode != BTM_INQ_RESULT_STANDARD) {
      STREAM_TO_UINT8(rssi, p);
    }

    p_i = btm_inq_db_find(bda);

    /* Check if this address has already been processed for this inquiry */
    if (btm_inq_find_bdaddr(bda)) {
      /* BTM_TRACE_DEBUG("BDA seen before %s", bda.ToString().c_str()); */

      /* By default suppose no update needed */
      i_rssi = (int8_t)rssi;

      /* If this new RSSI is higher than the last one */
      if ((rssi != 0) && p_i &&
          (i_rssi > p_i->inq_info.results.rssi ||
           p_i->inq_info.results.rssi == 0
           /* BR/EDR inquiry information update */
           ||
           (p_i->inq_info.results.device_type & BT_DEVICE_TYPE_BREDR) != 0)) {
        p_cur = &p_i->inq_info.results;
        BTM_TRACE_DEBUG("update RSSI new:%d, old:%d", i_rssi, p_cur->rssi);
        p_cur->rssi = i_rssi;
        update = true;
      }
      /* If we received a second Extended Inq Event for an already */
      /* discovered device, this is because for the first one EIR was not
         received */
      else if ((inq_res_mode == BTM_INQ_RESULT_EXTENDED) && (p_i)) {
        p_cur = &p_i->inq_info.results;
        update = true;
      }
      /* If no update needed continue with next response (if any) */
      else
        continue;
    }

    /* If existing entry, use that, else get a new one (possibly reusing the
     * oldest) */
    if (p_i == NULL) {
      p_i = btm_inq_db_new(bda);
      is_new = true;
    }

    /* If an entry for the device already exists, overwrite it ONLY if it is
       from
       a previous inquiry. (Ignore it if it is a duplicate response from the
       same
       inquiry.
    */
    else if (p_i->inq_count == p_inq->inq_counter &&
             (p_i->inq_info.results.device_type == BT_DEVICE_TYPE_BREDR))
      is_new = false;

    /* keep updating RSSI to have latest value */
    if (inq_res_mode != BTM_INQ_RESULT_STANDARD)
      p_i->inq_info.results.rssi = (int8_t)rssi;
    else
      p_i->inq_info.results.rssi = BTM_INQ_RES_IGNORE_RSSI;

    if (is_new) {
      /* Save the info */
      p_cur = &p_i->inq_info.results;
      p_cur->page_scan_rep_mode = page_scan_rep_mode;
      p_cur->page_scan_per_mode = page_scan_per_mode;
      p_cur->page_scan_mode = page_scan_mode;
      p_cur->dev_class[0] = dc[0];
      p_cur->dev_class[1] = dc[1];
      p_cur->dev_class[2] = dc[2];
      p_cur->clock_offset = clock_offset | BTM_CLOCK_OFFSET_VALID;

      p_i->time_of_resp = bluetooth::common::time_get_os_boottime_ms();

      if (p_i->inq_count != p_inq->inq_counter)
        p_inq->inq_cmpl_info.num_resp++; /* A new response was found */

      p_cur->inq_result_type |= BTM_INQ_RESULT_BR;
      if (p_i->inq_count != p_inq->inq_counter) {
        p_cur->device_type = BT_DEVICE_TYPE_BREDR;
        p_i->scan_rsp = false;
      } else
        p_cur->device_type |= BT_DEVICE_TYPE_BREDR;
      p_i->inq_count = p_inq->inq_counter; /* Mark entry for current inquiry */

      /* Initialize flag to false. This flag is set/used by application */
      p_i->inq_info.appl_knows_rem_name = false;
    }

    if (is_new || update) {
      if (inq_res_mode == BTM_INQ_RESULT_EXTENDED) {
        memset(p_cur->eir_uuid, 0,
               BTM_EIR_SERVICE_ARRAY_SIZE * (BTM_EIR_ARRAY_BITS / 8));
        /* set bit map of UUID list from received EIR */
        btm_set_eir_uuid(p, p_cur);
        p_eir_data = p;
      } else
        p_eir_data = NULL;

      /* If a callback is registered, call it with the results */
      if (p_inq_results_cb) {
        (p_inq_results_cb)((tBTM_INQ_RESULTS*)p_cur, p_eir_data,
                           HCI_EXT_INQ_RESPONSE_LEN);
      } else {
        BTM_TRACE_DEBUG("No callback is registered");
      }
    }
  }
}

/*******************************************************************************
 *
 * Function         btm_sort_inq_result
 *
 * Description      This function is called when inquiry complete is received
 *                  from the device to sort inquiry results based on rssi.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_sort_inq_result(void) {
  uint8_t xx, yy, num_resp;
  tINQ_DB_ENT* p_ent = btm_cb.btm_inq_vars.inq_db;
  tINQ_DB_ENT* p_next = btm_cb.btm_inq_vars.inq_db + 1;
  int size;
  tINQ_DB_ENT* p_tmp = (tINQ_DB_ENT*)osi_malloc(sizeof(tINQ_DB_ENT));

  num_resp = (btm_cb.btm_inq_vars.inq_cmpl_info.num_resp < BTM_INQ_DB_SIZE)
                 ? btm_cb.btm_inq_vars.inq_cmpl_info.num_resp
                 : BTM_INQ_DB_SIZE;

  size = sizeof(tINQ_DB_ENT);
  for (xx = 0; xx < num_resp - 1; xx++, p_ent++) {
    for (yy = xx + 1, p_next = p_ent + 1; yy < num_resp; yy++, p_next++) {
      if (p_ent->inq_info.results.rssi < p_next->inq_info.results.rssi) {
        memcpy(p_tmp, p_next, size);
        memcpy(p_next, p_ent, size);
        memcpy(p_ent, p_tmp, size);
      }
    }
  }

  osi_free(p_tmp);
}

/*******************************************************************************
 *
 * Function         btm_process_inq_complete
 *
 * Description      This function is called when inquiry complete is received
 *                  from the device.  Call the callback if not in periodic
 *                  inquiry mode AND it is not NULL
 *                  (The caller wants the event).
 *
 *                  The callback pass back the status and the number of
 *                  responses
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_process_inq_complete(uint8_t status, uint8_t mode) {
  tBTM_CMPL_CB* p_inq_cb = btm_cb.btm_inq_vars.p_inq_cmpl_cb;
  tBTM_INQUIRY_VAR_ST* p_inq = &btm_cb.btm_inq_vars;

  p_inq->inqparms.mode &= ~(mode);

#if (BTM_INQ_DEBUG == TRUE)
  BTM_TRACE_DEBUG("btm_process_inq_complete inq_active:0x%x state:%d",
                  btm_cb.btm_inq_vars.inq_active, btm_cb.btm_inq_vars.state);
#endif
  btm_acl_update_inquiry_status(BTM_INQUIRY_COMPLETE);
  /* Ignore any stray or late complete messages if the inquiry is not active */
  if (p_inq->inq_active) {
    p_inq->inq_cmpl_info.status = (tBTM_STATUS)(
        (status == HCI_SUCCESS) ? BTM_SUCCESS : BTM_ERR_PROCESSING);

    /* Notify caller that the inquiry has completed; (periodic inquiries do not
     * send completion events */
    if (p_inq->inqparms.mode == 0) {
      btm_clear_all_pending_le_entry();
      p_inq->state = BTM_INQ_INACTIVE_STATE;

      /* Increment so the start of a next inquiry has a new count */
      p_inq->inq_counter++;

      btm_clr_inq_result_flt();

      if ((p_inq->inq_cmpl_info.status == BTM_SUCCESS) &&
          controller_get_interface()->supports_rssi_with_inquiry_results()) {
        btm_sort_inq_result();
      }

      /* Clear the results callback if set */
      p_inq->p_inq_results_cb = NULL;
      p_inq->inq_active = BTM_INQUIRY_INACTIVE;
      p_inq->p_inq_cmpl_cb = NULL;

      /* If we have a callback registered for inquiry complete, call it */
      BTM_TRACE_DEBUG("BTM Inq Compl Callback: status 0x%02x, num results %d",
                      p_inq->inq_cmpl_info.status,
                      p_inq->inq_cmpl_info.num_resp);

      if (p_inq_cb) (p_inq_cb)((tBTM_INQUIRY_CMPL*)&p_inq->inq_cmpl_info);
    }
  }
#if (BTM_INQ_DEBUG == TRUE)
  BTM_TRACE_DEBUG("inq_active:0x%x state:%d", btm_cb.btm_inq_vars.inq_active,
                  btm_cb.btm_inq_vars.state);
#endif
}

/*******************************************************************************
 *
 * Function         btm_process_cancel_complete
 *
 * Description      This function is called when inquiry cancel complete is
 *                  received from the device. This function will also call the
 *                  btm_process_inq_complete. This function is needed to
 *                  differentiate a cancel_cmpl_evt from the inq_cmpl_evt.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_process_cancel_complete(uint8_t status, uint8_t mode) {
  btm_acl_update_inquiry_status(BTM_INQUIRY_CANCELLED);
  btm_process_inq_complete(status, mode);
}
/*******************************************************************************
 *
 * Function         btm_initiate_rem_name
 *
 * Description      This function looks initiates a remote name request.  It is
 *                  called either by GAP or by the API call
 *                  BTM_ReadRemoteDeviceName.
 *
 * Input Params:    p_cb            - callback function called when
 *                                    BTM_CMD_STARTED is returned.
 *                                    A pointer to tBTM_REMOTE_DEV_NAME is
 *                                    passed to the callback.
 *
 * Returns
 *                  BTM_CMD_STARTED is returned if the request was sent to HCI.
 *                  BTM_BUSY if already in progress
 *                  BTM_NO_RESOURCES if could not allocate resources to start
 *                                   the command
 *                  BTM_WRONG_MODE if the device is not up.
 *
 ******************************************************************************/
tBTM_STATUS btm_initiate_rem_name(const RawAddress& remote_bda, uint8_t origin,
                                  uint64_t timeout_ms, tBTM_CMPL_CB* p_cb) {
  tBTM_INQUIRY_VAR_ST* p_inq = &btm_cb.btm_inq_vars;

  /*** Make sure the device is ready ***/
  if (!BTM_IsDeviceUp()) return (BTM_WRONG_MODE);
  if (origin == BTM_RMT_NAME_EXT) {
    if (p_inq->remname_active) {
      return (BTM_BUSY);
    } else {
      /* If there is no remote name request running,call the callback function
       * and start timer */
      p_inq->p_remname_cmpl_cb = p_cb;
      p_inq->remname_bda = remote_bda;

      alarm_set_on_mloop(p_inq->remote_name_timer, timeout_ms,
                         btm_inq_remote_name_timer_timeout, NULL);

      /* If the database entry exists for the device, use its clock offset */
      tINQ_DB_ENT* p_i = btm_inq_db_find(remote_bda);
      if (p_i && (p_i->inq_info.results.inq_result_type & BTM_INQ_RESULT_BR)) {
        tBTM_INQ_INFO* p_cur = &p_i->inq_info;
        btsnd_hcic_rmt_name_req(
            remote_bda, p_cur->results.page_scan_rep_mode,
            p_cur->results.page_scan_mode,
            (uint16_t)(p_cur->results.clock_offset | BTM_CLOCK_OFFSET_VALID));
      } else {
        /* Otherwise use defaults and mark the clock offset as invalid */
        btsnd_hcic_rmt_name_req(remote_bda, HCI_PAGE_SCAN_REP_MODE_R1,
                                HCI_MANDATARY_PAGE_SCAN_MODE, 0);
      }

      p_inq->remname_active = true;
      return BTM_CMD_STARTED;
    }
  } else {
    return BTM_ILLEGAL_VALUE;
  }
}

/*******************************************************************************
 *
 * Function         btm_process_remote_name
 *
 * Description      This function is called when a remote name is received from
 *                  the device. If remote names are cached, it updates the
 *                  inquiry database.
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_process_remote_name(const RawAddress* bda, BD_NAME bdn,
                             uint16_t evt_len, uint8_t hci_status) {
  tBTM_REMOTE_DEV_NAME rem_name;
  tBTM_INQUIRY_VAR_ST* p_inq = &btm_cb.btm_inq_vars;
  tBTM_CMPL_CB* p_cb = p_inq->p_remname_cmpl_cb;
  uint8_t* p_n1;

  uint16_t temp_evt_len;

  if (bda) {
    VLOG(2) << "BDA " << *bda;
  }

  VLOG(2) << "Inquire BDA " << p_inq->remname_bda;

  /* If the inquire BDA and remote DBA are the same, then stop the timer and set
   * the active to false */
  if ((p_inq->remname_active) && (!bda || (*bda == p_inq->remname_bda))) {
    if (BTM_UseLeLink(p_inq->remname_bda)) {
      if (hci_status == HCI_ERR_UNSPECIFIED)
        btm_ble_cancel_remote_name(p_inq->remname_bda);
    }
    alarm_cancel(p_inq->remote_name_timer);
    p_inq->remname_active = false;
    /* Clean up and return the status if the command was not successful */
    /* Note: If part of the inquiry, the name is not stored, and the    */
    /*       inquiry complete callback is called.                       */

    if (hci_status == HCI_SUCCESS) {
      /* Copy the name from the data stream into the return structure */
      /* Note that even if it is not being returned, it is used as a  */
      /*      temporary buffer.                                       */
      p_n1 = (uint8_t*)rem_name.remote_bd_name;
      rem_name.length = (evt_len < BD_NAME_LEN) ? evt_len : BD_NAME_LEN;
      rem_name.remote_bd_name[rem_name.length] = 0;
      rem_name.status = BTM_SUCCESS;
      temp_evt_len = rem_name.length;

      while (temp_evt_len > 0) {
        *p_n1++ = *bdn++;
        temp_evt_len--;
      }
      rem_name.remote_bd_name[rem_name.length] = 0;
    }

    /* If processing a stand alone remote name then report the error in the
       callback */
    else {
      rem_name.status = BTM_BAD_VALUE_RET;
      rem_name.length = 0;
      rem_name.remote_bd_name[0] = 0;
    }
    /* Reset the remote BAD to zero and call callback if possible */
    p_inq->remname_bda = RawAddress::kEmpty;

    p_inq->p_remname_cmpl_cb = NULL;
    if (p_cb) (p_cb)(&rem_name);
  }
}

void btm_inq_remote_name_timer_timeout(UNUSED_ATTR void* data) {
  btm_inq_rmt_name_failed_cancelled();
}

/*******************************************************************************
 *
 * Function         btm_inq_rmt_name_failed_cancelled
 *
 * Description      This function is if timeout expires or request is cancelled
 *                  while getting remote name.  This is done for devices that
 *                  incorrectly do not report operation failure
 *
 * Returns          void
 *
 ******************************************************************************/
void btm_inq_rmt_name_failed_cancelled(void) {
  BTM_TRACE_ERROR("btm_inq_rmt_name_failed_cancelled()  remname_active=%d",
                  btm_cb.btm_inq_vars.remname_active);

  if (btm_cb.btm_inq_vars.remname_active) {
    btm_process_remote_name(&btm_cb.btm_inq_vars.remname_bda, NULL, 0,
                            HCI_ERR_UNSPECIFIED);
  }

  btm_sec_rmt_name_request_complete(NULL, NULL, HCI_ERR_UNSPECIFIED);
}

/*******************************************************************************
 *
 * Function         BTM_WriteEIR
 *
 * Description      This function is called to write EIR data to controller.
 *
 * Parameters       p_buff - allocated HCI command buffer including extended
 *                           inquriry response
 *
 * Returns          BTM_SUCCESS  - if successful
 *                  BTM_MODE_UNSUPPORTED - if local device cannot support it
 *
 ******************************************************************************/
tBTM_STATUS BTM_WriteEIR(BT_HDR* p_buff) {
  if (controller_get_interface()->supports_extended_inquiry_response()) {
    BTM_TRACE_API("Write Extended Inquiry Response to controller");
    btsnd_hcic_write_ext_inquiry_response(p_buff, BTM_EIR_DEFAULT_FEC_REQUIRED);
    return BTM_SUCCESS;
  } else {
    osi_free(p_buff);
    return BTM_MODE_UNSUPPORTED;
  }
}

/*******************************************************************************
 *
 * Function         btm_convert_uuid_to_eir_service
 *
 * Description      This function is called to get the bit position of UUID.
 *
 * Parameters       uuid16 - UUID 16-bit
 *
 * Returns          BTM EIR service ID if found
 *                  BTM_EIR_MAX_SERVICES - if not found
 *
 ******************************************************************************/
static uint8_t btm_convert_uuid_to_eir_service(uint16_t uuid16) {
  uint8_t xx;

  for (xx = 0; xx < BTM_EIR_MAX_SERVICES; xx++) {
    if (uuid16 == BTM_EIR_UUID_LKUP_TBL[xx]) {
      return xx;
    }
  }
  return BTM_EIR_MAX_SERVICES;
}

/*******************************************************************************
 *
 * Function         BTM_HasEirService
 *
 * Description      This function is called to know if UUID in bit map of UUID.
 *
 * Parameters       p_eir_uuid - bit map of UUID list
 *                  uuid16 - UUID 16-bit
 *
 * Returns          true - if found
 *                  false - if not found
 *
 ******************************************************************************/
bool BTM_HasEirService(const uint32_t* p_eir_uuid, uint16_t uuid16) {
  uint8_t service_id;

  service_id = btm_convert_uuid_to_eir_service(uuid16);
  if (service_id < BTM_EIR_MAX_SERVICES)
    return (BTM_EIR_HAS_SERVICE(p_eir_uuid, service_id));
  else
    return (false);
}

/*******************************************************************************
 *
 * Function         BTM_HasInquiryEirService
 *
 * Description      This function is called to know if UUID in bit map of UUID
 *                  list.
 *
 * Parameters       p_results - inquiry results
 *                  uuid16 - UUID 16-bit
 *
 * Returns          BTM_EIR_FOUND - if found
 *                  BTM_EIR_NOT_FOUND - if not found and it is complete list
 *                  BTM_EIR_UNKNOWN - if not found and it is not complete list
 *
 ******************************************************************************/
tBTM_EIR_SEARCH_RESULT BTM_HasInquiryEirService(tBTM_INQ_RESULTS* p_results,
                                                uint16_t uuid16) {
  if (BTM_HasEirService(p_results->eir_uuid, uuid16)) {
    return BTM_EIR_FOUND;
  } else if (p_results->eir_complete_list) {
    return BTM_EIR_NOT_FOUND;
  } else
    return BTM_EIR_UNKNOWN;
}

/*******************************************************************************
 *
 * Function         BTM_AddEirService
 *
 * Description      This function is called to add a service in bit map of UUID
 *                  list.
 *
 * Parameters       p_eir_uuid - bit mask of UUID list for EIR
 *                  uuid16 - UUID 16-bit
 *
 * Returns          None
 *
 ******************************************************************************/
void BTM_AddEirService(uint32_t* p_eir_uuid, uint16_t uuid16) {
  uint8_t service_id;

  service_id = btm_convert_uuid_to_eir_service(uuid16);
  if (service_id < BTM_EIR_MAX_SERVICES)
    BTM_EIR_SET_SERVICE(p_eir_uuid, service_id);
}

/*******************************************************************************
 *
 * Function         BTM_RemoveEirService
 *
 * Description      This function is called to remove a service in bit map of
 *                  UUID list.
 *
 * Parameters       p_eir_uuid - bit mask of UUID list for EIR
 *                  uuid16 - UUID 16-bit
 *
 * Returns          None
 *
 ******************************************************************************/
void BTM_RemoveEirService(uint32_t* p_eir_uuid, uint16_t uuid16) {
  uint8_t service_id;

  service_id = btm_convert_uuid_to_eir_service(uuid16);
  if (service_id < BTM_EIR_MAX_SERVICES)
    BTM_EIR_CLR_SERVICE(p_eir_uuid, service_id);
}

/*******************************************************************************
 *
 * Function         BTM_GetEirSupportedServices
 *
 * Description      This function is called to get UUID list from bit map of
 *                  UUID list.
 *
 * Parameters       p_eir_uuid - bit mask of UUID list for EIR
 *                  p - reference of current pointer of EIR
 *                  max_num_uuid16 - max number of UUID can be written in EIR
 *                  num_uuid16 - number of UUID have been written in EIR
 *
 * Returns          BTM_EIR_MORE_16BITS_UUID_TYPE, if it has more than max
 *                  BTM_EIR_COMPLETE_16BITS_UUID_TYPE, otherwise
 *
 ******************************************************************************/
uint8_t BTM_GetEirSupportedServices(uint32_t* p_eir_uuid, uint8_t** p,
                                    uint8_t max_num_uuid16,
                                    uint8_t* p_num_uuid16) {
  uint8_t service_index;

  *p_num_uuid16 = 0;

  for (service_index = 0; service_index < BTM_EIR_MAX_SERVICES;
       service_index++) {
    if (BTM_EIR_HAS_SERVICE(p_eir_uuid, service_index)) {
      if (*p_num_uuid16 < max_num_uuid16) {
        UINT16_TO_STREAM(*p, BTM_EIR_UUID_LKUP_TBL[service_index]);
        (*p_num_uuid16)++;
      }
      /* if max number of UUIDs are stored and found one more */
      else {
        return BTM_EIR_MORE_16BITS_UUID_TYPE;
      }
    }
  }
  return BTM_EIR_COMPLETE_16BITS_UUID_TYPE;
}

/*******************************************************************************
 *
 * Function         BTM_GetEirUuidList
 *
 * Description      This function parses EIR and returns UUID list.
 *
 * Parameters       p_eir - EIR
 *                  eir_len - EIR len
 *                  uuid_size - Uuid::kNumBytes16, Uuid::kNumBytes32,
 *                              Uuid::kNumBytes128
 *                  p_num_uuid - return number of UUID in found list
 *                  p_uuid_list - return UUID list
 *                  max_num_uuid - maximum number of UUID to be returned
 *
 * Returns          0 - if not found
 *                  BTM_EIR_COMPLETE_16BITS_UUID_TYPE
 *                  BTM_EIR_MORE_16BITS_UUID_TYPE
 *                  BTM_EIR_COMPLETE_32BITS_UUID_TYPE
 *                  BTM_EIR_MORE_32BITS_UUID_TYPE
 *                  BTM_EIR_COMPLETE_128BITS_UUID_TYPE
 *                  BTM_EIR_MORE_128BITS_UUID_TYPE
 *
 ******************************************************************************/
uint8_t BTM_GetEirUuidList(uint8_t* p_eir, size_t eir_len, uint8_t uuid_size,
                           uint8_t* p_num_uuid, uint8_t* p_uuid_list,
                           uint8_t max_num_uuid) {
  const uint8_t* p_uuid_data;
  uint8_t type;
  uint8_t yy, xx;
  uint16_t* p_uuid16 = (uint16_t*)p_uuid_list;
  uint32_t* p_uuid32 = (uint32_t*)p_uuid_list;
  char buff[Uuid::kNumBytes128 * 2 + 1];

  p_uuid_data =
      btm_eir_get_uuid_list(p_eir, eir_len, uuid_size, p_num_uuid, &type);
  if (p_uuid_data == NULL) {
    return 0x00;
  }

  if (*p_num_uuid > max_num_uuid) {
    BTM_TRACE_WARNING("%s: number of uuid in EIR = %d, size of uuid list = %d",
                      __func__, *p_num_uuid, max_num_uuid);
    *p_num_uuid = max_num_uuid;
  }

  BTM_TRACE_DEBUG("%s: type = %02X, number of uuid = %d", __func__, type,
                  *p_num_uuid);

  if (uuid_size == Uuid::kNumBytes16) {
    for (yy = 0; yy < *p_num_uuid; yy++) {
      STREAM_TO_UINT16(*(p_uuid16 + yy), p_uuid_data);
      BTM_TRACE_DEBUG("                     0x%04X", *(p_uuid16 + yy));
    }
  } else if (uuid_size == Uuid::kNumBytes32) {
    for (yy = 0; yy < *p_num_uuid; yy++) {
      STREAM_TO_UINT32(*(p_uuid32 + yy), p_uuid_data);
      BTM_TRACE_DEBUG("                     0x%08X", *(p_uuid32 + yy));
    }
  } else if (uuid_size == Uuid::kNumBytes128) {
    for (yy = 0; yy < *p_num_uuid; yy++) {
      STREAM_TO_ARRAY16(p_uuid_list + yy * Uuid::kNumBytes128, p_uuid_data);
      for (xx = 0; xx < Uuid::kNumBytes128; xx++)
        snprintf(buff + xx * 2, sizeof(buff) - xx * 2, "%02X",
                 *(p_uuid_list + yy * Uuid::kNumBytes128 + xx));
      BTM_TRACE_DEBUG("                     0x%s", buff);
    }
  }

  return type;
}

/*******************************************************************************
 *
 * Function         btm_eir_get_uuid_list
 *
 * Description      This function searches UUID list in EIR.
 *
 * Parameters       p_eir - address of EIR
 *                  eir_len - EIR length
 *                  uuid_size - size of UUID to find
 *                  p_num_uuid - number of UUIDs found
 *                  p_uuid_list_type - EIR data type
 *
 * Returns          NULL - if UUID list with uuid_size is not found
 *                  beginning of UUID list in EIR - otherwise
 *
 ******************************************************************************/
static const uint8_t* btm_eir_get_uuid_list(uint8_t* p_eir, size_t eir_len,
                                            uint8_t uuid_size,
                                            uint8_t* p_num_uuid,
                                            uint8_t* p_uuid_list_type) {
  const uint8_t* p_uuid_data;
  uint8_t complete_type, more_type;
  uint8_t uuid_len;

  switch (uuid_size) {
    case Uuid::kNumBytes16:
      complete_type = BTM_EIR_COMPLETE_16BITS_UUID_TYPE;
      more_type = BTM_EIR_MORE_16BITS_UUID_TYPE;
      break;
    case Uuid::kNumBytes32:
      complete_type = BTM_EIR_COMPLETE_32BITS_UUID_TYPE;
      more_type = BTM_EIR_MORE_32BITS_UUID_TYPE;
      break;
    case Uuid::kNumBytes128:
      complete_type = BTM_EIR_COMPLETE_128BITS_UUID_TYPE;
      more_type = BTM_EIR_MORE_128BITS_UUID_TYPE;
      break;
    default:
      *p_num_uuid = 0;
      return NULL;
      break;
  }

  p_uuid_data = AdvertiseDataParser::GetFieldByType(p_eir, eir_len,
                                                    complete_type, &uuid_len);
  if (p_uuid_data == NULL) {
    p_uuid_data = AdvertiseDataParser::GetFieldByType(p_eir, eir_len, more_type,
                                                      &uuid_len);
    *p_uuid_list_type = more_type;
  } else {
    *p_uuid_list_type = complete_type;
  }

  *p_num_uuid = uuid_len / uuid_size;
  return p_uuid_data;
}

/*******************************************************************************
 *
 * Function         btm_convert_uuid_to_uuid16
 *
 * Description      This function converts UUID to UUID 16-bit.
 *
 * Parameters       p_uuid - address of UUID
 *                  uuid_size - size of UUID
 *
 * Returns          0 - if UUID cannot be converted to UUID 16-bit
 *                  UUID 16-bit - otherwise
 *
 ******************************************************************************/
static uint16_t btm_convert_uuid_to_uuid16(const uint8_t* p_uuid,
                                           uint8_t uuid_size) {
  static const uint8_t base_uuid[Uuid::kNumBytes128] = {
      0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
      0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint16_t uuid16 = 0;
  uint32_t uuid32;
  bool is_base_uuid;
  uint8_t xx;

  switch (uuid_size) {
    case Uuid::kNumBytes16:
      STREAM_TO_UINT16(uuid16, p_uuid);
      break;
    case Uuid::kNumBytes32:
      STREAM_TO_UINT32(uuid32, p_uuid);
      if (uuid32 < 0x10000) uuid16 = (uint16_t)uuid32;
      break;
    case Uuid::kNumBytes128:
      /* See if we can compress the UUID down to 16 or 32bit UUIDs */
      is_base_uuid = true;
      for (xx = 0; xx < Uuid::kNumBytes128 - 4; xx++) {
        if (p_uuid[xx] != base_uuid[xx]) {
          is_base_uuid = false;
          break;
        }
      }
      if (is_base_uuid) {
        if ((p_uuid[Uuid::kNumBytes128 - 1] == 0) &&
            (p_uuid[Uuid::kNumBytes128 - 2] == 0)) {
          p_uuid += (Uuid::kNumBytes128 - 4);
          STREAM_TO_UINT16(uuid16, p_uuid);
        }
      }
      break;
    default:
      BTM_TRACE_WARNING("btm_convert_uuid_to_uuid16 invalid uuid size");
      break;
  }

  return (uuid16);
}

/*******************************************************************************
 *
 * Function         btm_set_eir_uuid
 *
 * Description      This function is called to store received UUID into inquiry
 *                  result.
 *
 * Parameters       p_eir - pointer of EIR significant part
 *                  p_results - pointer of inquiry result
 *
 * Returns          None
 *
 ******************************************************************************/
void btm_set_eir_uuid(uint8_t* p_eir, tBTM_INQ_RESULTS* p_results) {
  const uint8_t* p_uuid_data;
  uint8_t num_uuid;
  uint16_t uuid16;
  uint8_t yy;
  uint8_t type = BTM_EIR_MORE_16BITS_UUID_TYPE;

  p_uuid_data = btm_eir_get_uuid_list(p_eir, HCI_EXT_INQ_RESPONSE_LEN,
                                      Uuid::kNumBytes16, &num_uuid, &type);

  if (type == BTM_EIR_COMPLETE_16BITS_UUID_TYPE) {
    p_results->eir_complete_list = true;
  } else {
    p_results->eir_complete_list = false;
  }

  BTM_TRACE_API("btm_set_eir_uuid eir_complete_list=0x%02X",
                p_results->eir_complete_list);

  if (p_uuid_data) {
    for (yy = 0; yy < num_uuid; yy++) {
      STREAM_TO_UINT16(uuid16, p_uuid_data);
      BTM_AddEirService(p_results->eir_uuid, uuid16);
    }
  }

  p_uuid_data = btm_eir_get_uuid_list(p_eir, HCI_EXT_INQ_RESPONSE_LEN,
                                      Uuid::kNumBytes32, &num_uuid, &type);
  if (p_uuid_data) {
    for (yy = 0; yy < num_uuid; yy++) {
      uuid16 = btm_convert_uuid_to_uuid16(p_uuid_data, Uuid::kNumBytes32);
      p_uuid_data += Uuid::kNumBytes32;
      if (uuid16) BTM_AddEirService(p_results->eir_uuid, uuid16);
    }
  }

  p_uuid_data = btm_eir_get_uuid_list(p_eir, HCI_EXT_INQ_RESPONSE_LEN,
                                      Uuid::kNumBytes128, &num_uuid, &type);
  if (p_uuid_data) {
    for (yy = 0; yy < num_uuid; yy++) {
      uuid16 = btm_convert_uuid_to_uuid16(p_uuid_data, Uuid::kNumBytes128);
      p_uuid_data += Uuid::kNumBytes128;
      if (uuid16) BTM_AddEirService(p_results->eir_uuid, uuid16);
    }
  }
}
