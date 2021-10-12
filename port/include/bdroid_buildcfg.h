/*
 * Copyright (C) 2014 The Android Open Source Project
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

#ifndef _BDROID_BUILDCFG_H
#define _BDROID_BUILDCFG_H

#include <nuttx/config.h>

#define BT_RC_NUM_APP               CONFIG_BT_RC_NUM_APP
#define HF_CLIENT_MAX_DEVICES       CONFIG_HF_CLIENT_MAX_DEVICES
#define GATT_CL_MAX_LCB             CONFIG_GATT_CL_MAX_LCB

/******************************************************************************
 *
 * Buffer sizes
 *
 *****************************************************************************/

#define BT_DEFAULT_BUFFER_SIZE      CONFIG_BT_DEFAULT_BUFFER_SIZE

/******************************************************************************
 *
 * BTM
 *
 *****************************************************************************/

/* Default class of device
* {SERVICE_CLASS, MAJOR_CLASS, MINOR_CLASS}
*
* SERVICE_CLASS:0x5A (Bit17 -Networking,Bit19 - Capturing,Bit20 -Object
* Transfer,Bit22 -Telephony)
* MAJOR_CLASS:0x02 - PHONE
* MINOR_CLASS:0x0C - SMART_PHONE
*
*/
#define BTA_DM_COD \
{ CONFIG_FLUORIDE_SERVICE_CLASS, \
  CONFIG_FLUORIDE_MAJOR_CLASS, \
  CONFIG_FLUORIDE_MINOR_CLASS \
}

/* Maximum device name length used in btm database. */
#define BTM_MAX_REM_BD_NAME_LEN     CONFIG_BTM_MAX_REM_BD_NAME_LEN

/* Maximum local device name length stored btm database. */
#define BTM_MAX_LOC_BD_NAME_LEN     CONFIG_BTM_MAX_LOC_BD_NAME_LEN

#define BD_NAME_LEN                 CONFIG_BD_NAME_LEN

/* The number of SCO links. */
#define BTM_MAX_SCO_LINKS           CONFIG_BTM_MAX_SCO_LINKS

/* The number of security records for peer devices. */
#define BTM_SEC_MAX_DEVICE_RECORDS  CONFIG_BTM_SEC_MAX_DEVICE_RECORDS

/* The number of security records for services. */
#define BTM_SEC_MAX_SERVICE_RECORDS CONFIG_BTM_SEC_MAX_SERVICE_RECORDS

/* The size in bytes of the BTM inquiry database. */
#define BTM_INQ_DB_SIZE             CONFIG_BTM_INQ_DB_SIZE

/* The IO capability of the local device (for Simple Pairing) */
#define BTM_LOCAL_IO_CAPS           CONFIG_BTM_LOCAL_IO_CAPS

/* The IO capability of the local device (for BLE Simple Pairing) */
#define BTM_LOCAL_IO_CAPS_BLE       CONFIG_BTM_LOCAL_IO_CAPS_BLE

/* 4.1/4.2 secure connections feature */
#if defined(CONFIG_SC_MODE_INCLUDED)
  #define SC_MODE_INCLUDED          (1)
#else
  #define SC_MODE_INCLUDED          (0)
#endif

/******************************************************************************
 *
 * BTA
 *
 *****************************************************************************/

#define BTA_GATTC_KNOWN_SR_MAX      CONFIG_BTA_GATTC_KNOWN_SR_MAX
#define BTA_GATTC_CL_MAX            CONFIG_BTA_GATTC_CL_MAX
#define BTA_GATTC_NOTIF_REG_MAX     CONFIG_BTA_GATTC_NOTIF_REG_MAX
#define BTA_AG_AT_MAX_LEN           CONFIG_BTA_AG_AT_MAX_LEN

#define BTA_SDP_DB_SIZE             CONFIG_BTA_SDP_DB_SIZE
#define BTA_DM_SDP_DB_SIZE          CONFIG_BTA_DM_SDP_DB_SIZE
#define BTA_JV_SDP_DB_SIZE          CONFIG_BTA_JV_SDP_DB_SIZE
#define BTA_JV_SDP_RAW_DATA_SIZE    CONFIG_BTA_JV_SDP_RAW_DATA_SIZE

#if defined(CONFIG_BTA_AV_SINK_INCLUDED)
  #define BTA_AV_SINK_INCLUDED      (1)
#else
  #define BTA_AV_SINK_INCLUDED      (0)
#endif

/* maximum number of streams created */
#define BTA_AV_NUM_STRS             CONFIG_BTA_AV_NUM_STRS

/* Number of SCBs (AG service instances that can be registered) */
#define BTA_AG_MAX_NUM_CLIENTS      CONFIG_BTA_AG_MAX_NUM_CLIENTS

/******************************************************************************
 *
 * SDP
 *
 *****************************************************************************/

/* The maximum number of SDP records the server can support. */
#define SDP_MAX_RECORDS             CONFIG_SDP_MAX_RECORDS

/* The maximum number of attributes in each record. */
#define SDP_MAX_REC_ATTR            CONFIG_SDP_MAX_REC_ATTR

/* The maximum number of record handles retrieved in a search. */
#define SDP_MAX_DISC_SERVER_RECS    CONFIG_SDP_MAX_DISC_SERVER_RECS

/* The maximum number of simultaneous client and server connections. */
#define SDP_MAX_CONNECTIONS         CONFIG_SDP_MAX_CONNECTIONS

/* keep the raw data received from SDP server in database. */
#if defined(CONFIG_SDP_RAW_DATA_INCLUDED)
  #define SDP_RAW_DATA_INCLUDED     TRUE
#else
  #define SDP_RAW_DATA_INCLUDED     FALSE
#endif

/******************************************************************************
 *
 * AVDTP
 *
 *****************************************************************************/

/* Number of simultaneous links to different peer devices. */
#define AVDT_NUM_LINKS              CONFIG_AVDT_NUM_LINKS

/* Number of simultaneous stream endpoints. */
#define AVDT_NUM_SEPS               CONFIG_AVDT_NUM_SEPS

/* Maximum size in bytes of the content protection information element. */
#define AVDT_PROTECT_SIZE           CONFIG_AVDT_PROTECT_SIZE

/******************************************************************************
 *
 * AVRCP
 *
 *****************************************************************************/

#define AVRCP_DEFAULT_VERSION       CONFIG_AVRCP_DEFAULT_VERSION

/******************************************************************************
 *
 * L2CAP
 *
 *****************************************************************************/

/* The maximum number of simultaneous links that ACL can support. */
#define MAX_ACL_CONNECTIONS         CONFIG_MAX_ACL_CONNECTIONS

/* The maximum number of simultaneous channels that L2CAP can support. */
#define MAX_L2CAP_CHANNELS          CONFIG_MAX_L2CAP_CHANNELS

/* The maximum number of simultaneous applications that can register with L2CAP. */
#define MAX_L2CAP_CLIENTS           CONFIG_MAX_L2CAP_CLIENTS

/******************************************************************************
 *
 * RFCOMM
 *
 *****************************************************************************/

/* Maximum number of RFCOMM channels. */
#define MAX_RFC_CHANNEL             CONFIG_MAX_RFC_CHANNEL

/* The maximum number of ports supported. */
#define MAX_RFC_PORTS               CONFIG_MAX_RFC_PORTS

/******************************************************************************
 *
 * GAP
 *
 *****************************************************************************/

/* The maximum number of simultaneous GAP L2CAP connections. */
#define GAP_MAX_CONNECTIONS         CONFIG_GAP_MAX_CONNECTIONS

/******************************************************************************
 *
 * BNEP
 *
 *****************************************************************************/

/* Enable BNEP support */
#if defined(CONFIG_BNEP_INCLUDED)
  #define BNEP_INCLUDED TRUE
#else
  #define BNEP_INCLUDED FALSE
#endif

/******************************************************************************
 *
 * ATT/GATT Protocol/Profile Settings
 *
 *****************************************************************************/

#define GATT_MAX_PHY_CHANNEL        CONFIG_GATT_MAX_PHY_CHANNEL

#define GATT_MAX_SR_PROFILES        CONFIG_GATT_MAX_SR_PROFILES

#define GATT_MAX_APPS               CONFIG_GATT_MAX_APPS

/******************************************************************************
 *
 * BLE
 *
 *****************************************************************************/

/*
 * Enables or disables support for local privacy (ex. address rotation)
 */
#if defined(CONFIG_BLE_LOCAL_PRIVACY_ENABLED)
  #define BLE_LOCAL_PRIVACY_ENABLED TRUE
#else
  #define BLE_LOCAL_PRIVACY_ENABLED FALSE
#endif

/* The maximum number of simultaneous applications that can register with LE
 * L2CAP. */
#define BLE_MAX_L2CAP_CLIENTS       CONFIG_BLE_MAX_L2CAP_CLIENTS

/******************************************************************************
 *
 * PAN
 *
 *****************************************************************************/

/* Enable PAN support */
#if defined(CONFIG_BTA_PAN_INCLUDED)
  #define BTA_PAN_INCLUDED          TRUE
#else
  #define BTA_PAN_INCLUDED          FALSE
#endif

#define PAN_INCLUDED                BTA_PAN_INCLUDED

/******************************************************************************
 *
 * HID
 *
 *****************************************************************************/

#define HID_HOST_MAX_DEVICES        CONFIG_HID_HOST_MAX_DEVICES

/* Enable HID Host support */
#if defined(CONFIG_BTA_HH_INCLUDED)
  #define BTA_HH_INCLUDED           TRUE
#else
  #define BTA_HH_INCLUDED           FALSE
#endif

/* Enable HID Device support */
#if defined(CONFIG_BTA_HD_INCLUDED)
  #define BTA_HD_INCLUDED           TRUE
#else
  #define BTA_HD_INCLUDED           FALSE
#endif

/******************************************************************************
 *
 * AVCTP
 *
 *****************************************************************************/

/* Number of simultaneous ACL links to different peer devices. */
#define AVCT_NUM_LINKS              CONFIG_AVCT_NUM_LINKS

/* Number of simultaneous AVCTP connections. */
#define AVCT_NUM_CONN               (AVCT_NUM_LINKS * 2 + 2)

/******************************************************************************
 *
 * Hearing AID
 *
 *****************************************************************************/

/* Enable HEARING AID support */
#define BTA_HEARING_AID_INCLUDED    CONFIG_BTA_HEARING_AID_INCLUDED

#endif
