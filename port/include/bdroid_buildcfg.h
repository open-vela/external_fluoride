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

#define BT_RC_NUM_APP               CONFIG_BT_RC_NUM_APP
#define HF_CLIENT_MAX_DEVICES       CONFIG_HF_CLIENT_MAX_DEVICES
#define GATT_CL_MAX_LCB             CONFIG_GATT_CL_MAX_LCB

/******************************************************************************
 *
 * BTM
 *
 *****************************************************************************/

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
#define BTA_DM_SDP_DB_SIZE          CONFIG_BTA_DM_SDP_DB_SIZE

#if defined(CONFIG_BTA_AV_SINK_INCLUDED)
  #define BTA_AV_SINK_INCLUDED      (1)
#else
  #define BTA_AV_SINK_INCLUDED      (0)
#endif

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

/******************************************************************************
 *
 * AVDTP
 *
 *****************************************************************************/

/* Number of simultaneous links to different peer devices. */
#define AVDT_NUM_LINKS              CONFIG_AVDT_NUM_LINKS

/* Number of simultaneous stream endpoints. */
#define AVDT_NUM_SEPS               CONFIG_AVDT_NUM_SEPS

/******************************************************************************
 *
 * L2CAP
 *
 *****************************************************************************/

/* The maximum number of simultaneous links that ACL can support. */
#define MAX_ACL_CONNECTIONS         CONFIG_MAX_ACL_CONNECTIONS

#endif
