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

#pragma once

#include <cstdint>

#include "device/include/esco_parameters.h"
#include "stack/include/btm_api_types.h"

/* Define the structures needed by sco
 */

typedef void(tBTM_SCO_IND_CBACK)(uint16_t sco_inx);

/* Define the structure that contains (e)SCO data */
typedef struct {
  tBTM_ESCO_CBACK* p_esco_cback; /* Callback for eSCO events     */
  enh_esco_params_t setup;
  tBTM_ESCO_DATA data; /* Connection complete information */
  uint8_t hci_status;
} tBTM_ESCO_INFO;

/* Define the structure used for SCO Management
 */
typedef struct {
  tBTM_ESCO_INFO esco;    /* Current settings             */
  tBTM_SCO_CB* p_conn_cb; /* Callback for when connected  */
  tBTM_SCO_CB* p_disc_cb; /* Callback for when disconnect */
  uint16_t state;         /* The state of the SCO link    */
  uint16_t hci_handle;    /* HCI Handle                   */
  bool is_orig;           /* true if the originator       */
  bool rem_bd_known;      /* true if remote BD addr known */

} tSCO_CONN;

/* SCO Management control block */
typedef struct {
  tBTM_SCO_IND_CBACK* app_sco_ind_cb;
  tSCO_CONN sco_db[BTM_MAX_SCO_LINKS];
  enh_esco_params_t def_esco_parms;
  uint16_t sco_disc_reason;
  bool esco_supported;        /* true if 1.2 cntlr AND supports eSCO links */
  esco_data_path_t sco_route; /* HCI, PCM, or TEST */
} tSCO_CB;

extern void btm_sco_chk_pend_rolechange(uint16_t hci_handle);
extern void btm_sco_disc_chk_pend_for_modechange(uint16_t hci_handle);
