/*
 *  Copyright 2020 The Android Open Source Project
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
 */

#pragma once

#include <cstdint>

#include <cstdint>

#include "types/raw_address.h"

extern void btm_esco_proc_conn_chg(uint8_t status, uint16_t handle,
                                   uint8_t tx_interval, uint8_t retrans_window,
                                   uint16_t rx_pkt_len, uint16_t tx_pkt_len);
extern bool btm_is_sco_active(uint16_t handle);
extern void btm_sco_chk_pend_unpark(uint8_t hci_status, uint16_t hci_handle);
extern void btm_sco_conn_req(const RawAddress& bda, DEV_CLASS dev_class,
                             uint8_t link_type);
extern void btm_sco_connected(uint8_t hci_status, const RawAddress* bda,
                              uint16_t hci_handle, tBTM_ESCO_DATA* p_esco_data);
extern bool btm_sco_removed(uint16_t hci_handle, uint8_t reason);
