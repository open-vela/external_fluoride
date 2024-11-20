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

#include "stack/include/bt_types.h"

// This header contains functions for HCIF-Acl Management to invoke
//
void btm_acl_connection_request(const RawAddress& bda, uint8_t* dc);
void btm_acl_connected(const RawAddress& bda, uint16_t handle, uint8_t status,
                       uint8_t enc_mode);
void btm_acl_encrypt_change(uint16_t handle, uint8_t status,
                            uint8_t encr_enable);
void btm_acl_paging(BT_HDR* p, const RawAddress& dest);
void btm_acl_resubmit_page(void);
void btm_acl_role_changed(uint8_t hci_status, const RawAddress& bd_addr,
                          uint8_t new_role);
void btm_blacklist_role_change_device(const RawAddress& bd_addr,
                                      uint8_t hci_status);
void btm_pm_proc_cmd_status(uint8_t status);
void btm_pm_proc_mode_change(uint8_t hci_status, uint16_t hci_handle,
                             uint8_t mode, uint16_t interval);
void btm_pm_proc_ssr_evt(uint8_t* p, uint16_t evt_len);
void btm_process_clk_off_comp_evt(uint16_t hci_handle, uint16_t clock_offset);
void btm_read_automatic_flush_timeout_complete(uint8_t* p);
void btm_read_failed_contact_counter_complete(uint8_t* p);
void btm_read_link_quality_complete(uint8_t* p);
void btm_read_remote_ext_features_complete(uint8_t* p, uint8_t evt_len);
void btm_read_remote_ext_features_failed(uint8_t status, uint16_t handle);
void btm_read_remote_features_complete(uint8_t* p);
void btm_read_remote_version_complete(uint8_t* p);
void btm_read_rssi_complete(uint8_t* p);
void btm_read_tx_power_complete(uint8_t* p, bool is_ble);

void acl_rcv_acl_data(BT_HDR* p_msg);
void acl_link_segments_xmitted(BT_HDR* p_msg);
