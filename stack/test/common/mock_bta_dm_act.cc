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

/*
 * Generated mock file from original source file
 *   Functions generated:53
 */

#include <map>
#include <string>

extern std::map<std::string, int> mock_function_count_map;

#include "bta/dm/bta_dm_int.h"
#include "types/raw_address.h"

#ifndef UNUSED_ATTR
#define UNUSED_ATTR
#endif

void BTA_dm_acl_down(const RawAddress bd_addr, tBT_TRANSPORT transport) {
  mock_function_count_map[__func__]++;
}
void BTA_dm_acl_up(const RawAddress bd_addr, tBT_TRANSPORT transport) {
  mock_function_count_map[__func__]++;
}
void BTA_dm_notify_remote_features_complete(const RawAddress bd_addr) {
  mock_function_count_map[__func__]++;
}

void BTA_dm_on_hw_off() { mock_function_count_map[__func__]++; }
void BTA_dm_on_hw_on() { mock_function_count_map[__func__]++; }
void BTA_dm_report_role_change(const RawAddress bd_addr, uint8_t new_role,
                               uint8_t hci_status) {
  mock_function_count_map[__func__]++;
}
void bta_dm_add_device(std::unique_ptr<tBTA_DM_API_ADD_DEVICE> msg) {
  mock_function_count_map[__func__]++;
}
void bta_dm_bond(const RawAddress& bd_addr, tBLE_ADDR_TYPE addr_type,
                 tBT_TRANSPORT transport, int device_type) {
  mock_function_count_map[__func__]++;
}
void bta_dm_bond_cancel(const RawAddress& bd_addr) {
  mock_function_count_map[__func__]++;
}
void bta_dm_ci_rmt_oob_act(std::unique_ptr<tBTA_DM_CI_RMT_OOB> msg) {
  mock_function_count_map[__func__]++;
}
void bta_dm_close_acl(const RawAddress& bd_addr, bool remove_dev,
                      tBT_TRANSPORT transport) {
  mock_function_count_map[__func__]++;
}
void bta_dm_confirm(const RawAddress& bd_addr, bool accept) {
  mock_function_count_map[__func__]++;
}
void bta_dm_deinit_cb(void) { mock_function_count_map[__func__]++; }
void bta_dm_disable() { mock_function_count_map[__func__]++; }
void bta_dm_disc_result(tBTA_DM_MSG* p_data) {
  mock_function_count_map[__func__]++;
}
void bta_dm_disc_rmt_name(tBTA_DM_MSG* p_data) {
  mock_function_count_map[__func__]++;
}
void bta_dm_discover(tBTA_DM_MSG* p_data) {
  mock_function_count_map[__func__]++;
}
void bta_dm_free_sdp_db() { mock_function_count_map[__func__]++; }
void bta_dm_init_cb(void) { mock_function_count_map[__func__]++; }
void bta_dm_inq_cmpl(uint8_t num) { mock_function_count_map[__func__]++; }
void bta_dm_pin_reply(std::unique_ptr<tBTA_DM_API_PIN_REPLY> msg) {
  mock_function_count_map[__func__]++;
}
void bta_dm_process_remove_device(const RawAddress& bd_addr) {
  mock_function_count_map[__func__]++;
}
void bta_dm_queue_disc(tBTA_DM_MSG* p_data) {
  mock_function_count_map[__func__]++;
}
void bta_dm_queue_search(tBTA_DM_MSG* p_data) {
  mock_function_count_map[__func__]++;
}
void bta_dm_remove_device(const RawAddress& bd_addr) {
  mock_function_count_map[__func__]++;
}
void bta_dm_rmt_name(tBTA_DM_MSG* p_data) {
  mock_function_count_map[__func__]++;
}
void bta_dm_sdp_result(tBTA_DM_MSG* p_data) {
  mock_function_count_map[__func__]++;
}
void bta_dm_search_cancel() { mock_function_count_map[__func__]++; }
void bta_dm_search_cancel_cmpl() { mock_function_count_map[__func__]++; }
void bta_dm_search_cancel_notify() { mock_function_count_map[__func__]++; }
void bta_dm_search_clear_queue() { mock_function_count_map[__func__]++; }
void bta_dm_search_cmpl() { mock_function_count_map[__func__]++; }
void bta_dm_search_result(tBTA_DM_MSG* p_data) {
  mock_function_count_map[__func__]++;
}
void bta_dm_search_start(tBTA_DM_MSG* p_data) {
  mock_function_count_map[__func__]++;
}
void bta_dm_set_dev_name(const std::vector<uint8_t>& name) {
  mock_function_count_map[__func__]++;
}
