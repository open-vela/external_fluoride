/*
 * Copyright 2019 The Android Open Source Project
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

#define LOG_TAG "bt_shim_btm"

#include <base/callback.h>

#include <mutex>

#include "common/metric_id_allocator.h"
#include "common/time_util.h"
#include "device/include/controller.h"
#include "main/shim/btm_api.h"
#include "main/shim/controller.h"
#include "main/shim/shim.h"

namespace bluetooth {
  namespace shim {

    static Octet16 bogus_root;

    const Octet16& BTM_GetDeviceEncRoot() { return bogus_root; }
    const Octet16& BTM_GetDeviceIDRoot() { return bogus_root; }
    const Octet16& BTM_GetDeviceDHK() { return bogus_root; }

    void btm_api_process_inquiry_result(const RawAddress& raw_address, uint8_t page_scan_rep_mode, DEV_CLASS device_class, uint16_t clock_offset) { }
    void btm_api_process_inquiry_result_with_rssi(RawAddress raw_address, uint8_t page_scan_rep_mode, DEV_CLASS device_class, uint16_t clock_offset, int8_t rssi) { }
    void btm_api_process_extended_inquiry_result(RawAddress raw_address, uint8_t page_scan_rep_mode, DEV_CLASS device_class, uint16_t clock_offset, int8_t rssi, const uint8_t* eir_data, size_t eir_len) { }
    tBTM_STATUS BTM_StartInquiry(tBTM_INQ_RESULTS_CB* p_results_cb, tBTM_CMPL_CB* p_cmpl_cb) { return BTM_CMD_STARTED; }
    tBTM_STATUS BTM_SetDiscoverability(uint16_t discoverable_mode, uint16_t window, uint16_t interval) { return BTM_SUCCESS; }
    void BTM_EnableInterlacedInquiryScan() { }
    tBTM_STATUS BTM_BleObserve(bool start, uint8_t duration_sec, tBTM_INQ_RESULTS_CB* p_results_cb, tBTM_CMPL_CB* p_cmpl_cb) { return BTM_CMD_STARTED; }
    void BTM_EnableInterlacedPageScan() { }
    tBTM_STATUS BTM_SetInquiryMode(uint8_t inquiry_mode) { return BTM_SUCCESS; }
    tBTM_STATUS BTM_SetConnectability(uint16_t page_mode, uint16_t window, uint16_t interval) { return BTM_SUCCESS; }
    uint16_t BTM_IsInquiryActive(void) { return BTM_INQUIRY_INACTIVE; }
    void BTM_CancelInquiry(void) { }
    tBTM_STATUS BTM_ReadRemoteDeviceName( const RawAddress& raw_address, tBTM_CMPL_CB* callback, tBT_TRANSPORT transport) { return BTM_NO_RESOURCES; }
    tBTM_STATUS BTM_CancelRemoteDeviceName(void) { return BTM_SUCCESS; }
    tBTM_INQ_INFO* BTM_InqDbRead(const RawAddress& p_bda) { return nullptr; }
    tBTM_INQ_INFO* BTM_InqDbFirst(void) { return nullptr; }
    tBTM_INQ_INFO* BTM_InqDbNext(tBTM_INQ_INFO* p_cur) { return nullptr; }
    tBTM_STATUS BTM_ClearInqDb(const RawAddress* p_bda) { return BTM_NO_RESOURCES; }
    tBTM_STATUS BTM_WriteEIR(BT_HDR* p_buff) { return BTM_NO_RESOURCES; }
    bool BTM_HasEirService(const uint32_t* p_eir_uuid, uint16_t uuid16) { return false; }
    tBTM_EIR_SEARCH_RESULT BTM_HasInquiryEirService( tBTM_INQ_RESULTS* p_results, uint16_t uuid16) { return BTM_EIR_UNKNOWN; }
    void BTM_AddEirService(uint32_t* p_eir_uuid, uint16_t uuid16) { }
    void BTM_RemoveEirService(uint32_t* p_eir_uuid, uint16_t uuid16) { }
    uint8_t BTM_GetEirSupportedServices(uint32_t* p_eir_uuid, uint8_t** p, uint8_t max_num_uuid16, uint8_t* p_num_uuid16) { return BTM_NO_RESOURCES; }
    uint8_t BTM_GetEirUuidList(uint8_t* p_eir, size_t eir_len, uint8_t uuid_size, uint8_t* p_num_uuid, uint8_t* p_uuid_list, uint8_t max_num_uuid) { return 0; }
    bool BTM_SecAddBleDevice(const RawAddress& bd_addr, tBT_DEVICE_TYPE dev_type, tBLE_ADDR_TYPE addr_type) { return false; }
    bool BTM_SecAddBleKey(const RawAddress& bd_addr, tBTM_LE_KEY_VALUE* p_le_key, tBTM_LE_KEY_TYPE key_type) { return false; }
    void BTM_BleLoadLocalKeys(uint8_t key_type, tBTM_BLE_LOCAL_KEYS* p_key) { }

    void BTM_ReadConnectionAddr(const RawAddress& remote_bda, RawAddress& local_conn_addr, tBLE_ADDR_TYPE* p_addr_type) { }
    bool BTM_ReadRemoteConnectionAddr( const RawAddress& pseudo_addr, RawAddress& conn_addr, tBLE_ADDR_TYPE* p_addr_type) { return false; }
    void BTM_SecurityGrant(const RawAddress& bd_addr, uint8_t res) { }
    void BTM_BleOobDataReply(const RawAddress& bd_addr, uint8_t res, uint8_t len, uint8_t* p_data) { }
    void BTM_BleSecureConnectionOobDataReply( const RawAddress& bd_addr, uint8_t* p_c, uint8_t* p_r) { }
    void BTM_BleSetConnScanParams(uint32_t scan_interval, uint32_t scan_window) { }
    void BTM_BleSetPrefConnParams(const RawAddress& bd_addr, uint16_t min_conn_int, uint16_t max_conn_int, uint16_t slave_latency, uint16_t supervision_tout) { }
    void BTM_ReadDevInfo(const RawAddress& remote_bda, tBT_DEVICE_TYPE* p_dev_type, tBLE_ADDR_TYPE* p_addr_type) { }
    bool BTM_ReadConnectedTransportAddress( RawAddress* remote_bda, tBT_TRANSPORT transport) { return false; }
    void BTM_BleReceiverTest(uint8_t rx_freq, tBTM_CMPL_CB* p_cmd_cmpl_cback) { }
    void BTM_BleTransmitterTest(uint8_t tx_freq, uint8_t test_data_len, uint8_t packet_payload, tBTM_CMPL_CB* p_cmd_cmpl_cback) { }
    void BTM_BleTestEnd(tBTM_CMPL_CB* p_cmd_cmpl_cback) { }
    bool BTM_UseLeLink(const RawAddress& raw_address) { return false; }
    tBTM_STATUS BTM_SetBleDataLength(const RawAddress& bd_addr, uint16_t tx_pdu_length) { return BTM_NO_RESOURCES; }
    void BTM_BleReadPhy( const RawAddress& bd_addr, base::Callback<void(uint8_t tx_phy, uint8_t rx_phy, uint8_t status)> cb) { }
    void BTM_BleSetPhy(const RawAddress& bd_addr, uint8_t tx_phys, uint8_t rx_phys, uint16_t phy_options) { }
    bool BTM_BleDataSignature(const RawAddress& bd_addr, uint8_t* p_text, uint16_t len, BLE_SIGNATURE signature) { return false; }
    bool BTM_BleVerifySignature(const RawAddress& bd_addr, uint8_t* p_orig, uint16_t len, uint32_t counter, uint8_t* p_comp) { return false; }
    bool BTM_GetLeSecurityState(const RawAddress& bd_addr, uint8_t* p_le_dev_sec_flags, uint8_t* p_le_key_size) { return false; }
    bool BTM_BleSecurityProcedureIsRunning( const RawAddress& bd_addr) { return false; }
    uint8_t BTM_BleGetSupportedKeySize(const RawAddress& bd_addr) { return 0; }
    void BTM_LE_PF_local_name(tBTM_BLE_SCAN_COND_OP action, tBTM_BLE_PF_FILT_INDEX filt_index, std::vector<uint8_t> name, tBTM_BLE_PF_CFG_CBACK cb) { }
    void BTM_LE_PF_srvc_data(tBTM_BLE_SCAN_COND_OP action, tBTM_BLE_PF_FILT_INDEX filt_index) { }
    void BTM_LE_PF_manu_data( tBTM_BLE_SCAN_COND_OP action, tBTM_BLE_PF_FILT_INDEX filt_index, uint16_t company_id, uint16_t company_id_mask, std::vector<uint8_t> data, std::vector<uint8_t> data_mask, tBTM_BLE_PF_CFG_CBACK cb) { }
    void BTM_LE_PF_srvc_data_pattern( tBTM_BLE_SCAN_COND_OP action, tBTM_BLE_PF_FILT_INDEX filt_index, std::vector<uint8_t> data, std::vector<uint8_t> data_mask, tBTM_BLE_PF_CFG_CBACK cb) { }
    void BTM_LE_PF_addr_filter(tBTM_BLE_SCAN_COND_OP action, tBTM_BLE_PF_FILT_INDEX filt_index, tBLE_BD_ADDR addr, tBTM_BLE_PF_CFG_CBACK cb) { }
    void BTM_LE_PF_uuid_filter(tBTM_BLE_SCAN_COND_OP action, tBTM_BLE_PF_FILT_INDEX filt_index, tBTM_BLE_PF_COND_TYPE filter_type, const bluetooth::Uuid& uuid, tBTM_BLE_PF_LOGIC_TYPE cond_logic, const bluetooth::Uuid& uuid_mask, tBTM_BLE_PF_CFG_CBACK cb) { }
    void BTM_LE_PF_set(tBTM_BLE_PF_FILT_INDEX filt_index, std::vector<ApcfCommand> commands, tBTM_BLE_PF_CFG_CBACK cb) { }
    void BTM_LE_PF_clear(tBTM_BLE_PF_FILT_INDEX filt_index, tBTM_BLE_PF_CFG_CBACK cb) { }
    void BTM_BleAdvFilterParamSetup( int action, tBTM_BLE_PF_FILT_INDEX filt_index, std::unique_ptr<btgatt_filt_param_setup_t> p_filt_params, tBTM_BLE_PF_PARAM_CB cb) { }
    void BTM_BleUpdateAdvFilterPolicy(tBTM_BLE_AFP adv_policy) { }
    void BTM_BleEnableDisableFilterFeature( uint8_t enable, tBTM_BLE_PF_STATUS_CBACK p_stat_cback) { }
    uint8_t BTM_BleMaxMultiAdvInstanceCount() { return 0; }
    bool BTM_BleLocalPrivacyEnabled(void) { return controller_get_interface()->supports_ble_privacy(); }
    tBTM_STATUS BTM_SecBond(const RawAddress& bd_addr, tBLE_ADDR_TYPE addr_type, tBT_TRANSPORT transport, int device_type) { return BTM_SUCCESS; }
    bool BTM_SecRegister(const tBTM_APPL_INFO* bta_callbacks) { return true; }
    tBTM_STATUS BTM_SecBondCancel(const RawAddress& bd_addr) { return BTM_UNKNOWN_ADDR; }
    bool BTM_SecAddDevice(const RawAddress& bd_addr, DEV_CLASS dev_class, BD_NAME bd_name, uint8_t* features, LinkKey* link_key, uint8_t key_type, uint8_t pin_length) { return BTM_SUCCESS; }
    bool BTM_SecDeleteDevice(const RawAddress& bd_addr) { return false; }
    void BTM_ConfirmReqReply(tBTM_STATUS res, const RawAddress& bd_addr) { }
    uint16_t BTM_GetHCIConnHandle(const RawAddress& remote_bda, tBT_TRANSPORT transport) { return 0; }
    void SendRemoteNameRequest(const RawAddress& raw_address) { }
    tBTM_STATUS btm_sec_mx_access_request( const RawAddress& bd_addr, bool is_originator, uint16_t security_requirement, tBTM_SEC_CALLBACK* p_callback, void* p_ref_data) { return BTM_SUCCESS; }
    tBTM_STATUS BTM_SetEncryption(const RawAddress& bd_addr, tBT_TRANSPORT transport, tBTM_SEC_CALLBACK* p_callback, void* p_ref_data, tBTM_BLE_SEC_ACT sec_act) { return BTM_SUCCESS; }
    void BTM_SecClearSecurityFlags(const RawAddress& bd_addr) { }
    char* BTM_SecReadDevName(const RawAddress& address) { static char name[] = "TODO: See if this is needed"; return name; }
    bool BTM_SecAddRmtNameNotifyCallback( tBTM_RMT_NAME_CALLBACK* p_callback) { return true; }
    bool BTM_SecDeleteRmtNameNotifyCallback( tBTM_RMT_NAME_CALLBACK* p_callback) { return true; }
    void BTM_PINCodeReply(const RawAddress& bd_addr, uint8_t res, uint8_t pin_len, uint8_t* p_pin) { }
    void BTM_RemoteOobDataReply(tBTM_STATUS res, const RawAddress& bd_addr, const Octet16& c, const Octet16& r) { }
    tBTM_STATUS BTM_SetDeviceClass(DEV_CLASS dev_class) { return BTM_SUCCESS; }

  } // namespace shim
} // namespace bluetooth
