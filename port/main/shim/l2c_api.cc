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

#define LOG_TAG "bt_shim_l2cap"

#include "main/shim/l2c_api.h"
#include "osi/include/allocator.h"

namespace bluetooth {
  namespace shim {

    bool L2CA_ReadRemoteVersion(const RawAddress& addr, uint8_t* lmp_version, uint16_t* manufacturer, uint16_t* lmp_sub_version) { return false; }
    uint16_t L2CA_Register(uint16_t client_psm, const tL2CAP_APPL_INFO& callbacks, bool enable_snoop, tL2CAP_ERTM_INFO* p_ertm_info, uint16_t my_mtu, uint16_t required_remote_mtu, uint16_t sec_level) { return 0; }
    void L2CA_Deregister(uint16_t psm) { }
    uint16_t L2CA_ConnectReq(uint16_t psm, const RawAddress& raw_address) { return 0; }
    bool L2CA_DisconnectReq(uint16_t cid) { return false; }
    uint8_t L2CA_DataWrite(uint16_t cid, BT_HDR* p_data) { return 0; }
    bool L2CA_ReconfigCreditBasedConnsReq(const RawAddress& bd_addr, std::vector<uint16_t>& lcids, tL2CAP_LE_CFG_INFO* p_cfg) { return false; }
    std::vector<uint16_t> L2CA_ConnectCreditBasedReq(uint16_t psm, const RawAddress& p_bd_addr, tL2CAP_LE_CFG_INFO* p_cfg) { std::vector<uint16_t> result; return result; }
    bool L2CA_ConnectCreditBasedRsp(const RawAddress& bd_addr, uint8_t id, std::vector<uint16_t>& accepted_lcids, uint16_t result, tL2CAP_LE_CFG_INFO* p_cfg) { return false; }
    bool L2CA_SetIdleTimeoutByBdAddr(const RawAddress& bd_addr, uint16_t timeout, tBT_TRANSPORT transport) { return false; }
    bool L2CA_SetAclPriority(const RawAddress& bd_addr, tL2CAP_PRIORITY priority) { return false; }
    bool L2CA_GetPeerFeatures(const RawAddress& bd_addr, uint32_t* p_ext_feat, uint8_t* p_chnl_mask) { return false; }
    bool L2CA_RegisterFixedChannel(uint16_t cid, tL2CAP_FIXED_CHNL_REG* p_freg) { return false; }
    bool L2CA_ConnectFixedChnl(uint16_t cid, const RawAddress& rem_bda) { return false; }
    uint16_t L2CA_SendFixedChnlData(uint16_t cid, const RawAddress& rem_bda, BT_HDR* p_buf) { return L2CAP_DW_FAILED; }
    bool L2CA_RemoveFixedChnl(uint16_t cid, const RawAddress& rem_bda) { return false; }
    uint16_t L2CA_GetLeHandle(uint16_t cid, const RawAddress& rem_bda) { return 0; }
    void L2CA_LeConnectionUpdate(const RawAddress& rem_bda) { }
    bool L2CA_GetRemoteCid(uint16_t lcid, uint16_t* rcid) { return false; }
    bool L2CA_SetTxPriority(uint16_t cid, tL2CAP_CHNL_PRIORITY priority) { return false; }
    bool L2CA_SetFixedChannelTout(const RawAddress& rem_bda, uint16_t fixed_cid, uint16_t idle_tout) { return false; }
    bool L2CA_SetChnlFlushability(uint16_t cid, bool is_flushable) { return false; }
    uint16_t L2CA_FlushChannel(uint16_t lcid, uint16_t num_to_flush) { return 0; }
    bool L2CA_IsLinkEstablished(const RawAddress& bd_addr, tBT_TRANSPORT transport) { return false; }
    void L2CA_ConnectForSecurity(const RawAddress& bd_addr) { }
    void L2CA_SetBondingState(const RawAddress& bd_addr, bool is_bonding) { }
    void L2CA_DisconnectLink(const RawAddress& remote) { }
    uint16_t L2CA_GetNumLinks() { return 0; }
    uint16_t L2CA_AllocateLePSM() { return 0; }
    void L2CA_FreeLePSM(uint16_t psm) {}
    uint16_t L2CA_RegisterLECoc(uint16_t psm, const tL2CAP_APPL_INFO& callbacks, uint16_t sec_level, tL2CAP_LE_CFG_INFO cfg) { return 0; }
    void L2CA_DeregisterLECoc(uint16_t psm) { }
    uint16_t L2CA_ConnectLECocReq(uint16_t psm, const RawAddress& p_bd_addr, tL2CAP_LE_CFG_INFO* p_cfg) { return 0; }
    bool L2CA_GetPeerLECocConfig(uint16_t cid, tL2CAP_LE_CFG_INFO* peer_cfg) { return false; }
    bool L2CA_DisconnectLECocReq(uint16_t cid) { return false; }
    uint8_t L2CA_LECocDataWrite(uint16_t cid, BT_HDR* p_data) { return 0; }
    void L2CA_SwitchRoleToCentral(const RawAddress& addr) { }

  } // namespace shim
} // namespace bluetooth
