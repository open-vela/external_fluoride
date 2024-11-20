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
#include "gd/l2cap/le/l2cap_le_module.h"
#include "gd/os/log.h"
#include "gd/os/queue.h"
#include "main/shim/btm.h"
#include "main/shim/entry.h"
#include "main/shim/helpers.h"
#include "main/shim/l2cap.h"
#include "osi/include/allocator.h"

static bluetooth::shim::legacy::L2cap shim_l2cap;

/**
 * Classic Service Registration APIs
 */
uint16_t bluetooth::shim::L2CA_Register(uint16_t client_psm,
                                        const tL2CAP_APPL_INFO& callbacks,
                                        bool enable_snoop,
                                        tL2CAP_ERTM_INFO* p_ertm_info,
                                        uint16_t required_mtu) {
  if (L2C_INVALID_PSM(client_psm)) {
    LOG_ERROR("%s Invalid classic psm:%hd", __func__, client_psm);
    return 0;
  }

  if ((callbacks.pL2CA_ConfigCfm_Cb == nullptr) ||
      (callbacks.pL2CA_ConfigInd_Cb == nullptr) ||
      (callbacks.pL2CA_DataInd_Cb == nullptr) ||
      (callbacks.pL2CA_DisconnectInd_Cb == nullptr)) {
    LOG_ERROR("%s Invalid classic callbacks psm:%hd", __func__, client_psm);
    return 0;
  }

  /**
   * Check if this is a registration for an outgoing-only connection.
   */
  const bool is_outgoing_connection_only =
      callbacks.pL2CA_ConnectInd_Cb == nullptr;
  const uint16_t psm = shim_l2cap.ConvertClientToRealPsm(
      client_psm, is_outgoing_connection_only);

  if (shim_l2cap.Classic().IsPsmRegistered(psm)) {
    LOG_ERROR("%s Already registered classic client_psm:%hd psm:%hd", __func__,
              client_psm, psm);
    return 0;
  }
  LOG_INFO("%s classic client_psm:%hd psm:%hd", __func__, client_psm, psm);

  return shim_l2cap.RegisterService(psm, callbacks, enable_snoop, p_ertm_info,
                                    required_mtu);
}

void bluetooth::shim::L2CA_Deregister(uint16_t client_psm) {
  if (L2C_INVALID_PSM(client_psm)) {
    LOG_ERROR("%s Invalid classic client_psm:%hd", __func__, client_psm);
    return;
  }
  uint16_t psm = shim_l2cap.ConvertClientToRealPsm(client_psm);

  shim_l2cap.UnregisterService(psm);
  shim_l2cap.RemoveClientPsm(psm);
}

uint16_t bluetooth::shim::L2CA_AllocatePSM(void) {
  return shim_l2cap.GetNextDynamicClassicPsm();
}

uint16_t bluetooth::shim::L2CA_AllocateLePSM(void) {
  return shim_l2cap.GetNextDynamicLePsm();
}

void bluetooth::shim::L2CA_FreeLePSM(uint16_t psm) {
  if (!shim_l2cap.Le().IsPsmRegistered(psm)) {
    LOG_ERROR("%s Not previously registered le psm:%hd", __func__, psm);
    return;
  }
  shim_l2cap.Le().UnregisterPsm(psm);
}

/**
 * Classic Connection Oriented Channel APIS
 */
uint16_t bluetooth::shim::L2CA_ErtmConnectReq(uint16_t psm,
                                              const RawAddress& raw_address,
                                              tL2CAP_ERTM_INFO* p_ertm_info) {
  return shim_l2cap.CreateConnection(psm, raw_address);
}

uint16_t bluetooth::shim::L2CA_ConnectReq(uint16_t psm,
                                          const RawAddress& raw_address) {
  return shim_l2cap.CreateConnection(psm, raw_address);
}

bool bluetooth::shim::L2CA_ErtmConnectRsp(const RawAddress& p_bd_addr,
                                          uint8_t id, uint16_t lcid,
                                          uint16_t result, uint16_t status,
                                          tL2CAP_ERTM_INFO* p_ertm_info) {
  return shim_l2cap.ConnectResponse(p_bd_addr, id, lcid, result, status,
                                    p_ertm_info);
}

bool bluetooth::shim::L2CA_ConnectRsp(const RawAddress& p_bd_addr, uint8_t id,
                                      uint16_t lcid, uint16_t result,
                                      uint16_t status) {
  return bluetooth::shim::L2CA_ErtmConnectRsp(p_bd_addr, id, lcid, result,
                                              status, nullptr);
}

bool bluetooth::shim::L2CA_ConfigReq(uint16_t cid, tL2CAP_CFG_INFO* cfg_info) {
  return shim_l2cap.ConfigRequest(cid, cfg_info);
}

bool bluetooth::shim::L2CA_ConfigRsp(uint16_t cid, tL2CAP_CFG_INFO* cfg_info) {
  return shim_l2cap.ConfigResponse(cid, cfg_info);
}

bool bluetooth::shim::L2CA_DisconnectReq(uint16_t cid) {
  return shim_l2cap.DisconnectRequest(cid);
}

bool bluetooth::shim::L2CA_DisconnectRsp(uint16_t cid) {
  return shim_l2cap.DisconnectResponse(cid);
}

/**
 * Le Connection Oriented Channel APIs
 */
uint16_t bluetooth::shim::L2CA_RegisterLECoc(
    uint16_t psm, const tL2CAP_APPL_INFO& callbacks) {
  LOG_INFO("UNIMPLEMENTED %s psm:%hd", __func__, psm);
  return 0;
}

void bluetooth::shim::L2CA_DeregisterLECoc(uint16_t psm) {
  LOG_INFO("UNIMPLEMENTED %s psm:%hd", __func__, psm);
}

uint16_t bluetooth::shim::L2CA_ConnectLECocReq(uint16_t psm,
                                               const RawAddress& p_bd_addr,
                                               tL2CAP_LE_CFG_INFO* p_cfg) {
  LOG_INFO("UNIMPLEMENTED %s psm:%hd addr:%s p_cfg:%p", __func__, psm,
           p_bd_addr.ToString().c_str(), p_cfg);
  return 0;
}

bool bluetooth::shim::L2CA_ConnectLECocRsp(const RawAddress& p_bd_addr,
                                           uint8_t id, uint16_t lcid,
                                           uint16_t result, uint16_t status,
                                           tL2CAP_LE_CFG_INFO* p_cfg) {
  LOG_INFO(
      "UNIMPLEMENTED %s addr:%s id:%hhd lcid:%hd result:%hd status:%hd "
      "p_cfg:%p",
      __func__, p_bd_addr.ToString().c_str(), id, lcid, result, status, p_cfg);
  return false;
}

bool bluetooth::shim::L2CA_GetPeerLECocConfig(uint16_t lcid,
                                              tL2CAP_LE_CFG_INFO* peer_cfg) {
  LOG_INFO("UNIMPLEMENTED %s lcid:%hd peer_cfg:%p", __func__, lcid, peer_cfg);
  return false;
}

uint8_t bluetooth::shim::L2CA_DataWrite(uint16_t cid, BT_HDR* p_data) {
  bool write_success = shim_l2cap.Write(cid, p_data);
  return write_success ? L2CAP_DW_SUCCESS : L2CAP_DW_FAILED;
}

/**
 * Link APIs
 */
bool bluetooth::shim::L2CA_SetIdleTimeoutByBdAddr(const RawAddress& bd_addr,
                                                  uint16_t timeout,
                                                  tBT_TRANSPORT transport) {
  LOG_INFO("UNIMPLEMENTED %s", __func__);
  return false;
}

bool bluetooth::shim::L2CA_SetAclPriority(const RawAddress& bd_addr,
                                          uint8_t priority) {
  LOG_INFO("UNIMPLEMENTED %s", __func__);
  return false;
}

bool bluetooth::shim::L2CA_SetFlushTimeout(const RawAddress& bd_addr,
                                           uint16_t flush_tout) {
  LOG_INFO("UNIMPLEMENTED %s", __func__);
  return false;
}

bool bluetooth::shim::L2CA_GetPeerFeatures(const RawAddress& bd_addr,
                                           uint32_t* p_ext_feat,
                                           uint8_t* p_chnl_mask) {
  LOG_INFO("UNIMPLEMENTED %s", __func__);
  return false;
}

using bluetooth::hci::AddressWithType;
using bluetooth::l2cap::le::FixedChannel;
using bluetooth::l2cap::le::FixedChannelManager;
using bluetooth::l2cap::le::FixedChannelService;

static constexpr uint16_t kAttCid = 4;

struct LeFixedChannelHelper {
  LeFixedChannelHelper(uint16_t cid) : cid_(cid) {}

  uint16_t cid_;

  void on_registration_complete(FixedChannelManager::RegistrationResult result,
                                std::unique_ptr<FixedChannelService> service) {
    if (result != FixedChannelManager::RegistrationResult::SUCCESS) {
      LOG(ERROR) << "Channel is not registered. cid=" << +cid_;
      return;
    }
    channel_service_ = std::move(service);
  }

  std::unique_ptr<FixedChannelService> channel_service_ = nullptr;

  void on_channel_close(bluetooth::hci::AddressWithType device,
                        bluetooth::hci::ErrorCode error_code) {
    auto address = bluetooth::ToRawAddress(device.GetAddress());
    channel_enqueue_buffer_[device] = nullptr;
    channels_[device]->GetQueueUpEnd()->UnregisterDequeue();
    channels_[device] = nullptr;
    (freg_.pL2CA_FixedConn_Cb)(cid_, address, true, 0, 2);
  }

  void on_channel_open(std::unique_ptr<FixedChannel> channel) {
    auto device = channel->GetDevice();
    channel->RegisterOnCloseCallback(
        bluetooth::shim::GetGdShimHandler(),
        bluetooth::common::BindOnce(&LeFixedChannelHelper::on_channel_close,
                                    bluetooth::common::Unretained(this),
                                    device));
    channel->Acquire();
    channel_enqueue_buffer_[device] = std::make_unique<
        bluetooth::os::EnqueueBuffer<bluetooth::packet::BasePacketBuilder>>(
        channel->GetQueueUpEnd());
    channel->GetQueueUpEnd()->RegisterDequeue(
        bluetooth::shim::GetGdShimHandler(),
        bluetooth::common::Bind(&LeFixedChannelHelper::on_incoming_data,
                                bluetooth::common::Unretained(this), device));
    channels_[device] = std::move(channel);

    auto address = bluetooth::ToRawAddress(device.GetAddress());

    (freg_.pL2CA_FixedConn_Cb)(cid_, address, true, 0, BT_TRANSPORT_LE);
    bluetooth::shim::Btm::StoreAddressType(
        address, static_cast<tBLE_ADDR_TYPE>(device.GetAddressType()));
  }

  void on_incoming_data(bluetooth::hci::AddressWithType remote) {
    auto channel = channels_.find(remote);
    if (channel == channels_.end()) {
      LOG_ERROR("Channel is not open");
      return;
    }
    auto packet = channel->second->GetQueueUpEnd()->TryDequeue();
    std::vector<uint8_t> packet_vector(packet->begin(), packet->end());
    BT_HDR* buffer =
        static_cast<BT_HDR*>(osi_calloc(packet_vector.size() + sizeof(BT_HDR)));
    std::copy(packet_vector.begin(), packet_vector.end(), buffer->data);
    buffer->len = packet_vector.size();
    auto address = bluetooth::ToRawAddress(remote.GetAddress());
    freg_.pL2CA_FixedData_Cb(cid_, address, buffer);
  }

  void on_outgoing_connection_fail(
      RawAddress remote, FixedChannelManager::ConnectionResult result) {
    LOG(ERROR) << "Outgoing connection failed";
    freg_.pL2CA_FixedConn_Cb(cid_, remote, true, 0, BT_TRANSPORT_LE);
  }

  bool send(AddressWithType remote,
            std::unique_ptr<bluetooth::packet::BasePacketBuilder> packet) {
    auto buffer = channel_enqueue_buffer_.find(remote);
    if (buffer == channel_enqueue_buffer_.end() || buffer->second == nullptr) {
      LOG(ERROR) << "Channel is not open";
      return false;
    }
    buffer->second->Enqueue(std::move(packet),
                            bluetooth::shim::GetGdShimHandler());
    return true;
  }

  std::unordered_map<AddressWithType, std::unique_ptr<FixedChannel>> channels_;
  std::unordered_map<AddressWithType,
                     std::unique_ptr<bluetooth::os::EnqueueBuffer<
                         bluetooth::packet::BasePacketBuilder>>>
      channel_enqueue_buffer_;
  tL2CAP_FIXED_CHNL_REG freg_;
};

static LeFixedChannelHelper att_helper{4};
static std::unordered_map<uint16_t, LeFixedChannelHelper&>
    le_fixed_channel_helper_{
        {4, att_helper},
    };

/**
 * Fixed Channel APIs. Note: Classic fixed channel (connectionless and BR SMP)
 * is not supported
 */
bool bluetooth::shim::L2CA_RegisterFixedChannel(uint16_t cid,
                                                tL2CAP_FIXED_CHNL_REG* p_freg) {
  if (cid != kAttCid) {
    LOG(ERROR) << "Invalid cid: " << cid;
    return false;
  }
  auto* helper = &le_fixed_channel_helper_.find(cid)->second;
  if (helper == nullptr) {
    LOG(ERROR) << "Can't register cid " << cid;
    return false;
  }
  bluetooth::shim::GetL2capLeModule()
      ->GetFixedChannelManager()
      ->RegisterService(
          cid,
          common::BindOnce(&LeFixedChannelHelper::on_registration_complete,
                           common::Unretained(helper)),
          common::Bind(&LeFixedChannelHelper::on_channel_open,
                       common::Unretained(helper)),
          GetGdShimHandler());
  helper->freg_ = *p_freg;
  return true;
}

bool bluetooth::shim::L2CA_ConnectFixedChnl(uint16_t cid,
                                            const RawAddress& rem_bda) {
  if (cid != kAttCid) {
    LOG(ERROR) << "Invalid cid " << cid;
    return false;
  }

  auto* helper = &le_fixed_channel_helper_.find(cid)->second;
  auto remote = ToAddressWithType(rem_bda, Btm::GetAddressType(rem_bda));
  auto manager = bluetooth::shim::GetL2capLeModule()->GetFixedChannelManager();
  manager->ConnectServices(
      remote,
      common::BindOnce(&LeFixedChannelHelper::on_outgoing_connection_fail,
                       common::Unretained(helper), rem_bda),
      GetGdShimHandler());
  return true;
}

bool bluetooth::shim::L2CA_ConnectFixedChnl(uint16_t cid,
                                            const RawAddress& rem_bda,
                                            uint8_t initiating_phys) {
  return bluetooth::shim::L2CA_ConnectFixedChnl(cid, rem_bda);
}

uint16_t bluetooth::shim::L2CA_SendFixedChnlData(uint16_t cid,
                                                 const RawAddress& rem_bda,
                                                 BT_HDR* p_buf) {
  if (cid != kAttCid) {
    LOG(ERROR) << "Invalid cid " << cid;
    return false;
  }
  auto* helper = &le_fixed_channel_helper_.find(cid)->second;
  auto remote = ToAddressWithType(rem_bda, Btm::GetAddressType(rem_bda));
  auto len = p_buf->len;
  auto* data = p_buf->data + p_buf->offset;
  bool sent = helper->send(remote, MakeUniquePacket(data, len));
  return sent ? len : 0;
}

bool bluetooth::shim::L2CA_RemoveFixedChnl(uint16_t cid,
                                           const RawAddress& rem_bda) {
  if (cid != kAttCid) {
    LOG(ERROR) << "Invalid cid " << cid;
    return false;
  }
  auto* helper = &le_fixed_channel_helper_.find(cid)->second;
  auto remote = ToAddressWithType(rem_bda, Btm::GetAddressType(rem_bda));
  auto channel = helper->channels_.find(remote);
  if (channel == helper->channels_.end() || channel->second == nullptr) {
    LOG(ERROR) << "Channel is not open";
    return false;
  }
  channel->second->Release();
  return true;
}

/**
 * Channel hygiene APIs
 */
bool bluetooth::shim::L2CA_GetRemoteCid(uint16_t lcid, uint16_t* rcid) {
  return shim_l2cap.GetRemoteCid(lcid, rcid);
}

bool bluetooth::shim::L2CA_SetTxPriority(uint16_t cid,
                                         tL2CAP_CHNL_PRIORITY priority) {
  LOG_INFO("UNIMPLEMENTED %s", __func__);
  return false;
}

bool bluetooth::shim::L2CA_SetFixedChannelTout(const RawAddress& rem_bda,
                                               uint16_t fixed_cid,
                                               uint16_t idle_tout) {
  LOG_INFO("UNIMPLEMENTED %s", __func__);
  return false;
}

bool bluetooth::shim::L2CA_SetChnlFlushability(uint16_t cid,
                                               bool is_flushable) {
  LOG_INFO("UNIMPLEMENTED %s", __func__);
  return false;
}

uint16_t bluetooth::shim::L2CA_FlushChannel(uint16_t lcid,
                                            uint16_t num_to_flush) {
  LOG_INFO("UNIMPLEMENTED %s", __func__);
  return 0;
}

bool bluetooth::shim::L2CA_IsLinkEstablished(const RawAddress& bd_addr,
                                             tBT_TRANSPORT transport) {
  LOG_INFO("UNIMPLEMENTED %s", __func__);
  return true;
}
