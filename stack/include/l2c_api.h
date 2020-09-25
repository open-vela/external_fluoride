/******************************************************************************
 *
 *  Copyright 1999-2012 Broadcom Corporation
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
 *  this file contains the L2CAP API definitions
 *
 ******************************************************************************/
#ifndef L2C_API_H
#define L2C_API_H

#include <stdbool.h>

#include "bt_target.h"
#include "hcidefs.h"
#include "l2cdefs.h"
#include "types/bt_transport.h"

/*****************************************************************************
 *  Constants
 ****************************************************************************/

/* Define the minimum offset that L2CAP needs in a buffer. This is made up of
 * HCI type(1), len(2), handle(2), L2CAP len(2) and CID(2) => 9
 */
#define L2CAP_MIN_OFFSET 13 /* plus control(2), SDU length(2) */

#define L2CAP_LCC_SDU_LENGTH 2
#define L2CAP_LCC_OFFSET \
  (L2CAP_MIN_OFFSET + L2CAP_LCC_SDU_LENGTH) /* plus SDU length(2) */

#define L2CAP_FCS_LENGTH 2

/* result code for L2CA_DataWrite() */
#define L2CAP_DW_FAILED false
#define L2CAP_DW_SUCCESS true
#define L2CAP_DW_CONGESTED 2

/* Values for priority parameter to L2CA_SetAclPriority */
#define L2CAP_PRIORITY_NORMAL 0
#define L2CAP_PRIORITY_HIGH 1

/* Values for priority parameter to L2CA_SetTxPriority */
#define L2CAP_CHNL_PRIORITY_HIGH 0
#define L2CAP_CHNL_PRIORITY_LOW 2

typedef uint8_t tL2CAP_CHNL_PRIORITY;

/* Values for Tx/Rx data rate parameter to L2CA_SetChnlDataRate */
#define L2CAP_CHNL_DATA_RATE_LOW 1

typedef uint8_t tL2CAP_CHNL_DATA_RATE;

/* Data Packet Flags  (bits 2-15 are reserved) */
/* layer specific 14-15 bits are used for FCR SAR */
#define L2CAP_FLUSHABLE_MASK 0x0003
#define L2CAP_FLUSHABLE_CH_BASED 0x0000
#define L2CAP_FLUSHABLE_PKT 0x0001
#define L2CAP_NON_FLUSHABLE_PKT 0x0002

/* L2CA_FlushChannel num_to_flush definitions */
#define L2CAP_FLUSH_CHANS_ALL 0xffff
#define L2CAP_FLUSH_CHANS_GET 0x0000

/* Values for 'allowed_modes' field passed in structure tL2CAP_ERTM_INFO
 */
#define L2CAP_FCR_CHAN_OPT_BASIC (1 << L2CAP_FCR_BASIC_MODE)
#define L2CAP_FCR_CHAN_OPT_ERTM (1 << L2CAP_FCR_ERTM_MODE)

#define L2CAP_FCR_CHAN_OPT_ALL_MASK \
  (L2CAP_FCR_CHAN_OPT_BASIC | L2CAP_FCR_CHAN_OPT_ERTM)

/* Validity check for PSM.  PSM values must be odd.  Also, all PSM values must
 * be assigned such that the least significant bit of the most sigificant
 * octet equals zero.
 */
#define L2C_INVALID_PSM(psm) (((psm)&0x0101) != 0x0001)
#define L2C_IS_VALID_PSM(psm) (((psm)&0x0101) == 0x0001)
#define L2C_IS_VALID_LE_PSM(psm) (((psm) > 0x0000) && ((psm) < 0x0100))

/*****************************************************************************
 *  Type Definitions
 ****************************************************************************/

typedef struct {
#define L2CAP_FCR_BASIC_MODE 0x00
#define L2CAP_FCR_ERTM_MODE 0x03
#define L2CAP_FCR_STREAM_MODE 0x04
#define L2CAP_FCR_LE_COC_MODE 0x05

  uint8_t mode;

  uint8_t tx_win_sz;
  uint8_t max_transmit;
  uint16_t rtrans_tout;
  uint16_t mon_tout;
  uint16_t mps;
} tL2CAP_FCR_OPTS;

/* Define a structure to hold the configuration parameters. Since the
 * parameters are optional, for each parameter there is a boolean to
 * use to signify its presence or absence.
 */
typedef struct {
  uint16_t result; /* Only used in confirm messages */
  bool mtu_present;
  uint16_t mtu;
  bool qos_present;
  FLOW_SPEC qos;
  bool flush_to_present;
  uint16_t flush_to;
  bool fcr_present;
  tL2CAP_FCR_OPTS fcr;
  bool fcs_present; /* Optionally bypasses FCS checks */
  uint8_t fcs;      /* '0' if desire is to bypass FCS, otherwise '1' */
  bool ext_flow_spec_present;
  tHCI_EXT_FLOW_SPEC ext_flow_spec;
  uint16_t flags; /* bit 0: 0-no continuation, 1-continuation */
} tL2CAP_CFG_INFO;

/* LE credit based L2CAP connection parameters */
constexpr uint16_t L2CAP_LE_MIN_MTU = 23;  // Minimum SDU size
constexpr uint16_t L2CAP_LE_MIN_MPS = 23;
constexpr uint16_t L2CAP_LE_MAX_MPS = 65533;
constexpr uint16_t L2CAP_LE_CREDIT_MAX = 65535;

// This is initial amout of credits we send, and amount to which we increase
// credits once they fall below threshold
constexpr uint16_t L2CAP_LE_CREDIT_DEFAULT = 0xffff;

// If credit count on remote fall below this value, we send back credits to
// reach default value.
constexpr uint16_t L2CAP_LE_CREDIT_THRESHOLD = 0x0040;

static_assert(L2CAP_LE_CREDIT_THRESHOLD < L2CAP_LE_CREDIT_DEFAULT,
              "Threshold must be smaller than default credits");

/* Define a structure to hold the configuration parameter for LE L2CAP
 * connection oriented channels.
 */
struct tL2CAP_LE_CFG_INFO {
  uint16_t mtu;
  uint16_t mps;
  uint16_t credits = L2CAP_LE_CREDIT_DEFAULT;
};

/* L2CAP channel configured field bitmap */
#define L2CAP_CH_CFG_MASK_MTU 0x0001
#define L2CAP_CH_CFG_MASK_QOS 0x0002
#define L2CAP_CH_CFG_MASK_FLUSH_TO 0x0004
#define L2CAP_CH_CFG_MASK_FCR 0x0008
#define L2CAP_CH_CFG_MASK_FCS 0x0010

typedef uint16_t tL2CAP_CH_CFG_BITS;

/*********************************
 *  Callback Functions Prototypes
 *********************************/

/* Connection indication callback prototype. Parameters are
 *              BD Address of remote
 *              Local CID assigned to the connection
 *              PSM that the remote wants to connect to
 *              Identifier that the remote sent
 */
typedef void(tL2CA_CONNECT_IND_CB)(const RawAddress&, uint16_t, uint16_t,
                                   uint8_t);

/* Connection confirmation callback prototype. Parameters are
 *              Local CID
 *              Result - 0 = connected, non-zero means failure reason
 */
typedef void(tL2CA_CONNECT_CFM_CB)(uint16_t, uint16_t);

/* Configuration indication callback prototype. Parameters are
 *              Local CID assigned to the connection
 *              Pointer to configuration info
 */
typedef void(tL2CA_CONFIG_IND_CB)(uint16_t, tL2CAP_CFG_INFO*);

/* Configuration confirm callback prototype. Parameters are
 *              Local CID assigned to the connection
 *              Pointer to configuration info
 */
typedef void(tL2CA_CONFIG_CFM_CB)(uint16_t, tL2CAP_CFG_INFO*);

/* Disconnect indication callback prototype. Parameters are
 *              Local CID
 *              Boolean whether upper layer should ack this
 */
typedef void(tL2CA_DISCONNECT_IND_CB)(uint16_t, bool);

/* Disconnect confirm callback prototype. Parameters are
 *              Local CID
 *              Result
 */
typedef void(tL2CA_DISCONNECT_CFM_CB)(uint16_t, uint16_t);

/* Data received indication callback prototype. Parameters are
 *              Local CID
 *              Address of buffer
 */
typedef void(tL2CA_DATA_IND_CB)(uint16_t, BT_HDR*);

/* Callback function prototype to pass broadcom specific echo response  */
/* to the upper layer                                                   */
typedef void(tL2CA_ECHO_DATA_CB)(const RawAddress&, uint16_t, uint8_t*);

/* Congestion status callback protype. This callback is optional. If
 * an application tries to send data when the transmit queue is full,
 * the data will anyways be dropped. The parameter is:
 *              Local CID
 *              true if congested, false if uncongested
 */
typedef void(tL2CA_CONGESTION_STATUS_CB)(uint16_t, bool);

/* Transmit complete callback protype. This callback is optional. If
 * set, L2CAP will call it when packets are sent or flushed. If the
 * count is 0xFFFF, it means all packets are sent for that CID (eRTM
 * mode only). The parameters are:
 *              Local CID
 *              Number of SDUs sent or dropped
 */
typedef void(tL2CA_TX_COMPLETE_CB)(uint16_t, uint16_t);

/* Define the structure that applications use to register with
 * L2CAP. This structure includes callback functions. All functions
 * MUST be provided, with the exception of the "connect pending"
 * callback and "congestion status" callback.
 */
typedef struct {
  tL2CA_CONNECT_IND_CB* pL2CA_ConnectInd_Cb;
  tL2CA_CONNECT_CFM_CB* pL2CA_ConnectCfm_Cb;
  tL2CA_CONFIG_IND_CB* pL2CA_ConfigInd_Cb;
  tL2CA_CONFIG_CFM_CB* pL2CA_ConfigCfm_Cb;
  tL2CA_DISCONNECT_IND_CB* pL2CA_DisconnectInd_Cb;
  tL2CA_DATA_IND_CB* pL2CA_DataInd_Cb;
  tL2CA_CONGESTION_STATUS_CB* pL2CA_CongestionStatus_Cb;
  tL2CA_TX_COMPLETE_CB* pL2CA_TxComplete_Cb;
} tL2CAP_APPL_INFO;

/* Define the structure that applications use to create or accept
 * connections with enhanced retransmission mode.
 */
typedef struct {
  uint8_t preferred_mode;
  uint16_t user_rx_buf_size;
  uint16_t user_tx_buf_size;
  uint16_t fcr_rx_buf_size;
  uint16_t fcr_tx_buf_size;
} tL2CAP_ERTM_INFO;

/**
 * Stack management declarations
 */
void l2c_init();
void l2c_free();

/*****************************************************************************
 *  External Function Declarations
 ****************************************************************************/

// Also does security for you
uint16_t L2CA_Register2(uint16_t psm, const tL2CAP_APPL_INFO& p_cb_info,
                        bool enable_snoop, tL2CAP_ERTM_INFO* p_ertm_info,
                        uint16_t my_mtu, uint16_t required_remote_mtu,
                        uint16_t sec_level);

/*******************************************************************************
 *
 * Function         L2CA_Register
 *
 * Description      Other layers call this function to register for L2CAP
 *                  services.
 *
 * Returns          PSM to use or zero if error. Typically, the PSM returned
 *                  is the same as was passed in, but for an outgoing-only
 *                  connection to a dynamic PSM, a "virtual" PSM is returned
 *                  and should be used in the calls to L2CA_ConnectReq() and
 *                  BTM_SetSecurityLevel().
 *
 ******************************************************************************/
extern uint16_t L2CA_Register(uint16_t psm, const tL2CAP_APPL_INFO& p_cb_info,
                              bool enable_snoop, tL2CAP_ERTM_INFO* p_ertm_info,
                              uint16_t my_mtu, uint16_t required_remote_mtu);

/*******************************************************************************
 *
 * Function         L2CA_Deregister
 *
 * Description      Other layers call this function to deregister for L2CAP
 *                  services.
 *
 * Returns          void
 *
 ******************************************************************************/
extern void L2CA_Deregister(uint16_t psm);

/*******************************************************************************
 *
 * Function         L2CA_AllocatePSM
 *
 * Description      Other layers call this function to find an unused PSM for
 *                  L2CAP services.
 *
 * Returns          PSM to use.
 *
 ******************************************************************************/
extern uint16_t L2CA_AllocatePSM(void);

/*******************************************************************************
 *
 * Function         L2CA_AllocateLePSM
 *
 * Description      Other layers call this function to find an unused LE PSM for
 *                  L2CAP services.
 *
 * Returns          LE_PSM to use if success. Otherwise returns 0.
 *
 ******************************************************************************/
extern uint16_t L2CA_AllocateLePSM(void);

/*******************************************************************************
 *
 * Function         L2CA_FreeLePSM
 *
 * Description      Free an assigned LE PSM.
 *
 * Returns          void
 *
 ******************************************************************************/
extern void L2CA_FreeLePSM(uint16_t psm);

extern uint16_t L2CA_ConnectReq2(uint16_t psm, const RawAddress& p_bd_addr,
                                 uint16_t sec_level);
/*******************************************************************************
 *
 * Function         L2CA_ConnectReq
 *
 * Description      Higher layers call this function to create an L2CAP
 *                  connection.
 *                  Note that the connection is not established at this time,
 *                  but connection establishment gets started. The callback
 *                  will be invoked when connection establishes or fails.
 *
 * Returns          the CID of the connection, or 0 if it failed to start
 *
 ******************************************************************************/
extern uint16_t L2CA_ConnectReq(uint16_t psm, const RawAddress& p_bd_addr);

/*******************************************************************************
 *
 * Function         L2CA_ConnectRsp
 *
 * Description      Higher layers call this function to accept an incoming
 *                  L2CAP connection, for which they had gotten an connect
 *                  indication callback.
 *
 * Returns          true for success, false for failure
 *
 ******************************************************************************/
extern bool L2CA_ConnectRsp(const RawAddress& p_bd_addr, uint8_t id,
                            uint16_t lcid, uint16_t result, uint16_t status);

uint16_t L2CA_ErtmConnectReq2(uint16_t psm, const RawAddress& p_bd_addr,
                              tL2CAP_ERTM_INFO* p_ertm_info,
                              uint16_t sec_level);

/*******************************************************************************
 *
 * Function         L2CA_ErtmConnectReq
 *
 * Description      Higher layers call this function to create an L2CAP
 *                  connection that needs to use Enhanced Retransmission Mode.
 *                  Note that the connection is not established at this time,
 *                  but connection establishment gets started. The callback
 *                  will be invoked when connection establishes or fails.
 *
 * Returns          the CID of the connection, or 0 if it failed to start
 *
 ******************************************************************************/
extern uint16_t L2CA_ErtmConnectReq(uint16_t psm, const RawAddress& p_bd_addr,
                                    tL2CAP_ERTM_INFO* p_ertm_info);

/*******************************************************************************
 *
 * Function         L2CA_RegisterLECoc
 *
 * Description      Other layers call this function to register for L2CAP
 *                  Connection Oriented Channel.
 *
 * Returns          PSM to use or zero if error. Typically, the PSM returned
 *                  is the same as was passed in, but for an outgoing-only
 *                  connection to a dynamic PSM, a "virtual" PSM is returned
 *                  and should be used in the calls to L2CA_ConnectLECocReq()
 *                  and BTM_SetSecurityLevel().
 *
 ******************************************************************************/
extern uint16_t L2CA_RegisterLECoc(uint16_t psm,
                                   const tL2CAP_APPL_INFO& p_cb_info,
                                   uint16_t sec_level);

/*******************************************************************************
 *
 * Function         L2CA_DeregisterLECoc
 *
 * Description      Other layers call this function to deregister for L2CAP
 *                  Connection Oriented Channel.
 *
 * Returns          void
 *
 ******************************************************************************/
extern void L2CA_DeregisterLECoc(uint16_t psm);

/*******************************************************************************
 *
 * Function         L2CA_ConnectLECocReq
 *
 * Description      Higher layers call this function to create an L2CAP LE COC.
 *                  Note that the connection is not established at this time,
 *                  but connection establishment gets started. The callback
 *                  will be invoked when connection establishes or fails.
 *
 * Returns          the CID of the connection, or 0 if it failed to start
 *
 ******************************************************************************/
extern uint16_t L2CA_ConnectLECocReq(uint16_t psm, const RawAddress& p_bd_addr,
                                     tL2CAP_LE_CFG_INFO* p_cfg,
                                     uint16_t sec_level);

/*******************************************************************************
 *
 * Function         L2CA_ConnectLECocRsp
 *
 * Description      Higher layers call this function to accept an incoming
 *                  L2CAP LE COC connection, for which they had gotten a connect
 *                  indication callback.
 *
 * Returns          true for success, false for failure
 *
 ******************************************************************************/
extern bool L2CA_ConnectLECocRsp(const RawAddress& p_bd_addr, uint8_t id,
                                 uint16_t lcid, uint16_t result,
                                 uint16_t status, tL2CAP_LE_CFG_INFO* p_cfg);

/*******************************************************************************
 *
 *  Function         L2CA_GetPeerLECocConfig
 *
 *  Description      Get peers configuration for LE Connection Oriented Channel.
 *
 *  Return value:    true if peer is connected
 *
 ******************************************************************************/
extern bool L2CA_GetPeerLECocConfig(uint16_t lcid,
                                    tL2CAP_LE_CFG_INFO* peer_cfg);

/*******************************************************************************
 *
 * Function         L2CA_ErtmConnectRsp
 *
 * Description      Higher layers call this function to accept an incoming
 *                  L2CAP connection, for which they had gotten an connect
 *                  indication callback, and for which the higher layer wants
 *                  to use Enhanced Retransmission Mode.
 *
 * Returns          true for success, false for failure
 *
 ******************************************************************************/
extern bool L2CA_ErtmConnectRsp(const RawAddress& p_bd_addr, uint8_t id,
                                uint16_t lcid, uint16_t result, uint16_t status,
                                tL2CAP_ERTM_INFO* p_ertm_info);

/*******************************************************************************
 *
 * Function         L2CA_ConfigReq
 *
 * Description      Higher layers call this function to send configuration.
 *
 * Returns          true if configuration sent, else false
 *
 ******************************************************************************/
extern bool L2CA_ConfigReq(uint16_t cid, tL2CAP_CFG_INFO* p_cfg);

/*******************************************************************************
 *
 * Function         L2CA_ConfigRsp
 *
 * Description      Higher layers call this function to send a configuration
 *                  response.
 *
 * Returns          true if configuration response sent, else false
 *
 ******************************************************************************/
extern bool L2CA_ConfigRsp(uint16_t cid, tL2CAP_CFG_INFO* p_cfg);

/*******************************************************************************
 *
 * Function         L2CA_DisconnectReq
 *
 * Description      Higher layers call this function to disconnect a channel.
 *
 * Returns          true if disconnect sent, else false
 *
 ******************************************************************************/
extern bool L2CA_DisconnectReq(uint16_t cid);

/*******************************************************************************
 *
 * Function         L2CA_DataWrite
 *
 * Description      Higher layers call this function to write data.
 *
 * Returns          L2CAP_DW_SUCCESS, if data accepted, else false
 *                  L2CAP_DW_CONGESTED, if data accepted and the channel is
 *                                      congested
 *                  L2CAP_DW_FAILED, if error
 *
 ******************************************************************************/
extern uint8_t L2CA_DataWrite(uint16_t cid, BT_HDR* p_data);

// Given a local channel identifier, |lcid|, this function returns the bound
// remote channel identifier, |rcid|. If
// |lcid| is not known or is invalid, this function returns false and does not
// modify the value pointed at by |rcid|. |rcid| may be NULL.
bool L2CA_GetRemoteCid(uint16_t lcid, uint16_t* rcid);

/*******************************************************************************
 *
 * Function         L2CA_SetIdleTimeoutByBdAddr
 *
 * Description      Higher layers call this function to set the idle timeout for
 *                  a connection. The "idle timeout" is the amount of time that
 *                  a connection can remain up with no L2CAP channels on it.
 *                  A timeout of zero means that the connection will be torn
 *                  down immediately when the last channel is removed.
 *                  A timeout of 0xFFFF means no timeout. Values are in seconds.
 *                  A bd_addr is the remote BD address. If bd_addr =
 *                  RawAddress::kAny, then the idle timeouts for all active
 *                  l2cap links will be changed.
 *
 * Returns          true if command succeeded, false if failed
 *
 * NOTE             This timeout applies to all logical channels active on the
 *                  ACL link.
 ******************************************************************************/
extern bool L2CA_SetIdleTimeoutByBdAddr(const RawAddress& bd_addr,
                                        uint16_t timeout,
                                        tBT_TRANSPORT transport);

/*******************************************************************************
 *
 * Function         L2CA_SetTraceLevel
 *
 * Description      This function sets the trace level for L2CAP. If called with
 *                  a value of 0xFF, it simply reads the current trace level.
 *
 * Returns          the new (current) trace level
 *
 ******************************************************************************/
extern uint8_t L2CA_SetTraceLevel(uint8_t trace_level);

/*******************************************************************************
 *
 * Function     L2CA_FlushChannel
 *
 * Description  This function flushes none, some or all buffers queued up
 *              for xmission for a particular CID. If called with
 *              L2CAP_FLUSH_CHANS_GET (0), it simply returns the number
 *              of buffers queued for that CID L2CAP_FLUSH_CHANS_ALL (0xffff)
 *              flushes all buffers.  All other values specifies the maximum
 *              buffers to flush.
 *
 * Returns      Number of buffers left queued for that CID
 *
 ******************************************************************************/
extern uint16_t L2CA_FlushChannel(uint16_t lcid, uint16_t num_to_flush);

/*******************************************************************************
 *
 * Function         L2CA_SetAclPriority
 *
 * Description      Sets the transmission priority for an ACL channel.
 *                  (For initial implementation only two values are valid.
 *                  L2CAP_PRIORITY_NORMAL and L2CAP_PRIORITY_HIGH).
 *
 * Returns          true if a valid channel, else false
 *
 ******************************************************************************/
extern bool L2CA_SetAclPriority(const RawAddress& bd_addr, uint8_t priority);

/*******************************************************************************
 *
 * Function         L2CA_SetTxPriority
 *
 * Description      Sets the transmission priority for a channel. (FCR Mode)
 *
 * Returns          true if a valid channel, else false
 *
 ******************************************************************************/
extern bool L2CA_SetTxPriority(uint16_t cid, tL2CAP_CHNL_PRIORITY priority);

/*******************************************************************************
 *
 * Function         L2CA_SetFlushTimeout
 *
 * Description      This function set the automatic flush time out in Baseband
 *                  for ACL-U packets.
 *                  BdAddr : the remote BD address of ACL link. If it is
 *                           BT_DB_ANY then the flush time out will be applied
 *                           to all ACL link.
 *                  FlushTimeout: flush time out in ms
 *                           0x0000 : No automatic flush
 *                           L2CAP_NO_RETRANSMISSION : No retransmission
 *                           0x0002 - 0xFFFE : flush time out, if
 *                                             (flush_tout * 8) + 3 / 5) <=
 *                                             HCI_MAX_AUTOMATIC_FLUSH_TIMEOUT
 *                                             (in 625us slot).
 *                                    Otherwise, return false.
 *                           L2CAP_NO_AUTOMATIC_FLUSH : No automatic flush
 *
 * Returns          true if command succeeded, false if failed
 *
 * NOTE             This flush timeout applies to all logical channels active on
 *                  the ACL link.
 ******************************************************************************/
extern bool L2CA_SetFlushTimeout(const RawAddress& bd_addr,
                                 uint16_t flush_tout);

/*******************************************************************************
 *
 * Function         L2CA_SetChnlFlushability
 *
 * Description      Higher layers call this function to set a channels
 *                  flushability flags
 *
 * Returns          true if CID found, else false
 *
 ******************************************************************************/
extern bool L2CA_SetChnlFlushability(uint16_t cid, bool is_flushable);

/*******************************************************************************
 *
 *  Function         L2CA_GetPeerFeatures
 *
 *  Description      Get a peers features and fixed channel map
 *
 *  Parameters:      BD address of the peer
 *                   Pointers to features and channel mask storage area
 *
 *  Return value:    true if peer is connected
 *
 ******************************************************************************/
extern bool L2CA_GetPeerFeatures(const RawAddress& bd_addr,
                                 uint32_t* p_ext_feat, uint8_t* p_chnl_mask);

/*******************************************************************************
 *
 *                      Fixed Channel callback prototypes
 *
 ******************************************************************************/

/* Fixed channel connected and disconnected. Parameters are
 *      channel
 *      BD Address of remote
 *      true if channel is connected, false if disconnected
 *      Reason for connection failure
 *      transport : physical transport, BR/EDR or LE
 */
typedef void(tL2CA_FIXED_CHNL_CB)(uint16_t, const RawAddress&, bool, uint16_t,
                                  tBT_TRANSPORT);

/* Signalling data received. Parameters are
 *      channel
 *      BD Address of remote
 *      Pointer to buffer with data
 */
typedef void(tL2CA_FIXED_DATA_CB)(uint16_t, const RawAddress&, BT_HDR*);

/* Congestion status callback protype. This callback is optional. If
 * an application tries to send data when the transmit queue is full,
 * the data will anyways be dropped. The parameter is:
 *      remote BD_ADDR
 *      true if congested, false if uncongested
 */
typedef void(tL2CA_FIXED_CONGESTION_STATUS_CB)(const RawAddress&, bool);

/* Fixed channel registration info (the callback addresses and channel config)
 */
typedef struct {
  tL2CA_FIXED_CHNL_CB* pL2CA_FixedConn_Cb;
  tL2CA_FIXED_DATA_CB* pL2CA_FixedData_Cb;
  tL2CA_FIXED_CONGESTION_STATUS_CB* pL2CA_FixedCong_Cb;

  uint16_t default_idle_tout;
  tL2CA_TX_COMPLETE_CB*
      pL2CA_FixedTxComplete_Cb; /* fixed channel tx complete callback */
} tL2CAP_FIXED_CHNL_REG;

/*******************************************************************************
 *
 *  Function        L2CA_RegisterFixedChannel
 *
 *  Description     Register a fixed channel.
 *
 *  Parameters:     Fixed Channel #
 *                  Channel Callbacks and config
 *
 *  Return value:   true if registered OK
 *
 ******************************************************************************/
extern bool L2CA_RegisterFixedChannel(uint16_t fixed_cid,
                                      tL2CAP_FIXED_CHNL_REG* p_freg);

/*******************************************************************************
 *
 *  Function        L2CA_ConnectFixedChnl
 *
 *  Description     Connect an fixed signalling channel to a remote device.
 *
 *  Parameters:     Fixed CID
 *                  BD Address of remote
 *
 *  Return value:   true if connection started
 *
 ******************************************************************************/
extern bool L2CA_ConnectFixedChnl(uint16_t fixed_cid,
                                  const RawAddress& bd_addr);
extern bool L2CA_ConnectFixedChnl(uint16_t fixed_cid, const RawAddress& bd_addr,
                                  uint8_t initiating_phys);

/*******************************************************************************
 *
 *  Function        L2CA_SendFixedChnlData
 *
 *  Description     Write data on a fixed signalling channel.
 *
 *  Parameters:     Fixed CID
 *                  BD Address of remote
 *                  Pointer to buffer of type BT_HDR
 *
 * Return value     L2CAP_DW_SUCCESS, if data accepted
 *                  L2CAP_DW_FAILED,  if error
 *
 ******************************************************************************/
extern uint16_t L2CA_SendFixedChnlData(uint16_t fixed_cid,
                                       const RawAddress& rem_bda,
                                       BT_HDR* p_buf);

/*******************************************************************************
 *
 *  Function        L2CA_RemoveFixedChnl
 *
 *  Description     Remove a fixed channel to a remote device.
 *
 *  Parameters:     Fixed CID
 *                  BD Address of remote
 *                  Idle timeout to use (or 0xFFFF if don't care)
 *
 *  Return value:   true if channel removed
 *
 ******************************************************************************/
extern bool L2CA_RemoveFixedChnl(uint16_t fixed_cid, const RawAddress& rem_bda);

/*******************************************************************************
 *
 * Function         L2CA_SetFixedChannelTout
 *
 * Description      Higher layers call this function to set the idle timeout for
 *                  a fixed channel. The "idle timeout" is the amount of time
 *                  that a connection can remain up with no L2CAP channels on
 *                  it. A timeout of zero means that the connection will be torn
 *                  down immediately when the last channel is removed.
 *                  A timeout of 0xFFFF means no timeout. Values are in seconds.
 *                  A bd_addr is the remote BD address. If bd_addr =
 *                  RawAddress::kAny, then the idle timeouts for all active
 *                  l2cap links will be changed.
 *
 * Returns          true if command succeeded, false if failed
 *
 ******************************************************************************/
extern bool L2CA_SetFixedChannelTout(const RawAddress& rem_bda,
                                     uint16_t fixed_cid, uint16_t idle_tout);

/*******************************************************************************
 *
 *  Function        L2CA_CancelBleConnectReq
 *
 *  Description     Cancel a pending connection attempt to a BLE device.
 *
 *  Parameters:     BD Address of remote
 *
 *  Return value:   true if connection was cancelled
 *
 ******************************************************************************/
extern bool L2CA_CancelBleConnectReq(const RawAddress& rem_bda);

/*******************************************************************************
 *
 *  Function        L2CA_UpdateBleConnParams
 *
 *  Description     Update BLE connection parameters.
 *
 *  Parameters:     BD Address of remote
 *
 *  Return value:   true if update started
 *
 ******************************************************************************/
extern bool L2CA_UpdateBleConnParams(const RawAddress& rem_bdRa,
                                     uint16_t min_int, uint16_t max_int,
                                     uint16_t latency, uint16_t timeout);
extern bool L2CA_UpdateBleConnParams(const RawAddress& rem_bda,
                                     uint16_t min_int, uint16_t max_int,
                                     uint16_t latency, uint16_t timeout,
                                     uint16_t min_ce_len, uint16_t max_ce_len);

/*******************************************************************************
 *
 *  Function        L2CA_EnableUpdateBleConnParams
 *
 *  Description     Update BLE connection parameters.
 *
 *  Parameters:     BD Address of remote
 *                  enable flag
 *
 *  Return value:   true if update started
 *
 ******************************************************************************/
extern bool L2CA_EnableUpdateBleConnParams(const RawAddress& rem_bda,
                                           bool enable);

/*******************************************************************************
 *
 * Function         L2CA_GetBleConnRole
 *
 * Description      This function returns the connection role.
 *
 * Returns          link role.
 *
 ******************************************************************************/
extern uint8_t L2CA_GetBleConnRole(const RawAddress& bd_addr);

/*******************************************************************************
 *
 * Function         L2CA_GetDisconnectReason
 *
 * Description      This function returns the disconnect reason code.
 *
 *  Parameters:     BD Address of remote
 *                  Physical transport for the L2CAP connection (BR/EDR or LE)
 *
 * Returns          disconnect reason
 *
 ******************************************************************************/
extern uint16_t L2CA_GetDisconnectReason(const RawAddress& remote_bda,
                                         tBT_TRANSPORT transport);

extern void L2CA_AdjustConnectionIntervals(uint16_t* min_interval,
                                           uint16_t* max_interval,
                                           uint16_t floor_interval);

/**
 * Update max fixed channel tx data length if applicable
 */
extern void L2CA_SetLeFixedChannelTxDataLength(const RawAddress& remote_bda,
                                               uint16_t fix_cid,
                                               uint16_t tx_mtu);

/**
 * Check whether an ACL or LE link to the remote device is established
 */
extern bool L2CA_IsLinkEstablished(const RawAddress& bd_addr,
                                   tBT_TRANSPORT transport);

#endif /* L2C_API_H */
