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

/*****************************************************************************
 *
 *  This file contains definitions internal to the RFC unit
 *
 *****************************************************************************/

#ifndef RFC_INT_H
#define RFC_INT_H

#include "l2c_api.h"
#include "port_int.h"

#include <unordered_map>

/*
 * Define RFCOMM result codes
*/
#define RFCOMM_SUCCESS 0
#define RFCOMM_ERROR 1
#define RFCOMM_SECURITY_ERR 112

/*
 * Define max and min RFCOMM MTU (N1)
*/
#define RFCOMM_MIN_MTU 23
#define RFCOMM_MAX_MTU 32767

extern void RFCOMM_StartReq(tRFC_MCB* p_mcb);
extern void RFCOMM_StartRsp(tRFC_MCB* p_mcb, uint16_t result);

extern void RFCOMM_DlcEstablishReq(tRFC_MCB* p_mcb, uint8_t dlci, uint16_t mtu);
extern void RFCOMM_DlcEstablishRsp(tRFC_MCB* p_mcb, uint8_t dlci, uint16_t mtu,
                                   uint16_t result);

extern void RFCOMM_DataReq(tRFC_MCB* p_mcb, uint8_t dlci, BT_HDR* p_buf);

extern void RFCOMM_DlcReleaseReq(tRFC_MCB* p_mcb, uint8_t dlci);

extern void RFCOMM_ParameterNegotiationRequest(tRFC_MCB* p_mcb, uint8_t dlci,
                                               uint16_t mtu);
extern void RFCOMM_ParameterNegotiationResponse(tRFC_MCB* p_mcb, uint8_t dlci,
                                                uint16_t mtu, uint8_t cl,
                                                uint8_t k);

extern void RFCOMM_FlowReq(tRFC_MCB* p_mcb, uint8_t dlci, bool state);

extern void RFCOMM_PortParameterNegotiationRequest(tRFC_MCB* p_mcb,
                                                   uint8_t dlci,
                                                   tPORT_STATE* p_pars);
extern void RFCOMM_PortParameterNegotiationResponse(tRFC_MCB* p_mcb,
                                                    uint8_t dlci,
                                                    tPORT_STATE* p_pars,
                                                    uint16_t param_mask);

extern void RFCOMM_ControlReq(tRFC_MCB* p_mcb, uint8_t dlci,
                              tPORT_CTRL* p_pars);
extern void RFCOMM_ControlRsp(tRFC_MCB* p_mcb, uint8_t dlci,
                              tPORT_CTRL* p_pars);

extern void RFCOMM_LineStatusReq(tRFC_MCB* p_mcb, uint8_t dlci,
                                 uint8_t line_status);
/*
 * Define logical struct used for sending and decoding MX frames
*/
typedef struct {
  uint8_t dlci;
  uint8_t type;
  uint8_t cr;
  uint8_t ea;
  uint8_t pf;
  uint8_t credit;

  union {
    struct {
      uint8_t dlci;
      uint8_t frame_type;
      uint8_t conv_layer;
      uint8_t priority;
      uint8_t t1;
      uint16_t mtu;
      uint8_t n2;
      uint8_t k;
    } pn;
    struct {
      uint8_t* p_data;
      uint16_t data_len;
    } test;
    struct {
      uint8_t dlci;
      uint8_t signals;
      uint8_t break_present;
      uint8_t break_duration;
    } msc;
    struct {
      uint8_t ea;
      uint8_t cr;
      uint8_t type;
    } nsc;
    struct {
      uint8_t dlci;
      uint8_t is_request;
      uint8_t baud_rate;
      uint8_t byte_size;
      uint8_t stop_bits;
      uint8_t parity;
      uint8_t parity_type;
      uint8_t fc_type;
      uint8_t xon_char;
      uint8_t xoff_char;
      uint16_t param_mask;
    } rpn;
    struct {
      uint8_t dlci;
      uint8_t line_status;
    } rls;
  } u;
} MX_FRAME;

#define LINE_STATUS_NO_ERROR 0x00
#define LINE_STATUS_OVERRUN 0x02  /* Receive Overrun Error   */
#define LINE_STATUS_RXPARITY 0x04 /* Receive Parity Error    */
#define LINE_STATUS_FRAME 0x08    /* Receive Framing error   */
#define LINE_STATUS_FAILED 0x10   /* Connection Failed       */

/*
 * Define states and events for the RFC multiplexer state machine
*/
typedef enum : uint16_t {
  RFC_MX_STATE_IDLE = 0,
  RFC_MX_STATE_WAIT_CONN_CNF = 1,
  RFC_MX_STATE_CONFIGURE = 2,
  RFC_MX_STATE_SABME_WAIT_UA = 3,
  RFC_MX_STATE_WAIT_SABME = 4,
  RFC_MX_STATE_CONNECTED = 5,
  RFC_MX_STATE_DISC_WAIT_UA = 6,
} tRFC_MX_STATE;

inline std::string rfcomm_mx_state_text(tRFC_MX_STATE state) {
  switch (state) {
    case RFC_MX_STATE_IDLE:
      return std::string("idle");
    case RFC_MX_STATE_WAIT_CONN_CNF:
      return std::string("wait_config");
    case RFC_MX_STATE_CONFIGURE:
      return std::string("configure");
    case RFC_MX_STATE_SABME_WAIT_UA:
      return std::string("sabme_wait_ua");
    case RFC_MX_STATE_WAIT_SABME:
      return std::string("wait_sabme");
    case RFC_MX_STATE_CONNECTED:
      return std::string("connected");
    case RFC_MX_STATE_DISC_WAIT_UA:
      return std::string("disconnect_wait_ua");
    default:
      return std::string("UNKNOWN");
  }
}

/*
 * Define port states
 */
#define RFC_STATE_CLOSED 0
#define RFC_STATE_SABME_WAIT_UA 1
#define RFC_STATE_ORIG_WAIT_SEC_CHECK 2
#define RFC_STATE_TERM_WAIT_SEC_CHECK 3
#define RFC_STATE_OPENED 4
#define RFC_STATE_DISC_WAIT_UA 5

/*
 * Events that can be received by multiplexer as well as port state machines
*/
#define RFC_EVENT_SABME 0
#define RFC_EVENT_UA 1
#define RFC_EVENT_DM 2
#define RFC_EVENT_DISC 3
#define RFC_EVENT_UIH 4
#define RFC_EVENT_TIMEOUT 5
#define RFC_EVENT_BAD_FRAME 50
/*
 * Multiplexer events
*/
#define RFC_MX_EVENT_START_REQ 6
#define RFC_MX_EVENT_START_RSP 7
#define RFC_MX_EVENT_CLOSE_REQ 8
#define RFC_MX_EVENT_CONN_CNF 9
#define RFC_MX_EVENT_CONN_IND 10
#define RFC_MX_EVENT_CONF_CNF 11
#define RFC_MX_EVENT_CONF_IND 12
#define RFC_MX_EVENT_QOS_VIOLATION_IND 13
#define RFC_MX_EVENT_DISC_IND 14

/*
 * Port events
*/
#define RFC_EVENT_OPEN 9
#define RFC_EVENT_ESTABLISH_RSP 11
#define RFC_EVENT_CLOSE 12
#define RFC_EVENT_CLEAR 13
#define RFC_EVENT_DATA 14
#define RFC_EVENT_SEC_COMPLETE 15

/* seconds to wait for reply with Poll bit */
#define RFC_T1_TIMEOUT 20
/* seconds to wait for reply with Poll bit other than MX */
#define RFC_PORT_T1_TIMEOUT 60
/* timeout to wait for Mx UIH */
#define RFC_T2_TIMEOUT 20
/* If something goes wrong and we send DISC we should not wait for min */
#define RFC_DISC_TIMEOUT 3
#define RFC_CLOSE_TIMEOUT 10
/* first connection to be established on Mx */
#define RFCOMM_CONN_TIMEOUT 120

/* Define RFComm control block
*/
typedef struct {
  MX_FRAME rx_frame;
  tL2CAP_APPL_INFO reg_info; /* L2CAP Registration info */

  /* MCB based on the L2CAP's lcid */
  tRFC_MCB* p_rfc_lcid_mcb[MAX_L2CAP_CHANNELS];
  bool peer_rx_disabled; /* If true peer sent FCOFF */
  uint8_t last_mux;      /* Last mux allocated */
  uint8_t last_port_index;  // Index of last port allocated in rfc_cb.port
} tRFCOMM_CB;

/* Main Control Block for the RFCOMM Layer (PORT and RFC) */
typedef struct {
  tRFCOMM_CB rfc;
  tPORT_CB port;
  uint8_t trace_level;
} tRFC_CB;

extern tRFC_CB rfc_cb;

extern std::unordered_map<uint32_t /* scn */, uint16_t /* sec_mask */>
    rfcomm_security_records;

/* Timer running on the multiplexor channel while no DLCI connection is open */
#define RFC_MCB_INIT_INACT_TIMER 60 /* in seconds */

/* Timer running on the multiplexor channel after last DLCI is released */
#define RFC_MCB_RELEASE_INACT_TIMER 2 /* in seconds */

#ifdef RFCOMM_PRECALC_FCS

#define RFCOMM_SABME_FCS(p_data, cr, dlci) rfc_sabme_fcs[cr][dlci]
#define RFCOMM_UA_FCS(p_data, cr, dlci) rfc_ua_fcs[cr][dlci]
#define RFCOMM_DM_FCS(p_data, cr, dlci) rfc_dm_fcs[cr][dlci]
#define RFCOMM_DISC_FCS(p_data, cr, dlci) rfc_disc_fcs[cr][dlci]
#define RFCOMM_UIH_FCS(p_data, dlci) rfc_uih_fcs[dlci]

#else

extern uint8_t rfc_calc_fcs(uint16_t len, uint8_t* p);

#define RFCOMM_SABME_FCS(p_data, cr, dlci) rfc_calc_fcs(3, p_data)
#define RFCOMM_UA_FCS(p_data, cr, dlci) rfc_calc_fcs(3, p_data)
#define RFCOMM_DM_FCS(p_data, cr, dlci) rfc_calc_fcs(3, p_data)
#define RFCOMM_DISC_FCS(p_data, cr, dlci) rfc_calc_fcs(3, p_data)
#define RFCOMM_UIH_FCS(p_data, dlci) rfc_calc_fcs(2, p_data)

#endif

extern void rfc_mx_sm_execute(tRFC_MCB* p_mcb, uint16_t event, void* p_data);

/*
 * Functions provided by the rfc_port_fsm.cc
*/
extern void rfc_port_sm_execute(tPORT* p_port, uint16_t event, void* p_data);

extern void rfc_process_pn(tRFC_MCB* p_rfc_mcb, bool is_command,
                           MX_FRAME* p_frame);
extern void rfc_process_msc(tRFC_MCB* p_rfc_mcb, bool is_command,
                            MX_FRAME* p_frame);
extern void rfc_process_rpn(tRFC_MCB* p_rfc_mcb, bool is_command,
                            bool is_request, MX_FRAME* p_frame);
extern void rfc_process_rls(tRFC_MCB* p_rfc_mcb, bool is_command,
                            MX_FRAME* p_frame);
extern void rfc_process_nsc(tRFC_MCB* p_rfc_mcb, MX_FRAME* p_frame);
extern void rfc_process_test_rsp(tRFC_MCB* p_rfc_mcb, BT_HDR* p_buf);
extern void rfc_process_fcon(tRFC_MCB* p_rfc_mcb, bool is_command);
extern void rfc_process_fcoff(tRFC_MCB* p_rfc_mcb, bool is_command);
extern void rfc_process_l2cap_congestion(tRFC_MCB* p_mcb, bool is_congested);

void rfc_on_l2cap_error(uint16_t lcid, uint16_t result);

/*
 * Functions provided by the rfc_utils.cc
*/
tRFC_MCB* rfc_alloc_multiplexer_channel(const RawAddress& bd_addr,
                                        bool is_initiator);
extern void rfc_release_multiplexer_channel(tRFC_MCB* p_rfc_mcb);
extern void rfc_timer_start(tRFC_MCB* p_rfc_mcb, uint16_t timeout);
extern void rfc_timer_stop(tRFC_MCB* p_rfc_mcb);
extern void rfc_port_timer_start(tPORT* p_port, uint16_t tout);
extern void rfc_port_timer_stop(tPORT* p_port);

bool rfc_check_fcs(uint16_t len, uint8_t* p, uint8_t received_fcs);
tRFC_MCB* rfc_find_lcid_mcb(uint16_t lcid);
extern void rfc_save_lcid_mcb(tRFC_MCB* p_rfc_mcb, uint16_t lcid);
extern void rfc_check_mcb_active(tRFC_MCB* p_mcb);
extern void rfc_port_closed(tPORT* p_port);
extern void rfc_sec_check_complete(const RawAddress* bd_addr,
                                   tBT_TRANSPORT transport, void* p_ref_data,
                                   uint8_t res);
extern void rfc_inc_credit(tPORT* p_port, uint8_t credit);
extern void rfc_dec_credit(tPORT* p_port);
extern void rfc_check_send_cmd(tRFC_MCB* p_mcb, BT_HDR* p_buf);

/*
 * Functions provided by the rfc_ts_frames.cc
*/
extern void rfc_send_sabme(tRFC_MCB* p_rfc_mcb, uint8_t dlci);
extern void rfc_send_ua(tRFC_MCB* p_rfc_mcb, uint8_t dlci);
extern void rfc_send_dm(tRFC_MCB* p_rfc_mcb, uint8_t dlci, bool pf);
extern void rfc_send_disc(tRFC_MCB* p_rfc_mcb, uint8_t dlci);
extern void rfc_send_pn(tRFC_MCB* p_mcb, uint8_t dlci, bool is_command,
                        uint16_t mtu, uint8_t cl, uint8_t k);
extern void rfc_send_test(tRFC_MCB* p_rfc_mcb, bool is_command, BT_HDR* p_buf);
extern void rfc_send_msc(tRFC_MCB* p_mcb, uint8_t dlci, bool is_command,
                         tPORT_CTRL* p_pars);
extern void rfc_send_rls(tRFC_MCB* p_mcb, uint8_t dlci, bool is_command,
                         uint8_t status);
extern void rfc_send_rpn(tRFC_MCB* p_mcb, uint8_t dlci, bool is_command,
                         tPORT_STATE* p_pars, uint16_t mask);
extern void rfc_send_fcon(tRFC_MCB* p_mcb, bool is_command);
extern void rfc_send_fcoff(tRFC_MCB* p_mcb, bool is_command);
extern void rfc_send_buf_uih(tRFC_MCB* p_rfc_mcb, uint8_t dlci, BT_HDR* p_buf);
extern void rfc_send_credit(tRFC_MCB* p_mcb, uint8_t dlci, uint8_t credit);
extern void rfc_process_mx_message(tRFC_MCB* p_rfc_mcb, BT_HDR* p_buf);
extern uint8_t rfc_parse_data(tRFC_MCB* p_rfc_mcb, MX_FRAME* p_frame,
                              BT_HDR* p_buf);

/* Call back functions from RFCOMM */
extern void rfcomm_l2cap_if_init(void);

extern void PORT_StartInd(tRFC_MCB* p_mcb);
extern void PORT_StartCnf(tRFC_MCB* p_mcb, uint16_t result);

extern void PORT_CloseInd(tRFC_MCB* p_mcb);
extern void Port_TimeOutCloseMux(tRFC_MCB* p_mcb);

extern void PORT_DlcEstablishInd(tRFC_MCB* p_mcb, uint8_t dlci, uint16_t mtu);
extern void PORT_DlcEstablishCnf(tRFC_MCB* p_mcb, uint8_t dlci, uint16_t mtu,
                                 uint16_t result);

extern void PORT_DataInd(tRFC_MCB* p_mcb, uint8_t dlci, BT_HDR* p_buf);

extern void PORT_DlcReleaseInd(tRFC_MCB* p_mcb, uint8_t dlci);

extern void PORT_ParNegInd(tRFC_MCB* p_mcb, uint8_t dlci, uint16_t mtu,
                           uint8_t cl, uint8_t k);
extern void PORT_ParNegCnf(tRFC_MCB* p_mcb, uint8_t dlci, uint16_t mtu,
                           uint8_t cl, uint8_t k);

extern void PORT_FlowInd(tRFC_MCB* p_mcb, uint8_t dlci, bool fc);

extern void PORT_PortNegInd(tRFC_MCB* p_mcb, uint8_t dlci, tPORT_STATE* p_pars,
                            uint16_t param_mask);
extern void PORT_PortNegCnf(tRFC_MCB* p_mcb, uint8_t dlci, tPORT_STATE* p_pars,
                            uint16_t result);

extern void PORT_ControlInd(tRFC_MCB* p_mcb, uint8_t dlci, tPORT_CTRL* p_pars);
extern void PORT_ControlCnf(tRFC_MCB* p_mcb, uint8_t dlci, tPORT_CTRL* p_pars);

extern void PORT_LineStatusInd(tRFC_MCB* p_mcb, uint8_t dlci,
                               uint8_t line_status);

#endif
