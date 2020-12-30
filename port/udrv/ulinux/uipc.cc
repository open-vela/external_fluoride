/******************************************************************************
 *
 *  Copyright 2009-2012 Broadcom Corporation
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
 *  Filename:      uipc.cc
 *
 *  Description:   UIPC implementation for fluoride
 *
 *****************************************************************************/

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <poll.h>
#include <sys/prctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <mutex>
#include <set>

#include "audio_a2dp_hw/include/audio_a2dp_hw.h"
#include "bt_common.h"
#include "bt_types.h"
#include "bt_utils.h"
#include "osi/include/osi.h"
#include "osi/include/socket_utils/sockets.h"
#include "uipc.h"

/*****************************************************************************
 *  Constants & Macros
 *****************************************************************************/

#define CASE_RETURN_STR(const) \
  case const:                  \
    return #const;

/*****************************************************************************
 *  Local type definitions
 *****************************************************************************/

typedef enum {
  UIPC_TASK_FLAG_DISCONNECT_CHAN = 0x1,
} tUIPC_TASK_FLAGS;

const char* dump_uipc_event(tUIPC_EVENT event) {
  switch (event) {
    CASE_RETURN_STR(UIPC_OPEN_EVT)
    CASE_RETURN_STR(UIPC_CLOSE_EVT)
    CASE_RETURN_STR(UIPC_RX_DATA_EVT)
    CASE_RETURN_STR(UIPC_RX_DATA_READY_EVT)
    CASE_RETURN_STR(UIPC_TX_DATA_READY_EVT)
    default:
      return "UNKNOWN MSG ID";
  }
}

void uipc_main_cleanup(tUIPC_STATE& uipc) { }
void uipc_close_locked(tUIPC_STATE& uipc, tUIPC_CH_ID ch_id) { }
int uipc_start_main_server_thread(tUIPC_STATE& uipc) { return 0; }
void uipc_stop_main_server_thread(tUIPC_STATE& uipc) { }
std::unique_ptr<tUIPC_STATE> UIPC_Init() { std::unique_ptr<tUIPC_STATE> uipc = std::make_unique<tUIPC_STATE>(); return uipc; }
bool UIPC_Open(tUIPC_STATE& uipc, tUIPC_CH_ID ch_id, tUIPC_RCV_CBACK* p_cback, const char* socket_path) { return false; }
void UIPC_Close(tUIPC_STATE& uipc, tUIPC_CH_ID ch_id) { }
bool UIPC_Send(tUIPC_STATE& uipc, tUIPC_CH_ID ch_id, UNUSED_ATTR uint16_t msg_evt, const uint8_t* p_buf, uint16_t msglen) { return false; }
uint32_t UIPC_Read(tUIPC_STATE& uipc, tUIPC_CH_ID ch_id, UNUSED_ATTR uint16_t* p_msg_evt, uint8_t* p_buf, uint32_t len) { return 0; }
bool UIPC_Ioctl(tUIPC_STATE& uipc, tUIPC_CH_ID ch_id, uint32_t request, void* param) { return false; }
