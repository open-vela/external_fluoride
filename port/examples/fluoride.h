/****************************************************************************
 *
 *   Copyright (C) 2021 Xiaomi InC. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __FLUORIDE_PORT_EXAMPLES_FLUORIDE_H
#define __FLUORIDE_PORT_EXAMPLES_FLUORIDE_H

#include "btif_common.h"
#include "btif_bqr.h"
#include "btif_a2dp_sink.h"
#include "avdt_api.h"
#include "hardware/bluetooth.h"
#include "hardware/bt_av.h"
#include "hardware/bt_gatt.h"
#include "hardware/bt_rc.h"
#include "hardware/bt_sdp.h"
#include "hardware/avrcp/avrcp_common.h"
#include "hardware/avrcp/avrcp.h"
#include "hardware/bt_hf_client.h"
#include "property.h"
#include "btif_util.h"

#define __TRACE_CALLBACK

#if defined(__TRACE_CALLBACK)
#define TRACE_CALLBACK_BODY { LOG_SAMPLES("UNIMPLEMENTED: %s: %d\n", __func__, __LINE__); }
#else
#define TRACE_CALLBACK_BODY { }
#endif

#define LOG_SAMPLES(fmt, args...) printf("CALLBACK " fmt, ##args)

using bluetooth::Uuid;
using namespace bluetooth::avrcp;

struct fluoride_s
{
  const bt_interface_t          *interface;

  const btrc_interface_t        *avrcp;
  const btrc_ctrl_interface_t   *ctrl;
  const btsdp_interface_t       *sdp;

  const btav_sink_interface_t   *sink;
  const btav_source_interface_t *source;

  const bthf_client_interface_t *hfc;
  const btgatt_interface_t      *gatt;

  ServiceInterface              *avrcs;

  bt_state_t                    state;
  pthread_mutex_t               mutex;
  pthread_cond_t                cond;

  RawAddress                    addr;
  RawAddress                    avrcp_addr;
};

const btgatt_interface_t      *bt_profile_gatt_init(struct fluoride_s *flrd);
const btrc_ctrl_interface_t   *bt_profile_avrcp_control_init(struct fluoride_s *flrd);
const btsdp_interface_t       *bt_profile_sdp_init(struct fluoride_s *flrd);
const btav_sink_interface_t   *bt_profile_a2dp_sink_init(struct fluoride_s *flrd);
const btrc_interface_t        *bt_profile_avrcp_init(struct fluoride_s *flrd);
const bthf_client_interface_t *bt_profile_handsfree_init(struct fluoride_s *flrd);
ServiceInterface              *bt_profile_avrcp_service_init(struct fluoride_s *flrd);

struct fluoride_s             *fluoride_interface_get(void);
int                            fluoride_shell(struct fluoride_s *flrd, int argc, char **argv);

#endif /* __FLUORIDE_PORT_EXAMPLES_FLUORIDE_H */
