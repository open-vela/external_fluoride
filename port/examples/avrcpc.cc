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

#include "fluoride.h"

static void btavrcp_passthrough_response_callback(const RawAddress& bd_addr, int id, int pressed) TRACE_CALLBACK_BODY
static void btavrcp_groupnavigation_response_callback(int id, int pressed) TRACE_CALLBACK_BODY
static void btavrcp_get_rcfeatures_callback(const RawAddress& bd_addr, int features) TRACE_CALLBACK_BODY
static void btavrcp_setplayerapplicationsetting_rsp_callback( const RawAddress& bd_addr, uint8_t accepted) TRACE_CALLBACK_BODY
static void btavrcp_playerapplicationsetting_callback( const RawAddress& bd_addr, uint8_t num_attr, btrc_player_app_attr_t *app_attrs, uint8_t num_ext_attr, btrc_player_app_ext_attr_t *ext_attrs) TRACE_CALLBACK_BODY
static void btavrcp_playerapplicationsetting_changed_callback( const RawAddress& bd_addr, const btrc_player_settings_t& vals) TRACE_CALLBACK_BODY
static void btavrcp_set_abs_vol_cmd_callback(const RawAddress& bd_addr, uint8_t abs_vol, uint8_t label) TRACE_CALLBACK_BODY
static void btavrcp_register_notification_absvol_callback( const RawAddress& bd_addr, uint8_t label) TRACE_CALLBACK_BODY
static void btavrcp_track_changed_callback(const RawAddress& bd_addr, uint8_t num_attr, btrc_element_attr_val_t *p_attrs) TRACE_CALLBACK_BODY
static void btavrcp_play_position_changed_callback(const RawAddress& bd_addr, uint32_t song_len, uint32_t song_pos) TRACE_CALLBACK_BODY
static void btavrcp_play_status_changed_callback( const RawAddress& bd_addr, btrc_play_status_t play_status) TRACE_CALLBACK_BODY
static void btavrcp_get_folder_items_callback( const RawAddress& bd_addr, btrc_status_t status, const btrc_folder_items_t *folder_items, uint8_t count) TRACE_CALLBACK_BODY
static void btavrcp_change_path_callback(const RawAddress& bd_addr, uint32_t count) TRACE_CALLBACK_BODY
static void btavrcp_set_browsed_player_callback(const RawAddress& bd_addr, uint8_t num_items, uint8_t depth) TRACE_CALLBACK_BODY
static void btavrcp_set_addressed_player_callback(const RawAddress& bd_addr, uint8_t status) TRACE_CALLBACK_BODY
static void btavrcp_addressed_player_changed_callback(const RawAddress& bd_addr, uint16_t id) TRACE_CALLBACK_BODY
static void btavrcp_now_playing_content_changed_callback( const RawAddress& bd_addr) TRACE_CALLBACK_BODY
static void btavrcp_available_player_changed_callback ( const RawAddress& bd_addr) TRACE_CALLBACK_BODY
static void btavrcp_get_rcpsm_callback(const RawAddress& bd_addr, uint16_t psm) TRACE_CALLBACK_BODY

static void btavrcp_connection_state_callback(bool rc_connect, bool br_connect, const RawAddress& bd_addr)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  flrd->avrcp_addr = bd_addr;
}

static btrc_ctrl_callbacks_t sBluetoothAvrcpCallbacks =
{
  sizeof(sBluetoothAvrcpCallbacks),
  btavrcp_passthrough_response_callback,
  btavrcp_groupnavigation_response_callback,
  btavrcp_connection_state_callback,
  btavrcp_get_rcfeatures_callback,
  btavrcp_setplayerapplicationsetting_rsp_callback,
  btavrcp_playerapplicationsetting_callback,
  btavrcp_playerapplicationsetting_changed_callback,
  btavrcp_set_abs_vol_cmd_callback,
  btavrcp_register_notification_absvol_callback,
  btavrcp_track_changed_callback,
  btavrcp_play_position_changed_callback,
  btavrcp_play_status_changed_callback,
  btavrcp_get_folder_items_callback,
  btavrcp_change_path_callback,
  btavrcp_set_browsed_player_callback,
  btavrcp_set_addressed_player_callback,
  btavrcp_addressed_player_changed_callback,
  btavrcp_now_playing_content_changed_callback,
  btavrcp_available_player_changed_callback,
  btavrcp_get_rcpsm_callback
};

const btrc_ctrl_interface_t *bt_profile_avrcp_control_init(struct fluoride_s *flrd)
{
  const btrc_ctrl_interface_t *rcctrl;

  rcctrl = (const btrc_ctrl_interface_t *)
    flrd->interface->get_profile_interface(BT_PROFILE_AV_RC_CTRL_ID);
  if (rcctrl == NULL)
    return rcctrl;

  rcctrl->init(&sBluetoothAvrcpCallbacks);

  return rcctrl;
}
