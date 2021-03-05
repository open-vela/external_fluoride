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

void btavrcp_remote_features_callback(const RawAddress& bd_addr, btrc_remote_features_t features) TRACE_CALLBACK_BODY
void btavrcp_get_play_status_callback(const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_list_player_app_attr_callback(const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_list_player_app_values_callback(btrc_player_attr_t attr_id, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_get_player_app_value_callback(uint8_t num_attr, btrc_player_attr_t* p_attrs, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_get_player_app_attrs_text_callback(uint8_t num_attr, btrc_player_attr_t* p_attrs, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_get_player_app_values_text_callback(uint8_t attr_id, uint8_t num_val, uint8_t* p_vals, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_set_player_app_value_callback(btrc_player_settings_t* p_vals, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_get_element_attr_callback(uint8_t num_attr, btrc_media_attr_t* p_attrs, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_register_notification_callback(btrc_event_id_t event_id, uint32_t param, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_passthrough_cmd_callback(int id, int key_state, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_set_addressed_player_callback(uint16_t player_id, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_set_browsed_player_callback(uint16_t player_id, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_get_folder_items_callback(uint8_t scope, uint32_t start_item, uint32_t end_item, uint8_t num_attr, uint32_t* p_attr_ids, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_change_path_callback(uint8_t direction, uint8_t* folder_uid, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_get_item_attr_callback(uint8_t scope, uint8_t* uid, uint16_t uid_counter, uint8_t num_attr, btrc_media_attr_t* p_attrs, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_play_item_callback(uint8_t scope, uint16_t uid_counter, uint8_t* uid, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_get_total_num_of_items_callback(uint8_t scope, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_search_callback(uint16_t charset_id, uint16_t str_len, uint8_t* p_str, const RawAddress& bd_addr) TRACE_CALLBACK_BODY
void btavrcp_add_to_now_playing_callback(uint8_t scope, uint8_t* uid, uint16_t uid_counter, const RawAddress& bd_addr) TRACE_CALLBACK_BODY

void btavrcp_volume_change_callback(uint8_t volume, uint8_t ctype, const RawAddress& bd_addr)
{
  struct fluoride_s *flrd = fluoride_interface_get();

  if (flrd->ctrl)
    flrd->ctrl->set_volume_rsp(flrd->avrcp_addr, volume, 0);
  if (flrd->avrcp)
    flrd->avrcp->set_volume(volume);
}

static btrc_callbacks_t sBluetoothAvrcpInterfaceCallbacks =
{
  sizeof(sBluetoothAvrcpInterfaceCallbacks),
  btavrcp_remote_features_callback,
  btavrcp_get_play_status_callback,
  btavrcp_list_player_app_attr_callback,
  btavrcp_list_player_app_values_callback,
  btavrcp_get_player_app_value_callback,
  btavrcp_get_player_app_attrs_text_callback,
  btavrcp_get_player_app_values_text_callback,
  btavrcp_set_player_app_value_callback,
  btavrcp_get_element_attr_callback,
  btavrcp_register_notification_callback,
  btavrcp_volume_change_callback,
  btavrcp_passthrough_cmd_callback,
  btavrcp_set_addressed_player_callback,
  btavrcp_set_browsed_player_callback,
  btavrcp_get_folder_items_callback,
  btavrcp_change_path_callback,
  btavrcp_get_item_attr_callback,
  btavrcp_play_item_callback,
  btavrcp_get_total_num_of_items_callback,
  btavrcp_search_callback,
  btavrcp_add_to_now_playing_callback
};

const btrc_interface_t *bt_profile_avrcp_init(struct fluoride_s *flrd)
{
  const btrc_interface_t *avrc;

  avrc = (const btrc_interface_t *)
    flrd->interface->get_profile_interface(BT_PROFILE_AV_RC_ID);
  if (avrc == NULL)
    return avrc;

  avrc->init(&sBluetoothAvrcpInterfaceCallbacks);

  return avrc;
}
