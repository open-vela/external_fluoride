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

static void bta2dp_connection_state_callback(const RawAddress& bd_addr,
    btav_connection_state_t state)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  bt_property_t *property;

  LOG_SAMPLES("%s: state: %d\n", __func__, state);
  if (state == BTAV_CONNECTION_STATE_DISCONNECTED) {
    property = property_new_scan_mode(BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
    flrd->interface->set_adapter_property(property);
    property_free(property);
  } else if (state == BTAV_CONNECTION_STATE_CONNECTED ||
      state == BTAV_CONNECTION_STATE_CONNECTING) {
    flrd->addr = bd_addr;
  }
}

static void bta2dp_audio_state_callback(const RawAddress& bd_addr,
    btav_audio_state_t state)
{
  LOG_SAMPLES("%s: state: %d\n", __func__, state);
}

static void bta2dp_audio_config_callback(const RawAddress& bd_addr,
    uint32_t sample_rate, uint8_t channel_count)
{
  LOG_SAMPLES("%s: sample_rate: %" PRIu32 ", channel_count: %d\n",
      __func__, sample_rate, channel_count);
}

static btav_sink_callbacks_t sSinkBluetoothA2dpCallbacks =
{
  sizeof(sSinkBluetoothA2dpCallbacks),
  bta2dp_connection_state_callback,
  bta2dp_audio_state_callback,
  bta2dp_audio_config_callback,
};

const btav_sink_interface_t *bt_profile_a2dp_sink_init(struct fluoride_s *flrd)
{
  const btav_sink_interface_t *sink;

  sink = (const btav_sink_interface_t *)
    flrd->interface->get_profile_interface(BT_PROFILE_ADVANCED_AUDIO_SINK_ID);
  if (sink == NULL)
    return sink;

  sink->init(&sSinkBluetoothA2dpCallbacks);
  sink->set_audio_focus_state(BTIF_A2DP_SINK_FOCUS_GRANTED);

  return sink;
}
