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
  struct btav_source_cb_t* _cb;
  bt_property_t *property;
  bt_addr_t addr;

  LOG_SAMPLES("%s: state: %d\n", __func__, state);

  if (state == BTAV_CONNECTION_STATE_DISCONNECTED) {
    property = property_new_scan_mode(BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
    flrd->interface->set_adapter_property(property);
    property_free(property);

    if (flrd->arole == AVDT_TSEP_SNK) {
      if (flrd->sink)
        flrd->sink->set_active_device(RawAddress::kEmpty);
    } else {
      if (flrd->source)
        flrd->source->set_active_device(RawAddress::kEmpty);
    }
  } else if (state == BTAV_CONNECTION_STATE_CONNECTED) {
    property = property_new_scan_mode(BT_SCAN_MODE_NONE);
    flrd->interface->set_adapter_property(property);
    property_free(property);

    flrd->addr = bd_addr;

    if (flrd->arole == AVDT_TSEP_SNK) {
      if (flrd->sink)
        flrd->sink->set_active_device(bd_addr);
    } else {
      if (flrd->source)
        flrd->source->set_active_device(bd_addr);
    }
  }

  memcpy(addr.val, bd_addr.address, sizeof(addr.val));
  for (_cb = flrd->a2dp_source_cb; _cb; _cb = _cb->_next) {
    if(_cb->a2dp_conn_state_cb)
      flrd->a2dp_source_cb->a2dp_conn_state_cb(addr, state);
  }
}

static void bta2dp_audio_state_callback(const RawAddress& bd_addr,
    btav_audio_state_t state)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  struct btav_source_cb_t* _cb;
  bt_addr_t addr;

  LOG_SAMPLES("%s: state: %d\n", __func__, state);
  memcpy(addr.val, bd_addr.address, sizeof(addr.val));
  for (_cb = flrd->a2dp_source_cb; _cb; _cb = _cb->_next) {
    if(_cb->a2dp_audio_state_cb)
      flrd->a2dp_source_cb->a2dp_audio_state_cb(addr, state);
  }
}

static void bta2dp_audio_config_callback(const RawAddress& bd_addr,
    uint32_t sample_rate, uint8_t channel_count)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  struct btav_source_cb_t* _cb;
  bt_addr_t addr;

  LOG_SAMPLES("%s: sample_rate: %" PRIu32 ", channel_count: %d\n",
      __func__, sample_rate, channel_count);
  memcpy(addr.val, bd_addr.address, sizeof(addr.val));
  for (_cb = flrd->a2dp_source_cb; _cb; _cb = _cb->_next) {
    if(_cb->a2dp_audio_state_cb)
      flrd->a2dp_source_cb->a2dp_audio_config_cb(addr, sample_rate, channel_count);
  }
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

/** BT-AV A2DP Source callback structure. */

static void bta2dp_audio_source_config_callback(const RawAddress& bd_addr,
    btav_a2dp_codec_config_t codec_config,
    std::vector<btav_a2dp_codec_config_t> codecs_local_capabilities,
    std::vector<btav_a2dp_codec_config_t> codecs_selectable_capabilities) TRACE_CALLBACK_BODY

static bool bta2dp_mandatory_codec_preferred_callback( const RawAddress& bd_addr)
{
  LOG_SAMPLES("UNIMPLEMENTED: %s: %d\n", __func__, __LINE__);
  return true;
}

static btav_source_callbacks_t sSourceBluetoothA2dpCallbacks =
{
  sizeof(sSourceBluetoothA2dpCallbacks),
  bta2dp_connection_state_callback,
  bta2dp_audio_state_callback,
  bta2dp_audio_source_config_callback,
  bta2dp_mandatory_codec_preferred_callback,
};

const btav_source_interface_t *bt_profile_a2dp_source_init(struct fluoride_s *flrd)
{
  std::vector<btav_a2dp_codec_config_t> priorities;
  std::vector<btav_a2dp_codec_config_t> offloading;
  const btav_source_interface_t *source;

  source = (const btav_source_interface_t *)
    flrd->interface->get_profile_interface(BT_PROFILE_ADVANCED_AUDIO_ID);
  if (source == NULL)
    return source;

  source->init(&sSourceBluetoothA2dpCallbacks, 1, priorities, offloading);

  return source;
}

extern "C"
{
  void a2dp_source_register_cb(struct btav_source_cb_t* cb)
  {
    struct fluoride_s *flrd = fluoride_interface_get();
    struct btav_source_cb_t* _cb;

    for (_cb = flrd->a2dp_source_cb; _cb; _cb = _cb->_next)
      if (_cb == cb)
        return;

    cb->_next = flrd->a2dp_source_cb;
    flrd->a2dp_source_cb = cb;
  }

  int a2dp_source_connect(bt_addr_t addr)
  {
    struct fluoride_s *flrd = fluoride_interface_get();
    RawAddress bd_addr;

    bd_addr.FromOctets(addr.val);
    return flrd->source->connect(bd_addr);
  }

  int a2dp_source_disconnect(bt_addr_t addr)
  {
    struct fluoride_s *flrd = fluoride_interface_get();
    RawAddress bd_addr;

    bd_addr.FromOctets(addr.val);
    return flrd->source->disconnect(bd_addr);
  }
}
