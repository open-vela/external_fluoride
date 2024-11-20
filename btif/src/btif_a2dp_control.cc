/******************************************************************************
 *
 *  Copyright 2016 The Android Open Source Project
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

#define LOG_TAG "bt_btif_a2dp_control"

#include <base/logging.h>
#include <stdbool.h>
#include <stdint.h>

#include "audio_a2dp_hw/include/audio_a2dp_hw.h"
#include "bt_common.h"
#include "btif_a2dp.h"
#include "btif_a2dp_control.h"
#include "btif_a2dp_sink.h"
#include "btif_a2dp_source.h"
#include "btif_av.h"
#include "btif_av_co.h"
#include "btif_hf.h"
#include "osi/include/osi.h"
#include "uipc.h"

#define A2DP_DATA_READ_POLL_MS 10

struct {
  uint64_t total_bytes_read = 0;
  uint16_t audio_delay = 0;
  struct timespec timestamp = {};
} delay_report_stats;

static void btif_a2dp_data_cb(tUIPC_CH_ID ch_id, tUIPC_EVENT event);
static void btif_a2dp_ctrl_cb(tUIPC_CH_ID ch_id, tUIPC_EVENT event);

/* We can have max one command pending */
static tA2DP_CTRL_CMD a2dp_cmd_pending = A2DP_CTRL_CMD_NONE;
std::unique_ptr<tUIPC_STATE> a2dp_uipc = nullptr;

static const char *a2dp_uipc_path[] = {
  A2DP_CTRL_PATH,
  A2DP_DATA_PATH,
  "/data/misc/bluedroid/.sink_ctrl",
  "/data/misc/bluedroid/.sink_data"
};

void btif_a2dp_control_init(tUIPC_CH_ID ctrl_id, tUIPC_CH_ID data_id) {
  if (a2dp_uipc == nullptr)
    a2dp_uipc = UIPC_Init();

  UIPC_Open(*a2dp_uipc, ctrl_id, btif_a2dp_ctrl_cb, a2dp_uipc_path[ctrl_id]);
  UIPC_Open(*a2dp_uipc, data_id, btif_a2dp_data_cb, a2dp_uipc_path[data_id]);
}

void btif_a2dp_control_cleanup(tUIPC_CH_ID data_id) {
  /* This calls blocks until UIPC is fully closed */
  if (a2dp_uipc != nullptr) {
      UIPC_Close(*a2dp_uipc, data_id);
  }
}

static void btif_a2dp_recv_ctrl_data(tUIPC_CH_ID ch_id) {
  tA2DP_CTRL_CMD cmd = A2DP_CTRL_CMD_NONE;
  int n;

  uint8_t read_cmd = 0; /* The read command size is one octet */
  n = UIPC_Read(*a2dp_uipc, ch_id, NULL, &read_cmd, 1);
  cmd = static_cast<tA2DP_CTRL_CMD>(read_cmd);

  /* detach on ctrl channel means audioflinger process was terminated */
  if (n == 0) {
    APPL_TRACE_WARNING("%s: CTRL CH DETACHED", __func__);
    UIPC_Close(*a2dp_uipc, ch_id);
    return;
  }

  // Don't log A2DP_CTRL_GET_PRESENTATION_POSITION by default, because it
  // could be very chatty when audio is streaming.
  if (cmd == A2DP_CTRL_GET_PRESENTATION_POSITION) {
    APPL_TRACE_DEBUG("%s: a2dp-ctrl-cmd : %s", __func__,
                     audio_a2dp_hw_dump_ctrl_event(cmd));
  } else {
    APPL_TRACE_WARNING("%s: a2dp-ctrl-cmd : %s", __func__,
                       audio_a2dp_hw_dump_ctrl_event(cmd));
  }

  a2dp_cmd_pending = cmd;
  switch (cmd) {
    case A2DP_CTRL_CMD_CHECK_READY:
      if (btif_a2dp_source_media_task_is_shutting_down()) {
        APPL_TRACE_WARNING("%s: A2DP command %s while media task shutting down",
                           __func__, audio_a2dp_hw_dump_ctrl_event(cmd));
        btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_FAILURE);
        return;
      }

#ifdef CONFIG_FLUORIDE_A2DP_SINK_FFMPEG
      btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_SUCCESS);
#else
      /* check whether AV is ready to setup A2DP datapath */
      if (btif_av_stream_ready() || btif_av_stream_started_ready()) {
          btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_SUCCESS);
      } else {
          APPL_TRACE_WARNING("%s: A2DP command %s while AV stream is not ready",
                  __func__, audio_a2dp_hw_dump_ctrl_event(cmd));
          btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_FAILURE);
      }
#endif
      break;

    case A2DP_CTRL_CMD_START:
      /*
       * Don't send START request to stack while we are in a call.
       * Some headsets such as "Sony MW600", don't allow AVDTP START
       * while in a call, and respond with BAD_STATE.
       */
      if (!bluetooth::headset::IsCallIdle()) {
        APPL_TRACE_WARNING("%s: A2DP command %s while call state is busy",
                           __func__, audio_a2dp_hw_dump_ctrl_event(cmd));
        btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_INCALL_FAILURE);
        break;
      }

      if (btif_a2dp_source_is_streaming()) {
        APPL_TRACE_WARNING("%s: A2DP command %s while source is streaming",
                           __func__, audio_a2dp_hw_dump_ctrl_event(cmd));
        btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_SUCCESS);
        break;
      }

#ifdef CONFIG_FLUORIDE_A2DP_SINK_FFMPEG
      btif_a2dp_sink_enable_audio_send();
#endif
      if (btif_av_stream_ready()) {
        /* Setup audio data channel listener */
        if (ch_id == UIPC_CH_ID_AV_SINK_CTRL)
          UIPC_Open(*a2dp_uipc, UIPC_CH_ID_AV_SINK_AUDIO, btif_a2dp_data_cb,
              a2dp_uipc_path[UIPC_CH_ID_AV_SINK_AUDIO]);
        else
          UIPC_Open(*a2dp_uipc, UIPC_CH_ID_AV_SOURCE_AUDIO, btif_a2dp_data_cb,
              a2dp_uipc_path[UIPC_CH_ID_AV_SOURCE_AUDIO]);

        /*
         * Post start event and wait for audio path to open.
         * If we are the source, the ACK will be sent after the start
         * procedure is completed, othewise send it now.
         */
        btif_av_stream_start();
        if (btif_av_get_peer_sep() == AVDT_TSEP_SRC)
          btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_SUCCESS);
        break;
      }

      if (btif_av_stream_started_ready()) {
        /*
         * Already started, setup audio data channel listener and ACK
         * back immediately.
         */
        if (ch_id == UIPC_CH_ID_AV_SINK_CTRL)
          UIPC_Open(*a2dp_uipc, UIPC_CH_ID_AV_SINK_AUDIO, btif_a2dp_data_cb,
              a2dp_uipc_path[UIPC_CH_ID_AV_SINK_AUDIO]);
        else
          UIPC_Open(*a2dp_uipc, UIPC_CH_ID_AV_SOURCE_AUDIO, btif_a2dp_data_cb,
              a2dp_uipc_path[UIPC_CH_ID_AV_SOURCE_AUDIO]);
        btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_SUCCESS);
        break;
      }
      APPL_TRACE_WARNING("%s: A2DP command %s while AV stream is not ready",
                         __func__, audio_a2dp_hw_dump_ctrl_event(cmd));
      btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_FAILURE);
      break;

    case A2DP_CTRL_CMD_STOP:
#ifdef CONFIG_FLUORIDE_A2DP_SINK_FFMPEG
      btif_a2dp_sink_disable_audio_send();
#endif
      if (btif_av_get_peer_sep() == AVDT_TSEP_SNK &&
          !btif_a2dp_source_is_streaming()) {
        /* We are already stopped, just ack back */
        btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_SUCCESS);
        break;
      }
      btif_av_stream_stop(RawAddress::kEmpty);
      btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_SUCCESS);
      break;

    case A2DP_CTRL_CMD_SUSPEND:
      /* Local suspend */
      if (btif_av_stream_started_ready()) {
        btif_av_stream_suspend();
        break;
      }
      /* If we are not in started state, just ack back ok and let
       * audioflinger close the channel. This can happen if we are
       * remotely suspended, clear REMOTE SUSPEND flag.
       */
      btif_av_clear_remote_suspend_flag();
      btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_SUCCESS);
      break;

    case A2DP_CTRL_GET_INPUT_AUDIO_CONFIG: {
      tA2DP_CODEC_TYPE codec_type = btif_a2dp_sink_get_codec_type();
      tA2DP_SAMPLE_RATE sample_rate = btif_a2dp_sink_get_sample_rate();
      tA2DP_CHANNEL_COUNT channel_count = btif_a2dp_sink_get_channel_count();

      btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_SUCCESS);
      UIPC_Send(*a2dp_uipc, ch_id, 0,
                reinterpret_cast<uint8_t*>(&sample_rate),
                sizeof(tA2DP_SAMPLE_RATE));
      UIPC_Send(*a2dp_uipc, ch_id, 0, &channel_count,
                sizeof(tA2DP_CHANNEL_COUNT));
      UIPC_Send(*a2dp_uipc, ch_id, 0, (const uint8_t*)&codec_type,
                sizeof(tA2DP_CODEC_TYPE));
      break;
    }

    case A2DP_CTRL_GET_OUTPUT_AUDIO_CONFIG: {
      btav_a2dp_codec_config_t codec_config;
      codec_config.codec_type = BTAV_A2DP_CODEC_INDEX_SOURCE_MIN;
      codec_config.sample_rate = BTAV_A2DP_CODEC_SAMPLE_RATE_NONE;
      codec_config.bits_per_sample = BTAV_A2DP_CODEC_BITS_PER_SAMPLE_NONE;
      codec_config.channel_mode = BTAV_A2DP_CODEC_CHANNEL_MODE_NONE;

      A2dpCodecConfig* current_codec = bta_av_get_a2dp_current_codec();
      if (current_codec == nullptr) {
        btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_FAILURE);
        break;
      }
      codec_config = current_codec->getCodecConfig();
      uint32_t bit_rate = current_codec->getTrackBitRate();
      btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_SUCCESS);
      // Send the current codec config
      UIPC_Send(*a2dp_uipc, ch_id, 0,
                reinterpret_cast<const uint8_t*>(&codec_config.codec_type),
                sizeof(btav_a2dp_codec_sample_rate_t));
      UIPC_Send(*a2dp_uipc, ch_id, 0,
                reinterpret_cast<const uint8_t*>(&codec_config.sample_rate),
                sizeof(btav_a2dp_codec_sample_rate_t));
      UIPC_Send(*a2dp_uipc, ch_id, 0,
                reinterpret_cast<const uint8_t*>(&codec_config.bits_per_sample),
                sizeof(btav_a2dp_codec_bits_per_sample_t));
      UIPC_Send(*a2dp_uipc, ch_id, 0,
                reinterpret_cast<const uint8_t*>(&codec_config.channel_mode),
                sizeof(btav_a2dp_codec_channel_mode_t));
      UIPC_Send(*a2dp_uipc, ch_id, 0,
                reinterpret_cast<const uint8_t*>(&bit_rate),
                sizeof(uint32_t));
      break;
    }

    case A2DP_CTRL_SET_OUTPUT_AUDIO_CONFIG: {
      btav_a2dp_codec_config_t codec_config;
      codec_config.sample_rate = BTAV_A2DP_CODEC_SAMPLE_RATE_NONE;
      codec_config.bits_per_sample = BTAV_A2DP_CODEC_BITS_PER_SAMPLE_NONE;
      codec_config.channel_mode = BTAV_A2DP_CODEC_CHANNEL_MODE_NONE;

      btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_SUCCESS);
      // Send the current codec config
      if (UIPC_Read(*a2dp_uipc, ch_id, 0,
                    reinterpret_cast<uint8_t*>(&codec_config.sample_rate),
                    sizeof(btav_a2dp_codec_sample_rate_t)) !=
          sizeof(btav_a2dp_codec_sample_rate_t)) {
        APPL_TRACE_ERROR("%s: Error reading sample rate from audio HAL",
                         __func__);
        break;
      }
      if (UIPC_Read(*a2dp_uipc, ch_id, 0,
                    reinterpret_cast<uint8_t*>(&codec_config.bits_per_sample),
                    sizeof(btav_a2dp_codec_bits_per_sample_t)) !=
          sizeof(btav_a2dp_codec_bits_per_sample_t)) {
        APPL_TRACE_ERROR("%s: Error reading bits per sample from audio HAL",
                         __func__);
        break;
      }
      if (UIPC_Read(*a2dp_uipc, ch_id, 0,
                    reinterpret_cast<uint8_t*>(&codec_config.channel_mode),
                    sizeof(btav_a2dp_codec_channel_mode_t)) !=
          sizeof(btav_a2dp_codec_channel_mode_t)) {
        APPL_TRACE_ERROR("%s: Error reading channel mode from audio HAL",
                         __func__);
        break;
      }
      APPL_TRACE_DEBUG(
          "%s: A2DP_CTRL_SET_OUTPUT_AUDIO_CONFIG: "
          "sample_rate=0x%x bits_per_sample=0x%x "
          "channel_mode=0x%x",
          __func__, codec_config.sample_rate, codec_config.bits_per_sample,
          codec_config.channel_mode);
      btif_a2dp_source_feeding_update_req(codec_config);
      break;
    }

    case A2DP_CTRL_CMD_OFFLOAD_START:
      btif_av_stream_start_offload();
      break;

    case A2DP_CTRL_GET_PRESENTATION_POSITION: {
      btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_SUCCESS);

      UIPC_Send(*a2dp_uipc, ch_id, 0,
                (uint8_t*)&(delay_report_stats.total_bytes_read),
                sizeof(uint64_t));
      UIPC_Send(*a2dp_uipc, ch_id, 0,
                (uint8_t*)&(delay_report_stats.audio_delay), sizeof(uint16_t));

      uint32_t seconds = delay_report_stats.timestamp.tv_sec;
      UIPC_Send(*a2dp_uipc, ch_id, 0, (uint8_t*)&seconds,
                sizeof(seconds));

      uint32_t nsec = delay_report_stats.timestamp.tv_nsec;
      UIPC_Send(*a2dp_uipc, ch_id, 0, (uint8_t*)&nsec,
                sizeof(nsec));
      break;
    }
    default:
      APPL_TRACE_ERROR("%s: UNSUPPORTED CMD (%d)", __func__, cmd);
      btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_FAILURE);
      break;
  }

  // Don't log A2DP_CTRL_GET_PRESENTATION_POSITION by default, because it
  // could be very chatty when audio is streaming.
  if (cmd == A2DP_CTRL_GET_PRESENTATION_POSITION) {
    APPL_TRACE_DEBUG("%s: a2dp-ctrl-cmd : %s DONE", __func__,
                     audio_a2dp_hw_dump_ctrl_event(cmd));
  } else {
    APPL_TRACE_WARNING("%s: a2dp-ctrl-cmd : %s DONE", __func__,
                       audio_a2dp_hw_dump_ctrl_event(cmd));
  }
}

static void btif_a2dp_ctrl_cb(tUIPC_CH_ID ch_id,
                              tUIPC_EVENT event) {
  // Don't log UIPC_RX_DATA_READY_EVT by default, because it
  // could be very chatty when audio is streaming.
  if (event == UIPC_RX_DATA_READY_EVT) {
    APPL_TRACE_DEBUG("%s: A2DP-CTRL-CHANNEL EVENT %s", __func__,
                     dump_uipc_event(event));
  } else {
    APPL_TRACE_WARNING("%s: A2DP-CTRL-CHANNEL EVENT %s", __func__,
                       dump_uipc_event(event));
  }

  switch (event) {
    case UIPC_OPEN_EVT:
      break;

    case UIPC_CLOSE_EVT:
      /* restart ctrl server unless we are shutting down */
      if (btif_a2dp_source_media_task_is_running() || btif_a2dp_sink_media_task_is_running())
        UIPC_Open(*a2dp_uipc, ch_id, btif_a2dp_ctrl_cb,
                  A2DP_CTRL_PATH);
      break;

    case UIPC_RX_DATA_READY_EVT:
      btif_a2dp_recv_ctrl_data(ch_id);
      break;

    default:
      APPL_TRACE_ERROR("%s: ### A2DP-CTRL-CHANNEL EVENT %d NOT HANDLED ###",
                       __func__, event);
      break;
  }
}

static void btif_a2dp_data_cb(tUIPC_CH_ID ch_id,
                              tUIPC_EVENT event) {
  APPL_TRACE_WARNING("%s: BTIF MEDIA (A2DP-DATA) EVENT %s", __func__,
                     dump_uipc_event(event));

  switch (event) {
    case UIPC_OPEN_EVT:
      /*
       * Read directly from media task from here on (keep callback for
       * connection events.
       */
      if (ch_id == UIPC_CH_ID_AV_SINK_CTRL) {
        UIPC_Ioctl(*a2dp_uipc, UIPC_CH_ID_AV_SINK_AUDIO,
            UIPC_REG_REMOVE_ACTIVE_READSET, NULL);
        UIPC_Ioctl(*a2dp_uipc, UIPC_CH_ID_AV_SINK_AUDIO, UIPC_SET_READ_POLL_TMO,
            reinterpret_cast<void*>(A2DP_DATA_READ_POLL_MS));
      } else {
        UIPC_Ioctl(*a2dp_uipc, UIPC_CH_ID_AV_SOURCE_AUDIO,
            UIPC_REG_REMOVE_ACTIVE_READSET, NULL);
        UIPC_Ioctl(*a2dp_uipc, UIPC_CH_ID_AV_SOURCE_AUDIO, UIPC_SET_READ_POLL_TMO,
            reinterpret_cast<void*>(A2DP_DATA_READ_POLL_MS));
      }

      /* ACK back when media task is fully started */
      break;

    case UIPC_CLOSE_EVT:
      APPL_TRACE_EVENT("%s: ## AUDIO PATH DETACHED ##", __func__);
#ifdef CONFIG_FLUORIDE_A2DP_SINK_FFMPEG
      btif_a2dp_sink_disable_audio_send();
#endif
      btif_a2dp_command_ack(ch_id, A2DP_CTRL_ACK_SUCCESS);
      /*
       * Send stop request only if we are actively streaming and haven't
       * received a stop request. Potentially, the audioflinger detached
       * abnormally.
       */
      if (btif_a2dp_source_is_streaming()) {
        /* Post stop event and wait for audio path to stop */
        btif_av_stream_stop(RawAddress::kEmpty);
      }
      break;

    default:
      APPL_TRACE_ERROR("%s: ### A2DP-DATA EVENT %d NOT HANDLED ###", __func__,
                       event);
      break;
  }
}

void btif_a2dp_command_ack(tUIPC_CH_ID ch_id, tA2DP_CTRL_ACK status) {
  uint8_t ack = status;

  // Don't log A2DP_CTRL_GET_PRESENTATION_POSITION by default, because it
  // could be very chatty when audio is streaming.
  if (a2dp_cmd_pending == A2DP_CTRL_GET_PRESENTATION_POSITION) {
    APPL_TRACE_DEBUG("%s: ## a2dp ack : %s, status %d ##", __func__,
                     audio_a2dp_hw_dump_ctrl_event(a2dp_cmd_pending), status);
  } else {
    APPL_TRACE_WARNING("%s: ## a2dp ack : %s, status %d ##", __func__,
                       audio_a2dp_hw_dump_ctrl_event(a2dp_cmd_pending), status);
  }

  /* Sanity check */
  if (a2dp_cmd_pending == A2DP_CTRL_CMD_NONE) {
    APPL_TRACE_ERROR("%s: warning : no command pending, ignore ack", __func__);
    return;
  }

  /* Clear pending */
  a2dp_cmd_pending = A2DP_CTRL_CMD_NONE;

  /* Acknowledge start request */
  if (a2dp_uipc != nullptr) {
    UIPC_Send(*a2dp_uipc, ch_id, 0, &ack, sizeof(ack));
  }
}

void btif_a2dp_control_log_bytes_read(uint32_t bytes_read) {
  delay_report_stats.total_bytes_read += bytes_read;
  clock_gettime(CLOCK_MONOTONIC, &delay_report_stats.timestamp);
}

void btif_a2dp_control_set_audio_delay(uint16_t delay) {
  APPL_TRACE_DEBUG("%s: DELAY: %.1f ms", __func__, (float)delay / 10);
  delay_report_stats.audio_delay = delay;
}

void btif_a2dp_control_reset_audio_delay(void) {
  APPL_TRACE_DEBUG("%s", __func__);
  delay_report_stats.audio_delay = 0;
  delay_report_stats.total_bytes_read = 0;
  delay_report_stats.timestamp = {};
}
