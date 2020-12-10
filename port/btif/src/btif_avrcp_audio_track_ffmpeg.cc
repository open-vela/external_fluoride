/*
 * Copyright 2015 The Android Open Source Project
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

#include <atomic>
#include <cstdio>
#include <base/bind.h>

#include "uipc.h"
#include "btif_avrcp_audio_track.h"

typedef struct BtifAvrcpAudioTrack {
  int ffmpegReady;
} BtifAvrcpAudioTrack;

BtifAvrcpAudioTrack g_track = {};

extern std::unique_ptr<tUIPC_STATE> a2dp_uipc;

void BtifAvrcpAudioTrackDelete(void* handle)
{
  UIPC_Close(*a2dp_uipc, UIPC_CH_ID_AV_AUDIO);
}

void BtifAvrcpSetAudioTrackGain(void* handle, float gain)
{
}

void *BtifAvrcpAudioTrackCreate(int trackFreq, int bitsPerSample, int channelCount)
{
  int ret;

  BtifAvrcpAudioTrack *priv = (BtifAvrcpAudioTrack *)&g_track;
  priv->ffmpegReady = 0;

  return priv;
}

void BtifAvrcpAudioTrackStart(void* handle)
{
}

void BtifAvrcpEnableAudioSend(void* handle)
{
  BtifAvrcpAudioTrack *priv = (BtifAvrcpAudioTrack *)&g_track;
  priv->ffmpegReady = 1;
}

void BtifAvrcpDisableAudioSend(void* handle)
{
  BtifAvrcpAudioTrack *priv = (BtifAvrcpAudioTrack *)&g_track;
  priv->ffmpegReady = 0;
}

void BtifAvrcpAudioTrackStop(void* handle)
{
}

void BtifAvrcpAudioTrackPause(void* handle)
{
}

int BtifAvrcpAudioTrackWriteData(void *handle, void *audioBuffer, int bufferLength)
{
  BtifAvrcpAudioTrack *priv = (BtifAvrcpAudioTrack *)&g_track;

  if (priv->ffmpegReady > 0)
  {
    UIPC_Send(*a2dp_uipc, UIPC_CH_ID_AV_AUDIO, 0, (const uint8_t*)audioBuffer, bufferLength);
  }
  return bufferLength;
}
