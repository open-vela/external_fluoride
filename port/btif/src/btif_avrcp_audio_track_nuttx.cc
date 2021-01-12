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

#include <nuttx/config.h>
#include <nuttx/audio/audio.h>
#include <sys/ioctl.h>
#include <mqueue.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>

#include "btif_avrcp_audio_track.h"

#define MAX_RETRY       5

/****************************************************************************
 * Public Functions
 ****************************************************************************/

typedef struct {
  struct ap_buffer_s **abuffer;
  struct ap_buffer_s *lbuffer;
  uint8_t     *cache;
  int         cache_size;
  int         free_abuffer;
  char        mqname[16];
  mqd_t       mq;
  int         fd;
  int         periods;
  int         period_bytes;
  bool        pause;
} BtifAvrcpAudioTrack;

static BtifAvrcpAudioTrack g_track = { .fd = -1, };

static void BtifAvrcpAudioTrackDrainMessage(void* handle)
{
  BtifAvrcpAudioTrack *priv = (BtifAvrcpAudioTrack *)handle;
  struct audio_msg_s msg;
  int ret;

  if (!priv || priv->fd < 0)
    return;

  while (1) {
    struct mq_attr stat;
    int ret;

    ret = mq_getattr(priv->mq, &stat);
    if (ret < 0)
      return;

    if (!stat.mq_curmsgs)
      break;

    mq_receive(priv->mq, (char *)&msg, sizeof(msg), NULL);
  }
}

static int BtifAvrcpAudioTrackGetBuffer(void *handle, struct ap_buffer_s **abuffer)
{
  BtifAvrcpAudioTrack *priv = (BtifAvrcpAudioTrack *)handle;
  struct audio_msg_s msg;
  struct mq_attr stat;
  ssize_t size;
  int ret;

  if (!priv || priv->fd < 0)
    return -1;

  *abuffer = NULL;

  if (priv->free_abuffer) {
    *abuffer = priv->abuffer[--priv->free_abuffer];
    return 0;
  }

  ret = mq_getattr(priv->mq, &stat);
  if (ret < 0)
    return ret;

  size = mq_receive(priv->mq, (char *)&msg, sizeof(msg), NULL);
  if (size != sizeof(msg))
    return -EIO;

  if (msg.msg_id == AUDIO_MSG_DEQUEUE) {
    *abuffer = (struct ap_buffer_s *)msg.u.ptr;
  } else if (msg.msg_id == AUDIO_MSG_COMPLETE) {
  } else
    return -EIO;

  return 0;
}

void BtifAvrcpSetAudioTrackGain(void* handle, float gain)
{
}

void BtifAvrcpAudioTrackDelete(void* handle)
{
  BtifAvrcpAudioTrack *priv = (BtifAvrcpAudioTrack *)handle;
  int i;

  if (!priv || priv->fd < 0)
    return;

  if (priv->mq) {

    ioctl(priv->fd, AUDIOIOC_UNREGISTERMQ, NULL);

    mq_close(priv->mq);
    priv->mq = NULL;

    mq_unlink(priv->mqname);
  }

  ioctl(priv->fd, AUDIOIOC_RELEASE, 0);

  if (priv->abuffer) {
    for (i = 0; i < priv->periods; i++) {
      struct audio_buf_desc_s buf_desc;

      buf_desc.u.buffer = priv->abuffer[i];
      ioctl(priv->fd, AUDIOIOC_FREEBUFFER, (unsigned long)&buf_desc);
    }

    free(priv->abuffer);
    priv->abuffer = NULL;
  }

  if (priv->cache) {
    free(priv->cache);
    priv->cache = NULL;
    priv->cache_size = 0;
  }

  close(priv->fd);
  priv->fd = -1;
}

void *BtifAvrcpAudioTrackCreate(int trackFreq, int bitsPerSample, int channelCount)
{
  BtifAvrcpAudioTrack *priv = &g_track;
  struct audio_caps_desc_s caps_desc = {0};
  struct audio_buf_desc_s buf_desc;
  struct ap_buffer_info_s buf_info;
  int ret;
  int i;

  struct mq_attr attr = {
    .mq_maxmsg  = 8,
    .mq_msgsize = sizeof(struct audio_msg_s),
  };

  /* open device */
  ret = open(CONFIG_FLUORIDE_A2DP_SINK_DEVNAME, O_RDWR | O_CLOEXEC);
  if (ret < 0)
    return NULL;

  priv->fd = ret;
  priv->pause = 0;
  priv->cache_size = 0;
  priv->lbuffer = NULL;

  /* configure */
  ret = ioctl(priv->fd, AUDIOIOC_RESERVE, 0);
  if (ret < 0)
    goto out;

  caps_desc.caps.ac_len            = sizeof(struct audio_caps_s);
  caps_desc.caps.ac_type           = AUDIO_TYPE_OUTPUT;
  caps_desc.caps.ac_channels       = channelCount;
  caps_desc.caps.ac_controls.hw[0] = trackFreq;
  caps_desc.caps.ac_controls.b[3]  = trackFreq >> 16;
  caps_desc.caps.ac_controls.b[2]  = bitsPerSample;
  ret = ioctl(priv->fd, AUDIOIOC_CONFIGURE, (unsigned long)&caps_desc);
  if (ret < 0)
    goto out;

  /* create message queue */
  snprintf(priv->mqname, sizeof(priv->mqname), "/tmp/%0lx",
      (unsigned long)((uintptr_t)priv));
  priv->mq = mq_open(priv->mqname, O_RDWR | O_CREAT, 0644, &attr);
  if (priv->mq < 0)
    goto out;

  ret = ioctl(priv->fd, AUDIOIOC_REGISTERMQ, (unsigned long)priv->mq);
  if (ret < 0)
    goto out;

  ret = ioctl(priv->fd, AUDIOIOC_GETBUFFERINFO, (unsigned long)&buf_info);
  if (!ret) {
    priv->periods      = buf_info.nbuffers;
    priv->period_bytes = buf_info.buffer_size;
  } else {
    priv->periods      = CONFIG_AUDIO_NUM_BUFFERS;
    priv->period_bytes = CONFIG_AUDIO_BUFFER_NUMBYTES;
  }

  priv->abuffer = (struct ap_buffer_s **)malloc(priv->periods * sizeof(struct ap_buffer_s *));
  if (!priv->abuffer) {
    ret = -ENOMEM;
    goto out;
  }

  for (i = 0; i < priv->periods; i++) {
    buf_desc.numbytes  = priv->period_bytes;
    buf_desc.u.pbuffer = &priv->abuffer[i];
    ret = ioctl(priv->fd, AUDIOIOC_ALLOCBUFFER, (unsigned long)&buf_desc);
    if (ret != sizeof(buf_desc)) {
      goto out;
    }
  }

  /* create local buffer */
  priv->cache = (uint8_t *)malloc(priv->period_bytes);
  if (!priv->cache)
    ret = -ENOMEM;

out:
  if (ret < 0) {
    BtifAvrcpAudioTrackDelete(priv);
    return NULL;
  }

  return priv;
}

void BtifAvrcpAudioTrackStart(void* handle)
{
  BtifAvrcpAudioTrack *priv = (BtifAvrcpAudioTrack *)handle;
  struct audio_buf_desc_s buf_desc;
  int ret;
  int i;

  if (!priv || priv->fd < 0)
    return;

  BtifAvrcpAudioTrackDrainMessage(handle);

  for (i = 0; i < priv->periods; i++) {
    buf_desc.u.buffer = priv->abuffer[i];
    ret = ioctl(priv->fd, AUDIOIOC_ENQUEUEBUFFER, (unsigned long)&buf_desc);
    if (ret < 0) {
      return;
    }
  }

  if (priv->pause)
    ioctl(priv->fd, AUDIOIOC_RESUME, 0);
  else
    ioctl(priv->fd, AUDIOIOC_START, 0);

  return;
}

void BtifAvrcpAudioTrackStop(void* handle)
{
  BtifAvrcpAudioTrack *priv = (BtifAvrcpAudioTrack *)handle;

  if (!priv || priv->fd < 0)
    return;

  BtifAvrcpAudioTrackDrainMessage(handle);

  ioctl(priv->fd, AUDIOIOC_STOP, 0);
  priv->pause = false;
  priv->lbuffer = NULL;
  priv->cache_size = 0;
}

void BtifAvrcpAudioTrackPause(void* handle)
{
  BtifAvrcpAudioTrack *priv = (BtifAvrcpAudioTrack *)handle;

  if (!priv || priv->fd < 0)
    return;

  BtifAvrcpAudioTrackDrainMessage(handle);

  ioctl(priv->fd, AUDIOIOC_PAUSE, 0);
  priv->pause = true;
  priv->lbuffer = NULL;
  priv->cache_size = 0;
}

int BtifAvrcpAudioTrackWriteData(void *handle, void *audioBuffer, int bufferLength)
{
  BtifAvrcpAudioTrack *priv = (BtifAvrcpAudioTrack *)handle;
  struct audio_buf_desc_s desc;
  struct ap_buffer_s *abuffer;
  int expect, avail;
  int i = 0;
  int ret;

  if (!priv || priv->fd < 0)
    return -1;

  if (!priv->lbuffer) {
    while (1) {
      ret = BtifAvrcpAudioTrackGetBuffer(priv, &priv->lbuffer);
      if (ret == -EAGAIN || !priv->lbuffer) {
        if (i++ > MAX_RETRY) {
          return ret;
        }
        usleep(10);
      } else if (ret)
        return ret;
      else
        break;
    }
    priv->lbuffer->nbytes = 0;
  }

  abuffer = priv->lbuffer;

  if (priv->cache_size) {
    memcpy(abuffer->samp, priv->cache, priv->cache_size);
    abuffer->nbytes = priv->cache_size;
    priv->cache_size = 0;
  }

  avail = abuffer->nmaxbytes - abuffer->nbytes;

  if (avail >= bufferLength) {
    memcpy(abuffer->samp + abuffer->nbytes, audioBuffer, bufferLength);
    abuffer->nbytes += bufferLength;
  } else {
    memcpy(abuffer->samp + abuffer->nbytes, audioBuffer, avail);
    abuffer->nbytes += avail;
    priv->cache_size = bufferLength - avail;
    memcpy(priv->cache, (uint8_t *)audioBuffer + avail, priv->cache_size);
  }

  if (abuffer->nmaxbytes != abuffer->nbytes)
    return 0;

  desc.u.buffer = abuffer;
  priv->lbuffer = NULL;

  return ioctl(priv->fd, AUDIOIOC_ENQUEUEBUFFER, (unsigned long)&desc);
}
