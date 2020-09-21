/*
 * Copyright 2016 The Android Open Source Project
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

#define LOG_TAG "a2dp_vendor_ldac_decoder"
#define ATRACE_TAG ATRACE_TAG_AUDIO

#include "a2dp_vendor_ldac_decoder.h"

#ifndef OS_GENERIC
#include <cutils/trace.h>
#endif
#include <inttypes.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#include <ldacBT.h>
#include <ldacBT_bco_for_fluoride.h>

#include "a2dp_vendor.h"
#include "a2dp_vendor_ldac.h"
#include "bt_common.h"
#include "osi/include/log.h"
#include "osi/include/osi.h"

// offset
#if (BTA_AV_CO_CP_SCMS_T == TRUE)
#define A2DP_LDAC_OFFSET (AVDT_MEDIA_OFFSET + A2DP_LDAC_MPL_HDR_LEN + 1)
#else
#define A2DP_LDAC_OFFSET (AVDT_MEDIA_OFFSET + A2DP_LDAC_MPL_HDR_LEN)
#endif

typedef struct {
  uint32_t sample_rate;
  uint8_t channel_mode;
  uint8_t bits_per_sample;
  int pcm_wlength;
  LDACBT_SMPL_FMT_T pcm_fmt;
} tA2DP_LDAC_DECODER_PARAMS;

typedef struct {
  pthread_mutex_t mutex;
  bool use_SCMS_T;
  bool is_peer_edr;          // True if the peer device supports EDR
  bool peer_supports_3mbps;  // True if the peer device supports 3Mbps EDR
  uint16_t peer_mtu;         // MTU of the A2DP peer
  uint32_t timestamp;        // Timestamp for the A2DP frames

  HANDLE_LDAC_BCO ldac_handle_bco;
  bool has_ldac_handle;  // True if ldac_handle is valid
  unsigned char* decode_buf;
  decoded_data_callback_t decode_callback;
} tA2DP_LDAC_DECODER_CB;

static tA2DP_LDAC_DECODER_CB a2dp_ldac_decoder_cb;

bool A2DP_VendorLoadDecoderLdac(void) {

  // Initialize the control block
  memset(&a2dp_ldac_decoder_cb, 0, sizeof(a2dp_ldac_decoder_cb));

  pthread_mutex_init(&(a2dp_ldac_decoder_cb.mutex), NULL);

  return true;
}

void A2DP_VendorUnloadDecoderLdac(void) {
  // Cleanup any LDAC-related state
  if (a2dp_ldac_decoder_cb.has_ldac_handle)
    ldac_BCO_cleanup(a2dp_ldac_decoder_cb.ldac_handle_bco);
  pthread_mutex_destroy(&(a2dp_ldac_decoder_cb.mutex));
  memset(&a2dp_ldac_decoder_cb, 0, sizeof(a2dp_ldac_decoder_cb));
}

bool a2dp_vendor_ldac_decoder_init(decoded_data_callback_t decode_callback) {
  pthread_mutex_lock(&(a2dp_ldac_decoder_cb.mutex));

  if (a2dp_ldac_decoder_cb.has_ldac_handle)
    ldac_BCO_cleanup(a2dp_ldac_decoder_cb.ldac_handle_bco);

  a2dp_ldac_decoder_cb.ldac_handle_bco = ldac_BCO_init(decode_callback);
  a2dp_ldac_decoder_cb.has_ldac_handle =
      (a2dp_ldac_decoder_cb.ldac_handle_bco != NULL);

  pthread_mutex_unlock(&(a2dp_ldac_decoder_cb.mutex));
  return true;
}

void a2dp_vendor_ldac_decoder_cleanup(void) {
  pthread_mutex_lock(&(a2dp_ldac_decoder_cb.mutex));
  if (a2dp_ldac_decoder_cb.has_ldac_handle)
    ldac_BCO_cleanup(a2dp_ldac_decoder_cb.ldac_handle_bco);
  a2dp_ldac_decoder_cb.ldac_handle_bco = NULL;
  pthread_mutex_unlock(&(a2dp_ldac_decoder_cb.mutex));
}

bool a2dp_vendor_ldac_decoder_decode_packet(BT_HDR* p_buf) {
  pthread_mutex_lock(&(a2dp_ldac_decoder_cb.mutex));
  unsigned char* pBuffer =
      reinterpret_cast<unsigned char*>(p_buf->data + p_buf->offset);
  //  unsigned int bufferSize = p_buf->len;
  unsigned int bytesValid = p_buf->len;
  LDACBT_SMPL_FMT_T fmt;
  int bs_bytes, frame_number;

  fmt = LDACBT_SMPL_FMT_S32;
  frame_number = (int)pBuffer[0];
  bs_bytes = (int)bytesValid;
  bytesValid -= 1;
  LOG_DEBUG("%s:INPUT size : %d, frame : %d", __func__, bs_bytes, frame_number);

  if (a2dp_ldac_decoder_cb.has_ldac_handle)
    ldac_BCO_decode_packet(a2dp_ldac_decoder_cb.ldac_handle_bco, pBuffer,
                                bs_bytes);

  pthread_mutex_unlock(&(a2dp_ldac_decoder_cb.mutex));
  return true;
}

void a2dp_vendor_ldac_decoder_start(void) {
  pthread_mutex_lock(&(a2dp_ldac_decoder_cb.mutex));
  LOG_DEBUG("%s", __func__);
  if (a2dp_ldac_decoder_cb.has_ldac_handle)
    ldac_BCO_start(a2dp_ldac_decoder_cb.ldac_handle_bco);
  pthread_mutex_unlock(&(a2dp_ldac_decoder_cb.mutex));
}

void a2dp_vendor_ldac_decoder_suspend(void) {
  pthread_mutex_lock(&(a2dp_ldac_decoder_cb.mutex));
  LOG_DEBUG("%s", __func__);
  if (a2dp_ldac_decoder_cb.has_ldac_handle)
    ldac_BCO_suspend(a2dp_ldac_decoder_cb.ldac_handle_bco);
  pthread_mutex_unlock(&(a2dp_ldac_decoder_cb.mutex));
}

void a2dp_vendor_ldac_decoder_configure(const uint8_t* p_codec_info) {
  int32_t sample_rate;
  int32_t bits_per_sample;
  int32_t channel_mode;

  if (p_codec_info == NULL) {
    LOG_ERROR("%s: p_codec_info is NULL", __func__);
    return;
  }

  pthread_mutex_lock(&(a2dp_ldac_decoder_cb.mutex));
  sample_rate = A2DP_VendorGetTrackSampleRateLdac(p_codec_info);
  bits_per_sample = A2DP_VendorGetTrackBitsPerSampleLdac(p_codec_info);
  channel_mode = A2DP_VendorGetChannelModeCodeLdac(p_codec_info);

  LOG_DEBUG("%s , sample_rate=%d, bits_per_sample=%d, channel_mode=%d",
            __func__, sample_rate, bits_per_sample, channel_mode);

  if (a2dp_ldac_decoder_cb.has_ldac_handle)
    ldac_BCO_configure(a2dp_ldac_decoder_cb.ldac_handle_bco, sample_rate,
                            bits_per_sample, channel_mode);
  pthread_mutex_unlock(&(a2dp_ldac_decoder_cb.mutex));
}
