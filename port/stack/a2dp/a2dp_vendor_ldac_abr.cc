/*
 * Copyright 2017 The Android Open Source Project
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

#define LOG_TAG "a2dp_vendor_ldac_abr"

#include "a2dp_vendor_ldac_abr.h"

#include <inttypes.h>

bool A2DP_VendorLoadLdacAbr(void) { return true; }

void A2DP_VendorUnloadLdacAbr(void) { }

HANDLE_LDAC_ABR a2dp_ldac_abr_get_handle(void) {
  return ldac_ABR_get_handle();
}

void a2dp_ldac_abr_free_handle(HANDLE_LDAC_ABR hLdacAbr) {
  return ldac_ABR_free_handle(hLdacAbr);
}

int a2dp_ldac_abr_init(HANDLE_LDAC_ABR hLdacAbr, unsigned int interval_ms) {
  return ldac_ABR_Init(hLdacAbr, interval_ms);
}

int a2dp_ldac_abr_set_thresholds(HANDLE_LDAC_ABR hLdacAbr,
                                 unsigned int th_critical,
                                 unsigned int th_dangerous_trend,
                                 unsigned int th_4hqsq) {
  return ldac_ABR_set_thresholds(hLdacAbr, th_critical, th_dangerous_trend,
                                      th_4hqsq);
}

int a2dp_ldac_abr_proc(HANDLE_LDAC_BT hLdacParam, HANDLE_LDAC_ABR hLdacAbr,
                       size_t transmit_queue_length, unsigned int flag_enable) {
  return ldac_ABR_Proc(hLdacParam, hLdacAbr, transmit_queue_length,
                            flag_enable);
}
