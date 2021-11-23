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

enum bt_le_adv_state
{
  ADV_STATE_DISABLED,
  ADV_STATE_TIMEOUT,
  ADV_STATE_ENABLED,
};

static bt_le_adv_state g_bt_le_adv_state;

static void bt_le_adv_set_state(bt_le_adv_state state)
{
  struct fluoride_s* flrd = fluoride_interface_get();

  pthread_mutex_lock(&flrd->mutex);
  g_bt_le_adv_state = state;
  pthread_cond_broadcast(&flrd->cond);
  pthread_mutex_unlock(&flrd->mutex);
}

static void bt_le_adv_enable_cb(uint8_t rid, bool enable, uint8_t status)
{
  bt_le_adv_set_state(ADV_STATE_ENABLED);
}

static void bt_le_adv_disable_cb(uint8_t rid, bool enable, uint8_t status)
{
  bt_le_adv_set_state(ADV_STATE_DISABLED);
}

static void bt_le_adv_set_timeout_cb(uint8_t advertiser_id, uint8_t status)
{
  bt_le_adv_set_state(ADV_STATE_TIMEOUT);
}

static void bt_le_adv_set_started_cb(int reg_id, uint8_t advertiser_id,
                                     int8_t tx_power, uint8_t status)
{
  struct fluoride_s *flrd = fluoride_interface_get();

  flrd->aid = advertiser_id;

  flrd->gatt->advertiser->Enable(advertiser_id, true,
      base::Bind(&bt_le_adv_enable_cb, advertiser_id, true), 0, 0,
      base::Bind(&bt_le_adv_enable_cb, advertiser_id, false));
}

extern "C" {

  int bt_le_adv_start(const struct bt_le_adv_param *param,
      const struct bt_data *ad, size_t ad_len,
      const struct bt_data *sd, size_t sd_len)
  {
    struct fluoride_s *flrd = fluoride_interface_get();
    PeriodicAdvertisingParameters pparams = {};
    AdvertiseParameters params = {};
    std::vector<uint8_t> adata;
    std::vector<uint8_t> sdata;
    std::vector<uint8_t> pdata;
    uint16_t properties = 0;
    int reg_id = -2;
    size_t i;
    int ret;

    if (!flrd)
      return -EPERM;
    else if (!param)
      return -EINVAL;

    if (param->options & BT_LE_ADV_OPT_CONNECTABLE)
      properties |= 0x01;
    if (param->options & BT_LE_ADV_OPT_SCANNABLE)
      properties |= 0x02;
    if (param->options & BT_LE_ADV_OPT_DIR_MODE_LOW_DUTY)
      properties |= 0x04;
    if (param->options & BT_LE_ADV_OPT_NOTIFY_SCAN_REQ)
      properties |= 0x08;
    if (param->options & BT_LE_ADV_OPT_ANONYMOUS)
      properties |= 0x20;
    if (param->options & BT_LE_ADV_OPT_USE_TX_POWER)
      properties |= 0x40;

    if (!properties)
      return -EINVAL;

    properties |= 0x10; /* legacy */

    params.advertising_event_properties     = properties;
    params.min_interval                     = param->interval_min;
    params.max_interval                     = param->interval_max;
    params.channel_map                      = 0x07; /* all channels */
    params.tx_power                         = -15;
    params.primary_advertising_phy          = 1;
    params.secondary_advertising_phy        = 1;
    params.scan_request_notification_enable = false;

    pparams.enable                          = false;
    pparams.min_interval                    = 80;
    pparams.max_interval                    = 80 + 16; /* 20ms difference betwen min and max */

    for (i = 0; i < ad_len; i++) {
      adata.push_back(ad[i].data_len + 1);
      adata.push_back(ad[i].type);
      adata.insert(adata.end(), ad[i].data, ad[i].data + ad[i].data_len);
    }

    for (i = 0; i < sd_len; i++) {
      sdata.push_back(sd[i].data_len + 1);
      sdata.push_back(sd[i].type);
      sdata.insert(sdata.end(), sd[i].data, sd[i].data + sd[i].data_len);
    }

    flrd->gatt->advertiser->StartAdvertisingSet(reg_id,
        base::Bind(&bt_le_adv_set_started_cb, reg_id), params, adata,
        sdata, pparams, pdata, 0, 0, base::Bind(bt_le_adv_set_timeout_cb));

    pthread_mutex_lock(&flrd->mutex);
    g_bt_le_adv_state = ADV_STATE_DISABLED;
    while (g_bt_le_adv_state == ADV_STATE_DISABLED)
      pthread_cond_wait(&flrd->cond, &flrd->mutex);
    pthread_mutex_unlock(&flrd->mutex);

    return (g_bt_le_adv_state == ADV_STATE_ENABLED) ? 0 : -1;
  }

  int bt_le_adv_stop(void)
  {
    struct fluoride_s *flrd = fluoride_interface_get();
    if (!flrd)
      return -EPERM;

    flrd->gatt->advertiser->Enable(flrd->aid, false,
        base::Bind(&bt_le_adv_disable_cb, flrd->aid, true), 0, 0,
        base::Bind(&bt_le_adv_disable_cb, flrd->aid, false));

    pthread_mutex_lock(&flrd->mutex);
    g_bt_le_adv_state = ADV_STATE_ENABLED;
    while (g_bt_le_adv_state == ADV_STATE_ENABLED)
      pthread_cond_wait(&flrd->cond, &flrd->mutex);
    pthread_mutex_unlock(&flrd->mutex);

    flrd->gatt->advertiser->Unregister(flrd->aid);

    return (g_bt_le_adv_state == ADV_STATE_DISABLED) ? 0 : -1;
  }

}
