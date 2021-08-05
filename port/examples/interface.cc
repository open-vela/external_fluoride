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

static void adapter_state_changed(bt_state_t state)
{
  struct fluoride_s *flrd = fluoride_interface_get();

  if (state != BT_STATE_ON)
    return;

  pthread_mutex_lock(&flrd->mutex);
  flrd->state = state;

#ifdef CONFIG_FLUORIDE_BLE_ENABLED
  flrd->gatt  = (const btgatt_interface_t      *)bt_profile_gatt_init(flrd);
#endif
  flrd->ctrl  = (const btrc_ctrl_interface_t   *)bt_profile_avrcp_control_init(flrd);
  flrd->avrcp = (const btrc_interface_t        *)bt_profile_avrcp_init(flrd);
  flrd->sdp   = (const btsdp_interface_t       *)bt_profile_sdp_init(flrd);
  flrd->sink  = (const btav_sink_interface_t   *)bt_profile_a2dp_sink_init(flrd);
  flrd->hfc   = (const bthf_client_interface_t *)bt_profile_handsfree_init(flrd);
  flrd->hid   = (const bthd_interface_t        *)bt_profile_hid_init(flrd);
  flrd->avrcs = (ServiceInterface              *)bt_profile_avrcp_service_init(flrd);

  pthread_cond_broadcast(&flrd->cond);
  pthread_mutex_unlock(&flrd->mutex);
}

static void parse_properties(int num_properties, bt_property_t *property)
{
  while (num_properties-- > 0) {
    switch (property->type) {
      case BT_PROPERTY_BDADDR:
        {
          const RawAddress *addr = property_as_addr(property);
          LOG_SAMPLES("[%s]: addr: %s\n",
              dump_property_type(property->type), addr->ToString().c_str());
        }
        break;

      case BT_PROPERTY_TYPE_OF_DEVICE:
        {
          bt_device_type_t device_type = property_as_device_type(property);
          if (device_type) {
            const struct {
              const char *device_type;
            } device_type_lookup[] = {
              { "Unknown" },
              { "Classic Only" },
              { "BLE Only" },
              { "Both Classic and BLE" },
            };
            int idx = (int)device_type;
            if (idx > BT_DEVICE_DEVTYPE_DUAL)
              idx = 0;
            LOG_SAMPLES("[%s]: type: %s\n", dump_property_type(property->type),
                device_type_lookup[idx].device_type);
          }
        }
        break;

      case BT_PROPERTY_CLASS_OF_DEVICE:
        {
          LOG_SAMPLES("[%s]: class: 0x%x\n", dump_property_type(property->type),
              device_class_to_int(property_as_device_class(property)));
        }
        break;

      case BT_PROPERTY_REMOTE_RSSI:
        {
          LOG_SAMPLES("[%s]: rssi: %d\n", dump_property_type(property->type),
              property_as_rssi(property));
        }
        break;
      default:
        break;
    }
    property++;
  }
}

static void adapter_properties(bt_status_t status, int num_properties, bt_property_t *properties)
{
  parse_properties(num_properties, properties);
}

static void remote_device_properties(bt_status_t status, RawAddress *bd_addr, int num_properties, bt_property_t *properties)
{
  parse_properties(num_properties, properties);
}

static void device_found(int num_properties, bt_property_t *properties) TRACE_CALLBACK_BODY
static void discovery_state_changed(bt_discovery_state_t state) TRACE_CALLBACK_BODY
static void pin_request(RawAddress *remote_bd_addr, bt_bdname_t *bd_name, uint32_t cod, bool min_16_digit) TRACE_CALLBACK_BODY

static void ssp_request(RawAddress *remote_bd_addr, bt_bdname_t *bd_name, uint32_t cod,
    bt_ssp_variant_t pairing_variant, uint32_t pass_key)
{
  struct fluoride_s *flrd = fluoride_interface_get();

  LOG_SAMPLES("%s: class: %" PRIu32 ", passkey: %" PRIx32 ", variant: %d\n", __func__, cod, pass_key, pairing_variant);

  flrd->interface->ssp_reply(remote_bd_addr, pairing_variant, true, pass_key);
}

static void bond_state_changed(bt_status_t status, RawAddress *remote_bd_addr, bt_bond_state_t state)
{
  LOG_SAMPLES("%s: state: %d\n", __func__, state);
}

static void acl_state_changed(bt_status_t status, RawAddress *remote_bd_addr, bt_acl_state_t state)
{
  LOG_SAMPLES("%s: state: %d, acl_state: %d\n", __func__, status, state);
}

static void thread_event(bt_cb_thread_evt evt) TRACE_CALLBACK_BODY
static void dut_mode_recv(uint16_t opcode, uint8_t *buf, uint8_t len) TRACE_CALLBACK_BODY
static void le_test_mode(bt_status_t status, uint16_t num_packets) TRACE_CALLBACK_BODY
static void energy_info(bt_activity_energy_info *energy_info, bt_uid_traffic_t *uid_data) TRACE_CALLBACK_BODY

static bt_callbacks_t bt_callbacks =
{
  .size = sizeof(bt_callbacks_t),
    .adapter_state_changed_cb     = adapter_state_changed,
    .adapter_properties_cb        = adapter_properties,
    .remote_device_properties_cb  = remote_device_properties,
    .device_found_cb              = device_found,
    .discovery_state_changed_cb   = discovery_state_changed,
    .pin_request_cb               = pin_request,
    .ssp_request_cb               = ssp_request,
    .bond_state_changed_cb        = bond_state_changed,
    .acl_state_changed_cb         = acl_state_changed,
    .thread_evt_cb                = thread_event,
    .dut_mode_recv_cb             = dut_mode_recv,
    .le_test_mode_cb              = le_test_mode,
    .energy_info_cb               = energy_info,
};

static struct fluoride_s g_fluoride =
{
  .state = BT_STATE_OFF,
  .mutex = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP,
  .cond  = PTHREAD_COND_INITIALIZER,
};

static int bt_interface_init(void)
{
  extern bt_interface_t bluetoothInterface;
  struct fluoride_s *flrd = &g_fluoride;
  int ret;

  pthread_mutex_lock(&flrd->mutex);

  if (flrd->interface) {
    ret = BT_STATUS_DONE;
    goto bail;
  }

  flrd->interface = &bluetoothInterface;

  ret = bluetoothInterface.init(&bt_callbacks,
      false, false, 0, nullptr, false);
  if (ret != BT_STATUS_SUCCESS)
    goto bail;

  ret = bluetoothInterface.enable();
  if (ret != BT_STATUS_SUCCESS)
    goto bail;

  while (flrd->state != BT_STATE_ON)
    pthread_cond_wait(&flrd->cond, &flrd->mutex);

bail:
  pthread_mutex_unlock(&flrd->mutex);

  return ret;
}

struct fluoride_s *fluoride_interface_get(void)
{
  int ret;

  ret = bt_interface_init();
  if (ret == BT_STATUS_DONE)
    return &g_fluoride;
  else if (ret == BT_STATUS_SUCCESS)
    pause();

  return NULL;
}
