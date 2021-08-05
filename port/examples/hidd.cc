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
#include <hardware/bt_hd.h>
#include "btif_hd.h"

#define QOS_TOKEN_RATE              800 // 9 bytes * 1000000 us / 11250 us
#define QOS_TOKEN_BUCKET_SIZE       9
#define QOS_PEAK_BANDWIDTH          0
#define QOS_LATENCY                 11250
#define QOS_LATENCY_MAX             0xffffffff

#define SUBCLASS1_NONE              0x00
#define SUBCLASS1_KEYBOARD          0x40
#define SUBCLASS1_MOUSE             0x80
#define SUBCLASS1_COMBO             0xC0
#define SUBCLASS2_UNCATEGORIZED     0x00
#define SUBCLASS2_JOYSTICK          0x01
#define SUBCLASS2_GAMEPAD           0x02
#define SUBCLASS2_REMOTE_CONTROL    0x03
#define SUBCLASS2_SENSING_DEVICE    0x04
#define SUBCLASS2_DIGITIZER_TABLET  0x05
#define SUBCLASS2_CARD_READER       0x06

#define ID_KEYBOARD                 1
#define ID_MOUSE                    2
#define ID_CONTROL                  3

static uint8_t dest_data[] =
{
  0x05, 0x01, // Usage page (Generic Desktop)
  0x09, 0x06, // Usage (Keyboard)
  0xA1, 0x01, // Collection (Application)
  0x85, ID_KEYBOARD, //    Report ID
  0x05, 0x07, //       Usage page (Key Codes)
  0x19, 0xE0, //       Usage minimum (224)
  0x29, 0xE7, //       Usage maximum (231)
  0x15, 0x00, //       Logical minimum (0)
  0x25, 0x01, //       Logical maximum (1)
  0x75, 0x01, //       Report size (1)
  0x95, 0x08, //       Report count (8)
  0x81, 0x02, //       Input (Data, Variable, Absolute) ; Modifier byte
  0x75, 0x08, //       Report size (8)
  0x95, 0x01, //       Report count (1)
  0x81, 0x01, //       Input (Constant)                 ; Reserved byte
  0x75, 0x08, //       Report size (8)
  0x95, 0x06, //       Report count (6)
  0x15, 0x00, //       Logical Minimum (0)
  0x25, 0x65, //       Logical Maximum (101)
  0x05, 0x07, //       Usage page (Key Codes)
  0x19, 0x00, //       Usage Minimum (0)
  0x29, 0x65, //       Usage Maximum (101)
  0x81, 0x00, //       Input (Data, Array)              ; Key array (6 keys)
  0xC0, // End Collection

  0x05, 0x01, // Usage Page (Generic Desktop)
  0x09, 0x02, // Usage (Mouse)
  0xA1, 0x01, // Collection (Application)
  0x85, ID_MOUSE, //    Report ID
  0x09, 0x01, //    Usage (Pointer)
  0xA1, 0x00, //    Collection (Physical)
  0x05, 0x09, //       Usage Page (Buttons)
  0x19, 0x01, //       Usage minimum (1)
  0x29, 0x03, //       Usage maximum (3)
  0x15, 0x00, //       Logical minimum (0)
  0x25, 0x01, //       Logical maximum (1)
  0x75, 0x01, //       Report size (1)
  0x95, 0x03, //       Report count (3)
  0x81, 0x02, //       Input (Data, Variable, Absolute)
  0x75, 0x05, //       Report size (5)
  0x95, 0x01, //       Report count (1)
  0x81, 0x01, //       Input (constant)                 ; 5 bit padding
  0x05, 0x01, //       Usage page (Generic Desktop)
  0x09, 0x30, //       Usage (X)
  0x09, 0x31, //       Usage (Y)
  0x09, 0x38, //       Usage (Wheel)
  0x15, 0x81, //       Logical minimum (-127)
  0x25, 0x7F, //       Logical maximum (127)
  0x75, 0x08, //       Report size (8)
  0x95, 0x03, //       Report count (3)
  0x81, 0x06, //       Input (Data, Variable, Relative)
  0xC0, //    End Collection

  0x05, 0x0C, // Usage Page (Consumer)
  0x09, 0x01, // Usage (Consumer Control)
  0xA1, 0x01, // Collection (Application)
  0x85, ID_CONTROL,// Report Id (1)
  0x15, 0x00, //        Logical minimum (0)
  0x25, 0x01, //        Logical maximum (1)
  0x75, 0x01, //        Report Size (1)
  0x95, 0x01, //        Report Count (1)
  0x09, 0xCD, //        Usage (Play/Pause)
  0x81, 0x06, //        Input (Data,Value,Relative,Bit Field)
  0x0A, 0x83, 0x01, //  Usage (AL Consumer Control Configuration)
  0x81, 0x06, //        Input (Data,Value,Relative,Bit Field)
  0x09, 0xB5, //        Usage (Scan Next Track)
  0x81, 0x06, //        Input (Data,Value,Relative,Bit Field)
  0x09, 0xB6, //        Usage (Scan Previous Track)
  0x81, 0x06, //        Input (Data,Value,Relative,Bit Field)
  0x09, 0xEA, //        Usage (Volume Down)
  0x81, 0x06, //        Input (Data,Value,Relative,Bit Field)
  0x09, 0xE9, //        Usage (Volume Up)
  0x81, 0x06, //        Input (Data,Value,Relative,Bit Field)
  0x0A, 0x25, 0x02, //  Usage (AC Forward)
  0x81, 0x06, //        Input (Data,Value,Relative,Bit Field)
  0x0A, 0x24, 0x02, //  Usage (AC Back)
  0x81, 0x06, //        Input (Data,Value,Relative,Bit Field)
  0xC0, // End Collection
  0xC0 // End Collection
};

static bthd_app_param_t app_param = {
  .name               = "HID Input",
  .description        = "Fluoride HID Device",
  .provider           = "Xiaomi Inc.",
  .subclass           = SUBCLASS1_COMBO,
  .desc_list          = dest_data,
  .desc_list_len      = sizeof(dest_data),
};

static bthd_qos_param_t qos_param = {
  .service_type       = SVC_TYPE_BEST_EFFORT,
  .token_rate         = QOS_TOKEN_RATE,
  .token_bucket_size  = QOS_TOKEN_BUCKET_SIZE,
  .peak_bandwidth     = QOS_PEAK_BANDWIDTH,
  .access_latency     = QOS_LATENCY,
  .delay_variation    = QOS_LATENCY_MAX,
};

static void application_state_cb(RawAddress* bd_addr, bthd_application_state_t state) TRACE_CALLBACK_BODY
static void connection_state_cb(RawAddress* bd_addr, bthd_connection_state_t state) TRACE_CALLBACK_BODY
static void get_report_cb(uint8_t type, uint8_t id, uint16_t buffer_size) TRACE_CALLBACK_BODY
static void set_report_cb(uint8_t type, uint8_t id, uint16_t len, uint8_t* p_data) TRACE_CALLBACK_BODY
static void set_protocol_cb(uint8_t protocol) TRACE_CALLBACK_BODY
static void intr_data_cb(uint8_t report_id, uint16_t len, uint8_t* p_data) TRACE_CALLBACK_BODY
static void vc_unplug_cb(void) TRACE_CALLBACK_BODY

static bthd_callbacks_t bt_hid_callbacks = {
  sizeof(bt_hid_callbacks),
  application_state_cb,
  connection_state_cb,
  get_report_cb,
  set_report_cb,
  set_protocol_cb,
  intr_data_cb,
  vc_unplug_cb,
};

const bthd_interface_t *bt_profile_hid_init(struct fluoride_s *flrd)
{
  const bthd_interface_t *hid;

  hid = (const bthd_interface_t *)
    flrd->interface->get_profile_interface(BT_PROFILE_HIDDEV_ID);
  if (hid == NULL)
    return hid;

  hid->init((bthd_callbacks_t*)&bt_hid_callbacks);
  hid->register_app(&app_param, &qos_param, &qos_param);

  return hid;
}
