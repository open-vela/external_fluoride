/**********************************************************************
 *
 *  Copyright 2017 The Android Open Source Project
 *  Copyright 2015 Intel Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *  implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 **********************************************************************/

#include <base/bind.h>
#include <base/location.h>
#include <base/logging.h>
#include <base/threading/thread.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>

#include <sys/ioctl.h>
#include <sys/socket.h>

#include "buffer_allocator.h"
#include "hci_internals.h"
#include "hci_layer.h"
#include "osi/include/compat.h"
#include "osi/include/log.h"
#include "osi/include/osi.h"
#include "osi/include/properties.h"

#include <libusb-1.0/libusb.h>

#define HCI_BUFSIZE          800
//#define HCI_DEBUG

extern void hci_event_received(const base::Location& from_here, BT_HDR* packet);
extern void acl_event_received(BT_HDR* packet);
extern void sco_data_received(BT_HDR* packet);

enum HciPacketType {
  HCI_PACKET_TYPE_UNKNOWN   = 0,
  HCI_PACKET_TYPE_COMMAND   = 1,
  HCI_PACKET_TYPE_ACL_DATA  = 2,
  HCI_PACKET_TYPE_SCO_DATA  = 3,
  HCI_PACKET_TYPE_EVENT     = 4
};

enum {
  TRANSFER_INPUT_CMD,
  TRANSFER_INPUT_ACL,
  TRANSFER_OUTPUT_CMD,
  TRANSFER_OUTPUT_ACL,
};

struct usb_handler {
  struct libusb_transfer *transfer;
  int                     address;
  sem_t                   sem;
  int                     type;
};
static struct usb_handler      g_uhandle[4];
static libusb_device_handle   *g_handle;
static struct libusb_transfer *g_command_transfer;
static struct libusb_transfer *g_acl_transfer;

static uint8_t hci_command_input_buffer[HCI_BUFSIZE];
static uint8_t hci_acl_input_buffer[HCI_BUFSIZE];
static uint8_t hci_command_send_buffer[3 + 256 + LIBUSB_CONTROL_SETUP_SIZE];

void hci_close() { }
int  hci_open_firmware_log_file() { return INVALID_FD; }
void hci_close_firmware_log_file(int fd) { }
void hci_log_firmware_debug_packet(int fd, BT_HDR* packet) { }

static void usb_data_dump(const char *tag, uint8_t *data, uint32_t len)
{
#ifdef HCI_DEBUG
  uint8_t *end = data + len;

  printf("%s[%03d]: ", tag, len);
  while (data != end)
    printf("%02x,", *data++);
  printf("\n");
#endif
}

static void usb_callback(struct libusb_transfer *transfer)
{
  struct usb_handler *uhandle = (struct usb_handler *)transfer->user_data;

  if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
    if (transfer->endpoint == 0 ||
        uhandle->type == TRANSFER_OUTPUT_ACL)
      sem_post(&uhandle->sem);
    else if (uhandle->type == TRANSFER_INPUT_CMD) {
      if (g_command_transfer == NULL)
        g_command_transfer = transfer;
    } else if (uhandle->type == TRANSFER_INPUT_ACL) {
      if (g_acl_transfer == NULL)
        g_acl_transfer = transfer;
    }
  } else if (transfer->status == LIBUSB_TRANSFER_STALL) {
    libusb_clear_halt(transfer->dev_handle, transfer->endpoint);
    libusb_submit_transfer(transfer);
  }

}

static int usb_open(libusb_device_handle **handle)
{
  struct libusb_device_descriptor desc;
  libusb_device_handle *_handle = NULL;
  libusb_device **devs;
  ssize_t num_devices;
  int ret, i;

  libusb_set_debug(NULL, LIBUSB_LOG_LEVEL_WARNING);

  ret = libusb_get_device_list(NULL, &devs);
  if (ret < 0)
    return ret;

  num_devices = ret;

  for (i = 0; i < num_devices; i++) {
    ret = libusb_get_device_descriptor(devs[i], &desc);
    if (ret < 0)
      goto bail;

    if (desc.bDeviceClass    == 0xE0 &&
        desc.bDeviceSubClass == 0x01 &&
        desc.bDeviceProtocol == 0x01) {
      printf("%04x:%04x (bus %d, device %d) - class %x subclass %x protocol %x\n",
          desc.idVendor, desc.idProduct,
          libusb_get_bus_number(devs[i]), libusb_get_device_address(devs[i]),
          desc.bDeviceClass, desc.bDeviceSubClass, desc.bDeviceProtocol);
      break;
    }
  }


  if (i == num_devices) {
    ret = -EINVAL;
    goto bail;
  }

  ret = libusb_open(devs[i], &_handle);
  if (ret < 0)
    goto bail;

  ret = libusb_reset_device(_handle);

bail:
  if (ret < 0 && _handle)
    libusb_close(_handle);
  else
    *handle = _handle;

  libusb_free_device_list(devs, 1);

  return ret;
}

static int usb_prepare(libusb_device_handle *handle)
{
  const int configuration = 1;
  libusb_device *dev;
  int ret;

  dev = libusb_get_device(handle);
  if (dev == NULL)
    return -EINVAL;

  ret = libusb_kernel_driver_active(handle, 0);
  if (ret < 0)
    return ret;

  if (ret == 1) {
    ret = libusb_detach_kernel_driver(handle, 0);
    if (ret < 0)
      return ret;
  }

  ret = libusb_set_configuration(handle, configuration);
  if (ret < 0)
    goto bail;

  ret = libusb_claim_interface(handle, 0);
  if (ret < 0)
    goto bail;

bail:
  if (ret < 0)
    libusb_attach_kernel_driver(handle, 0);

  return ret;
}

static int usb_scan(libusb_device_handle *handle)
{
  const struct libusb_interface_descriptor *idescriptor;
  const struct libusb_endpoint_descriptor *endpoint;
  const struct libusb_interface *interface;
  struct libusb_config_descriptor *descriptor;
  int num_interfaces;
  int cmd_in_address;
  int acl_in_address;
  int acl_out_address;
  int sco_in_address;
  int sco_out_address;
  int i, j, ret;

  ret = libusb_get_active_config_descriptor(
      libusb_get_device(handle), &descriptor);
  if (ret < 0)
    return ret;

  num_interfaces = descriptor->bNumInterfaces;
  cmd_in_address = acl_in_address =
    acl_out_address = sco_in_address =
    sco_out_address = 0;

  for (i = 0; i < num_interfaces ; i++){
    interface   = &descriptor->interface[i];
    idescriptor = interface->altsetting;
    endpoint    = idescriptor->endpoint;

    for (j = 0; j < idescriptor->bNumEndpoints; j++, endpoint++) {
      switch (endpoint->bmAttributes & 0x3) {

        case LIBUSB_TRANSFER_TYPE_INTERRUPT:
          if (cmd_in_address)
            continue;

          cmd_in_address = endpoint->bEndpointAddress;
          printf("-> using 0x%2.2X for HCI Events\n", cmd_in_address);
          break;
        case LIBUSB_TRANSFER_TYPE_BULK:
          if (endpoint->bEndpointAddress & 0x80) {
            if (acl_in_address)
              continue;

            acl_in_address = endpoint->bEndpointAddress;
            printf("-> using 0x%2.2X for ACL Data In\n", acl_in_address);
          } else {
            if (acl_out_address)
              continue;

            acl_out_address = endpoint->bEndpointAddress;
            printf("-> using 0x%2.2X for ACL Data Out\n", acl_out_address);
          }
          break;

        case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
          if (endpoint->bEndpointAddress & 0x80) {
            if (sco_in_address)
              continue;

            sco_in_address = endpoint->bEndpointAddress;
            printf("-> using 0x%2.2X for SCO Data In\n", sco_in_address);
          } else {
            if (sco_out_address)
              continue;

            sco_out_address = endpoint->bEndpointAddress;
            printf("-> using 0x%2.2X for SCO Data Out\n", sco_out_address);
          }
          break;
        default:
          break;
      }
    }
  }

  g_uhandle[TRANSFER_INPUT_CMD].address  = cmd_in_address;
  g_uhandle[TRANSFER_INPUT_ACL].address  = acl_in_address;
  g_uhandle[TRANSFER_OUTPUT_ACL].address = acl_out_address;

  libusb_free_config_descriptor(descriptor);

  return 0;
}

static int usb_alloc(libusb_device_handle *handle)
{
  struct usb_handler *uhandle;
  unsigned int i;
  int ret;

  for (i = 0; i < ARRAY_SIZE(g_uhandle); i++) {
    uhandle = &g_uhandle[i];
    uhandle->transfer = libusb_alloc_transfer(0);
    if (uhandle->transfer == NULL)
      return -ENOMEM;
    sem_init(&uhandle->sem, 0, 1);
    uhandle->type = i;
  }

  uhandle = &g_uhandle[TRANSFER_INPUT_CMD];

  libusb_fill_interrupt_transfer(uhandle->transfer,
      handle, uhandle->address, hci_command_input_buffer, HCI_BUFSIZE,
      usb_callback, uhandle, 0);

  ret = libusb_submit_transfer(uhandle->transfer);
  if (ret)
    return ret;

  uhandle = &g_uhandle[TRANSFER_INPUT_ACL];

  libusb_fill_bulk_transfer(uhandle->transfer,
      handle, uhandle->address, hci_acl_input_buffer,
      HCI_BUFSIZE, usb_callback, uhandle, 0) ;

  return libusb_submit_transfer(uhandle->transfer);
}

static void usb_close(libusb_device_handle *handle)
{
  struct usb_handler *uhandle;
  struct timeval tv = {};
  unsigned int i;

  for (i = 0; i < ARRAY_SIZE(g_uhandle); i++) {
    uhandle = &g_uhandle[i];
    if (uhandle->transfer == NULL)
      continue;

    if (uhandle->type == TRANSFER_INPUT_CMD ||
        uhandle->type == TRANSFER_INPUT_ACL)
      libusb_cancel_transfer(uhandle->transfer);
    libusb_free_transfer(uhandle->transfer);
  }

  libusb_set_debug(NULL, LIBUSB_LOG_LEVEL_WARNING);
  libusb_handle_events_timeout(NULL, &tv);
  libusb_release_interface(handle, 0);
  libusb_close(handle);
}

static void *usb_rx_thread(void *arg)
{
  const allocator_t* allocator = buffer_allocator_get_interface();
  struct libusb_transfer *transfer;
  struct bt_hci_evt_hdr *evt;
  struct timeval tv = {};
  struct net_buf *buf;
  BT_HDR* packet;
  uint8_t *data;
  uint32_t len;
  uint8_t type;
  int ret;

  for (;;) {
    ret = libusb_handle_events_timeout(NULL, &tv);
    if (g_command_transfer) {
      transfer = g_command_transfer;
      g_command_transfer = NULL;
      type = HCI_PACKET_TYPE_EVENT;
    } else if (g_acl_transfer) {
      transfer = g_acl_transfer;
      g_acl_transfer = NULL;
      type = HCI_PACKET_TYPE_ACL_DATA;
    } else {
      usleep(1);
      continue;
    }

    data = transfer->buffer;
    len = transfer->actual_length;

    usb_data_dump("R", data, len);

    packet = reinterpret_cast<BT_HDR*>(allocator->alloc(len + BT_HDR_SIZE));
    packet->offset = 0;
    packet->layer_specific = 0;
    packet->len = len;
    memcpy(packet->data, data, len);

    if (type == HCI_PACKET_TYPE_EVENT) {
      packet->event = MSG_HC_TO_STACK_HCI_EVT;
      hci_event_received(FROM_HERE, packet);
    } else if (type == HCI_PACKET_TYPE_ACL_DATA) {
      packet->event = MSG_HC_TO_STACK_HCI_ACL;
      acl_event_received(packet);
    }

    ret = libusb_submit_transfer(transfer);
    if (ret < 0)
      break;
  }

  return NULL;
}

void hci_transmit(BT_HDR* packet)
{
  uint8_t *data = packet->data + packet->offset;
  uint16_t event = packet->event & MSG_EVT_MASK;
  struct usb_handler *uhandle;
  uint32_t len = packet->len;

  usb_data_dump("W", data, len);

  switch (event) {
    case MSG_STACK_TO_HC_HCI_CMD:
      uhandle = &g_uhandle[TRANSFER_OUTPUT_CMD];
      break;
    case MSG_STACK_TO_HC_HCI_ACL:
      uhandle = &g_uhandle[TRANSFER_OUTPUT_ACL];
      break;
    default:
      return;
  }

  sem_wait(&uhandle->sem);

  if (event == MSG_STACK_TO_HC_HCI_CMD) {
    libusb_fill_control_setup(hci_command_send_buffer,
        LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE, 0, 0, 0, len);
    memcpy(hci_command_send_buffer + LIBUSB_CONTROL_SETUP_SIZE, data, len);

    libusb_fill_control_transfer(uhandle->transfer, g_handle,
        hci_command_send_buffer, usb_callback, uhandle, 0);
    uhandle->transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;

  } else {
    libusb_fill_bulk_transfer(uhandle->transfer, g_handle,
        uhandle->address, data, len, usb_callback, uhandle, 0);
  }

  libusb_submit_transfer(uhandle->transfer);
}


void hci_initialize(void)
{
  libusb_device_handle *handle = NULL;
  pthread_attr_t pattr;
  pthread_t pid;
  int ret;

  ret = libusb_init(NULL);
  if (ret < 0)
    return;

  ret = usb_open(&handle);
  if (ret < 0)
    goto bail;

  ret = usb_prepare(handle);
  if (ret < 0)
    goto bail;

  ret = usb_scan(handle);
  if (ret < 0)
    goto bail;

  ret = usb_alloc(handle);
  if (ret < 0)
    goto bail;

  g_handle = handle;

  pthread_attr_init(&pattr);

  pthread_attr_setstacksize(&pattr, CONFIG_FLUORIDE_HCI_RX_STACKSIZE);

  ret = pthread_create(&pid, &pattr, usb_rx_thread, NULL);
  pthread_attr_destroy(&pattr);
  if (ret < 0)
    return;

  prctl(PR_SET_NAME_EXT, "bt_driver", pid);

  extern void initialization_complete();
  initialization_complete();

  return;

bail:
  if (handle)
    usb_close(handle);

  libusb_exit(NULL);
}
