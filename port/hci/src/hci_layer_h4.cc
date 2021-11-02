/****************************************************************************
 *
 *   Copyright (C) 2020 Xiaomi InC. All rights reserved.
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

#include <debug.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#ifdef CONFIG_KVDB
#include <kvdb.h>
#endif

#include <base/bind.h>
#include <base/location.h>
#include <base/logging.h>
#include <base/threading/thread.h>

#include "buffer_allocator.h"
#include "hci_internals.h"
#include "hci_layer.h"
#include "l2cdefs.h"

#include "osi/include/log.h"

enum HciPacketType {
  HCI_PACKET_TYPE_UNKNOWN   = 0,
  HCI_PACKET_TYPE_COMMAND   = 1,
  HCI_PACKET_TYPE_ACL_DATA  = 2,
  HCI_PACKET_TYPE_SCO_DATA  = 3,
  HCI_PACKET_TYPE_EVENT     = 4,
  HCI_PACKET_TYPE_ISO_DATA  = 5,
};

struct bt_hci_evt_hdr {
  uint8_t  evt;
  uint8_t  len;
};

struct bt_hci_acl_hdr {
  uint16_t handle;
  uint16_t len;
};

struct bt_hci_iso_hdr {
  uint16_t handle;
  uint16_t len;
};

extern void hci_event_received(const base::Location& from_here, BT_HDR* packet);
extern void acl_event_received(BT_HDR* packet);
extern void iso_data_received(BT_HDR* packet);

static pthread_mutex_t g_mutex = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
static struct file     g_filep;
static bool            g_debug;

void hci_close() { }
int  hci_open_firmware_log_file() { return INVALID_FD; }
void hci_close_firmware_log_file(int fd) { }
void hci_log_firmware_debug_packet(int fd, BT_HDR* packet) { }

static void h4_data_dump(const char *tag, uint8_t type, uint8_t *data, uint32_t len)
{
  struct bt_hci_acl_hdr *acl;
  struct iovec bufs[2];
  uint16_t cid;

  if (!g_debug)
    return;

  if (type == HCI_PACKET_TYPE_ACL_DATA && len > 69) {
    acl = (struct bt_hci_acl_hdr *)data;
    cid = *(uint16_t *)(data + 6);
    if (cid > L2CAP_BASE_APPL_CID ||
        (acl->handle >> 12) == L2CAP_PKT_CONTINUE)
      return;
  }

  bufs[0].iov_base = &type;
  bufs[0].iov_len = 1;
  bufs[1].iov_base = data;
  bufs[1].iov_len = len;

  lib_dumpvbuffer(tag, bufs, 2);
}

static int h4_recv_data(uint8_t *buf, int count)
{
  int ret, nread = 0;

  while (count != nread) {
    ret = file_read(&g_filep, buf + nread, count - nread);
    if (ret < 0) {
      if (ret == -EAGAIN) {
        usleep(500);
        continue;
      } else
        return ret;
    }

    nread += ret;
  }

  return nread;
}

static int h4_send_data(uint8_t *buf, int count)
{
  int ret, nwritten = 0;

  while (nwritten != count) {
    ret = file_write(&g_filep, buf + nwritten, count - nwritten);
    if (ret < 0) {
      if (ret == -EAGAIN) {
        usleep(500);
        continue;
      } else
        return ret;
    }

    nwritten += ret;
  }

  return nwritten;
}

static void *h4_rx_thread(void *arg)
{
  const allocator_t* allocator = buffer_allocator_get_interface();
  BT_HDR* packet;
  uint8_t *data;
  int hdr_len;
  int data_len;
  uint8_t type;
  union {
    struct bt_hci_evt_hdr evt;
    struct bt_hci_acl_hdr acl;
    struct bt_hci_iso_hdr iso;
  } hdr;
  int ret;

  for (;;) {
    ret = h4_recv_data(&type, 1);
    pthread_mutex_lock(&g_mutex);
    if (ret != 1)
      break;

    if (type == HCI_PACKET_TYPE_EVENT)
      hdr_len = sizeof(struct bt_hci_evt_hdr);
    else if (type == HCI_PACKET_TYPE_ACL_DATA)
      hdr_len = sizeof(struct bt_hci_acl_hdr);
    else if (type == HCI_PACKET_TYPE_ISO_DATA)
      hdr_len = sizeof(struct bt_hci_iso_hdr);
    else
      break;

    ret = h4_recv_data((uint8_t *)&hdr, hdr_len);
    if (ret != hdr_len)
      break;

    if (type == HCI_PACKET_TYPE_EVENT)
      data_len = hdr.evt.len;
    else if (type == HCI_PACKET_TYPE_ACL_DATA)
      data_len = hdr.acl.len;
    else if (type == HCI_PACKET_TYPE_ISO_DATA)
      data_len = hdr.iso.len;
    else
      break;

    packet = reinterpret_cast<BT_HDR*>(allocator->alloc(hdr_len + data_len + BT_HDR_SIZE));
    packet->offset = 0;
    packet->layer_specific = 0;

    ret = h4_recv_data(packet->data + hdr_len, data_len);
    if (ret != data_len)
      break;

    memcpy(packet->data, &hdr, hdr_len);

    packet->len = hdr_len + data_len;
    h4_data_dump("R", type, packet->data, packet->len);

    pthread_mutex_unlock(&g_mutex);

    if (type == HCI_PACKET_TYPE_EVENT) {
      packet->event = MSG_HC_TO_STACK_HCI_EVT;
      hci_event_received(FROM_HERE, packet);
    } else if (type == HCI_PACKET_TYPE_ACL_DATA) {
      packet->event = MSG_HC_TO_STACK_HCI_ACL;
      acl_event_received(packet);
    } else if (type == HCI_PACKET_TYPE_ISO_DATA) {
      packet->event = MSG_HC_TO_STACK_HCI_ISO;
      iso_data_received(packet);
    }
  }

  pthread_mutex_unlock(&g_mutex);

  LOG_ERROR("%s: invalid hci packet.", __func__);

  return NULL;
}


void hci_transmit(BT_HDR* packet)
{
  uint8_t type;
  int ret;

  switch (packet->event & MSG_EVT_MASK) {
    case MSG_STACK_TO_HC_HCI_CMD:
      type = HCI_PACKET_TYPE_COMMAND;
      break;
    case MSG_STACK_TO_HC_HCI_ACL:
      type = HCI_PACKET_TYPE_ACL_DATA;
      break;
    case MSG_STACK_TO_HC_HCI_ISO:
      type = HCI_PACKET_TYPE_ISO_DATA;
      break;
    default:
      return;
  }

  pthread_mutex_lock(&g_mutex);

  ret = h4_send_data(&type, 1);
  if (ret != 1) {
    pthread_mutex_unlock(&g_mutex);
    return;
  }

  h4_data_dump("W", type, packet->data + packet->offset, packet->len);

  ret = h4_send_data(packet->data + packet->offset, packet->len);
  if (ret != packet->len)
    LOG_ERROR("%s: fail to send to hci packet.", __func__);

  pthread_mutex_unlock(&g_mutex);
}

void hci_initialize(void)
{
  struct sched_param sparam;
  pthread_attr_t pattr;
  pthread_t pid;
  int ret;

#ifdef CONFIG_KVDB
  if (property_get_bool("persist.bluetooth.hcidebug", false))
    g_debug = true;
#endif

  ret = file_open(&g_filep, CONFIG_FLUORIDE_HCI_UART_NAME, O_RDWR | O_BINARY);
  if (ret < 0)
    goto bail;

  pthread_attr_init(&pattr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 9;
  pthread_attr_setschedparam(&pattr, &sparam);
  pthread_attr_setschedpolicy(&pattr, SCHED_FIFO);
  pthread_attr_setstacksize(&pattr, CONFIG_FLUORIDE_HCI_RX_STACKSIZE);

  ret = pthread_create(&pid, &pattr, h4_rx_thread, NULL);
  pthread_attr_destroy(&pattr);
  if (ret < 0)
    goto bail;

  prctl(PR_SET_NAME_EXT, "bt_rx_thread", pid);

  extern void initialization_complete();
  initialization_complete();

  return;

bail:
  file_close(&g_filep);

  LOG_ERROR("%s: fail to initialize hci drvier.", __func__);
}
