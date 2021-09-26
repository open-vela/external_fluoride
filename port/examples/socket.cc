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

#include <string.h>
#include <stdlib.h>

#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "fluoride.h"

static ssize_t socket_read_all(void *filep, void *buf,
                               size_t len, int *outfd)
{
  char cmsgbuf[CMSG_SPACE(sizeof(int))];
  struct cmsghdr *pcmsg;
  struct iovec iov[1];
  struct msghdr msg;
  ssize_t ret;

  iov[0].iov_base = buf;
  iov[0].iov_len = len;

  msg.msg_name = NULL;
  msg.msg_namelen = 0;
  msg.msg_iov = iov;
  msg.msg_iovlen = 1;
  msg.msg_control = cmsgbuf;
  msg.msg_controllen = sizeof(cmsgbuf);

  ret = psock_recvmsg(file_socket((struct file *)filep), &msg, 0);
  if (ret <= 0)
    return ret;

  if ((pcmsg = CMSG_FIRSTHDR(&msg)) != NULL &&
      pcmsg->cmsg_len == CMSG_LEN(sizeof(int))) {

    if (pcmsg->cmsg_level != SOL_SOCKET ||
        pcmsg->cmsg_type != SCM_RIGHTS)
      return ret;

    if (outfd)
      *outfd = *((int *)CMSG_DATA(pcmsg));
  }

  return ret;
}

static ssize_t socket_read(void *filep, void *buf, size_t len)
{
  return socket_read_all(filep, buf, len, NULL);
}

static int socket_read_fd(void *filep, void *buf, size_t len)
{
  int outfd = -1;
  int ret;

  ret = socket_read_all(filep, buf, len, &outfd);

  return ret < 0 ? ret : outfd;
}

extern "C" {

  int bt_socket_data_send(void *handle, const void *data, size_t len)
  {
    struct fluoride_s *flrd = fluoride_interface_get();
    struct file *filep = NULL;
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(flrd->accfp); i++) {
      if (flrd->accfp[i] == handle)
        filep = (struct file *)handle;
    }

    if (handle == NULL || !data || len < 0)
      return -EINVAL;

    return file_write(filep, data, len);
  }

  void bt_socket_register_cb(bt_socket_conn_cb_t ccb,
                             bt_socket_data_cb_t dcb)
  {
    struct fluoride_s *flrd = fluoride_interface_get();

    if (flrd->rfcfp == NULL || flrd->sock == NULL)
      return;

    flrd->sock_data_cb = dcb;
    flrd->sock_conn_cb = ccb;
  }
}

static bool bt_socket_conn_changed(struct fluoride_s *flrd,
                                   void *fp, bool connected)
{
  struct file *filep = (struct file *)fp;
  bool prepare = false;
  unsigned int i;

  if (connected)
    {
      for (i = 0; i < ARRAY_SIZE(flrd->accfp); i++) {
        if (flrd->accfp[i] == NULL) {
          flrd->accfp[i] = (struct file *)filep;
          break;
        }
      }

      if (i == ARRAY_SIZE(flrd->accfp)) {
        file_close(filep);
        return false;
      }

      if (flrd->sock_conn_cb)
        flrd->sock_conn_cb(flrd->accfp[i], true);

      prepare = true;
    }
  else
    {
      if (flrd->sock_conn_cb)
        flrd->sock_conn_cb(filep, false);

      for (i = 0; i < ARRAY_SIZE(flrd->accfp); i++) {
        if (flrd->accfp[i] == filep) {
          flrd->accfp[i] = NULL;
          break;
        }
      }

      file_close(filep);

      prepare = true;
    }

  return prepare;
}

static int bt_socket_prepare_poll(struct fluoride_s *flrd,
                                  struct pollfd *pfds)
{
  unsigned int i;
  int count = 0;

  for (i = 0; i < ARRAY_SIZE(flrd->accfp); i++) {
    if (flrd->accfp[i]) {
      pfds[count].ptr = flrd->accfp[i];
      pfds[count++].events |= POLLIN | POLLFILE;
    }
  }

  pfds[count].ptr = flrd->rfcfp;
  pfds[count++].events = POLLIN | POLLFILE;

  return count;
}

void bt_socket_loop(struct fluoride_s *flrd)
{
  struct pollfd pfds[ARRAY_SIZE(flrd->accfp) + 1];
  sock_connect_signal_t cs;
  bool prepare = true;
  struct file *filep;
  char data[1024];
  int nfds;
  int ret;
  int fd;
  int i;

  while (1) {
    if (prepare)
      nfds = bt_socket_prepare_poll(flrd, (struct pollfd *)&pfds);

    ret = poll(pfds, nfds, -1);
    if (ret <= 0)
      break;

    for (i = 0; i < nfds; i++) {
      if (pfds[i].revents & (POLLNVAL | POLLHUP | POLLERR)) {
        if (pfds[i].ptr == flrd->rfcfp)
          break;

        prepare = bt_socket_conn_changed(flrd, pfds[i].ptr, false);
      } else if (pfds[i].revents & POLLIN) {
        if (pfds[i].ptr == flrd->rfcfp) {
          fd = socket_read_fd(flrd->rfcfp, &cs, sizeof(cs));
          if (fd < 0 || fs_getfilep(fd, &filep) < 0)
            continue;

          prepare = bt_socket_conn_changed(flrd, filep, true);
        } else {
          ret = socket_read(pfds[i].ptr, data, sizeof(data));
          if (ret > 0) {
            if (flrd->sock_data_cb)
              ret = flrd->sock_data_cb(pfds[i].ptr, data, ret);
          }
          if (ret <= 0)
            prepare = bt_socket_conn_changed(flrd, pfds[i].ptr, false);

        }
      }
    }

  }
}

const btsock_interface_t *bt_profile_socket_init(struct fluoride_s* flrd)
{
  Uuid uuid = Uuid::From16Bit(UUID_SERVCLASS_SERIAL_PORT);
  const btsock_interface_t *sock;
  bt_status_t status;
  int uid = 1000;
  int32_t scn;
  int fd = -1;
  int ret;

  sock = (const btsock_interface_t*)
    flrd->interface->get_profile_interface(BT_PROFILE_SOCKETS_ID);

  status = sock->listen(BTSOCK_RFCOMM, "rfcomm", &uuid, -1,
      &fd, BTSOCK_FLAG_AUTH | BTSOCK_FLAG_ENCRYPT, uid);
  if (status != BT_STATUS_SUCCESS || fd < 0)
    return NULL;

  fs_getfilep(fd, &flrd->rfcfp);
  if (!flrd->rfcfp)
    goto fail;

  ret = socket_read(flrd->rfcfp, &scn, sizeof(scn));
  if (ret != sizeof(scn))
    goto fail;

  return sock;

fail:
  if (fd > 0)
    close(fd);

  return NULL;
}
