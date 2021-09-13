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
#include <sys/socket.h>
#include <sys/types.h>

#include "fluoride.h"

static ssize_t socket_read_all(int fd, void *buf, size_t len, int *outfd)
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

  ret = recvmsg(fd, &msg, 0);
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

static ssize_t socket_read(int fd, void *buf, size_t len)
{
  return socket_read_all(fd, buf, len, NULL);
}

static int socket_read_fd(int fd, void* buf, size_t len)
{
  int outfd = -1;
  int ret;

  ret = socket_read_all(fd, buf, len, &outfd);

  return ret < 0 ? ret : outfd;
}

static void *socket_thread_proc(void *arg)
{
  struct fluoride_s *flrd = (struct fluoride_s *)arg;
  sock_connect_signal_t cs;
  unsigned char buffer[64];
  int32_t scn;
  int ret;

  ret = socket_read(flrd->rfcfd, &scn, sizeof(scn));
  if (ret != sizeof(scn))
    goto bail;

  flrd->accfd = -1;

  while (1) {
    if (flrd->accfd < 0) {
      flrd->accfd = socket_read_fd(flrd->rfcfd, &cs, sizeof(cs));
    } else {
      ret = socket_read(flrd->accfd, buffer, sizeof(buffer));
      if (ret > 0)
        ret = write(flrd->accfd, buffer, ret);
      if (ret <= 0) {
        close(flrd->accfd);
        flrd->accfd = -1;
      }
    }
  }

bail:
  if (flrd->rfcfd > 0)
    close(flrd->rfcfd);

  return NULL;
}

const btsock_interface_t* bt_profile_socket_get(struct fluoride_s* flrd)
{
  Uuid uuid = Uuid::From16Bit(UUID_SERVCLASS_SERIAL_PORT);
  const btsock_interface_t* sock;
  bt_status_t status;
  int uid = 1000;

  sock = (const btsock_interface_t*)flrd->interface->get_profile_interface(
      BT_PROFILE_SOCKETS_ID);

  status = sock->listen(BTSOCK_RFCOMM, "rfcomm", &uuid, -1,
      &flrd->rfcfd, BTSOCK_FLAG_AUTH | BTSOCK_FLAG_ENCRYPT, uid);
  if (status != BT_STATUS_SUCCESS)
    return NULL;

  pthread_create(&flrd->pid, NULL, socket_thread_proc, flrd);
  pthread_detach(flrd->pid);

  return sock;
}

