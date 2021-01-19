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

typedef int (*flrdcmd_func)(struct fluoride_s *flrd, int argc, char **argv);

struct fluoride_cmd_s
{
  const char     *cmd;
  flrdcmd_func    func;
  const char     *help;
};

static int fluoride_command_scan(struct fluoride_s *flrd, int argc, char **argv)
{
  bt_property_t *property;
  int mode;
  int ret;

  if (argc == 0)
    return -1;

  mode = atoi(argv[0]);
  if (mode < 0 || mode > 2)
    return -1;

  property = property_new_scan_mode((bt_scan_mode_t)mode);
  ret = flrd->interface->set_adapter_property(property);
  property_free(property);

  return ret;
}

static int fluoride_command_bdname(struct fluoride_s *flrd, int argc, char **argv)
{
  bt_property_t *property;
  int ret;

  if (argc == 0 || strlen(argv[0]) <= 0)
    return -1;

  property = property_new_name(argv[0]);
  ret = flrd->interface->set_adapter_property(property);
  property_free(property);

  return ret;
}

static int fluoride_command_volume(struct fluoride_s *flrd, int argc, char **argv)
{
  if (argc == 0 || strlen(argv[0]) <= 0)
    return -1;

  return flrd->avrcp->set_volume(atoi(argv[0]));
}

static struct fluoride_cmd_s g_shell_cmds[] =
{
  {
    "scan",
    fluoride_command_scan,
    "  < 0-2 > (0:off, 1:connectable, 2:connectable + discoverable)",
  },
  {
    "bdname",
    fluoride_command_bdname,
    "< bdname > (bluetooth device name)",
  },
  {
    "volume",
    fluoride_command_volume,
    "< 0-127 > (AVRCP Volume - 1.4 enhancements)",
  },
};

int fluoride_shell(struct fluoride_s *flrd, int argc, char **argv)
{
  int ret = -1;
  int i;

  while (!btif_is_enabled())
    usleep(100 * 1000);

  pthread_mutex_lock(&flrd->mutex);

  if (argc > 1) {
    for (i = 0; i < sizeof(g_shell_cmds) / sizeof(g_shell_cmds[0]); i++)
      if (!strncmp(g_shell_cmds[i].cmd, argv[1], strlen(g_shell_cmds[i].cmd))) {
        ret = g_shell_cmds[i].func(flrd, argc - 2, &argv[2]);
        break;
      }
  }

  if (ret < 0) {
    printf("%s shell commands :\n", argv[0]);
    for (i = 0; i < sizeof(g_shell_cmds) / sizeof(g_shell_cmds[0]); i++)
      if (g_shell_cmds[i].help)
        printf("[%02d]: %s %s: %s\n", i, argv[0], g_shell_cmds[i].cmd, g_shell_cmds[i].help);
  }

  pthread_mutex_unlock(&flrd->mutex);

  return ret;
}
