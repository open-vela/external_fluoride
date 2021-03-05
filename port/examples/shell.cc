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

#define PASSTHROUGH_COMMAND(C) { #C, C }

struct passthrough_command
{
  const unsigned char *command;
  uint8_t             value;
};

struct passthrough_command g_passthrough_command_maps[] =
{
  PASSTHROUGH_COMMAND(AVRC_ID_SELECT),     /* select */
  PASSTHROUGH_COMMAND(AVRC_ID_UP),         /* up */
  PASSTHROUGH_COMMAND(AVRC_ID_DOWN),       /* down */
  PASSTHROUGH_COMMAND(AVRC_ID_LEFT),       /* left */
  PASSTHROUGH_COMMAND(AVRC_ID_RIGHT),      /* right */
  PASSTHROUGH_COMMAND(AVRC_ID_RIGHT_UP),   /* right-up */
  PASSTHROUGH_COMMAND(AVRC_ID_RIGHT_DOWN), /* right-down */
  PASSTHROUGH_COMMAND(AVRC_ID_LEFT_UP),    /* left-up */
  PASSTHROUGH_COMMAND(AVRC_ID_LEFT_DOWN),  /* left-down */
  PASSTHROUGH_COMMAND(AVRC_ID_ROOT_MENU),  /* root menu */
  PASSTHROUGH_COMMAND(AVRC_ID_SETUP_MENU), /* setup menu */
  PASSTHROUGH_COMMAND(AVRC_ID_CONT_MENU),  /* contents menu */
  PASSTHROUGH_COMMAND(AVRC_ID_FAV_MENU),   /* favorite menu */
  PASSTHROUGH_COMMAND(AVRC_ID_EXIT),       /* exit */
  PASSTHROUGH_COMMAND(AVRC_ID_0),          /* 0 */
  PASSTHROUGH_COMMAND(AVRC_ID_1),          /* 1 */
  PASSTHROUGH_COMMAND(AVRC_ID_2),          /* 2 */
  PASSTHROUGH_COMMAND(AVRC_ID_3),          /* 3 */
  PASSTHROUGH_COMMAND(AVRC_ID_4),          /* 4 */
  PASSTHROUGH_COMMAND(AVRC_ID_5),          /* 5 */
  PASSTHROUGH_COMMAND(AVRC_ID_6),          /* 6 */
  PASSTHROUGH_COMMAND(AVRC_ID_7),          /* 7 */
  PASSTHROUGH_COMMAND(AVRC_ID_8),          /* 8 */
  PASSTHROUGH_COMMAND(AVRC_ID_9),          /* 9 */
  PASSTHROUGH_COMMAND(AVRC_ID_DOT),        /* dot */
  PASSTHROUGH_COMMAND(AVRC_ID_ENTER),      /* enter */
  PASSTHROUGH_COMMAND(AVRC_ID_CLEAR),      /* clear */
  PASSTHROUGH_COMMAND(AVRC_ID_CHAN_UP),    /* channel up */
  PASSTHROUGH_COMMAND(AVRC_ID_CHAN_DOWN),  /* channel down */
  PASSTHROUGH_COMMAND(AVRC_ID_PREV_CHAN),  /* previous channel */
  PASSTHROUGH_COMMAND(AVRC_ID_SOUND_SEL),  /* sound select */
  PASSTHROUGH_COMMAND(AVRC_ID_INPUT_SEL),  /* input select */
  PASSTHROUGH_COMMAND(AVRC_ID_DISP_INFO),  /* display information */
  PASSTHROUGH_COMMAND(AVRC_ID_HELP),       /* help */
  PASSTHROUGH_COMMAND(AVRC_ID_PAGE_UP),    /* page up */
  PASSTHROUGH_COMMAND(AVRC_ID_PAGE_DOWN),  /* page down */
  PASSTHROUGH_COMMAND(AVRC_ID_POWER),      /* power */
  PASSTHROUGH_COMMAND(AVRC_ID_VOL_UP),     /* volume up */
  PASSTHROUGH_COMMAND(AVRC_ID_VOL_DOWN),   /* volume down */
  PASSTHROUGH_COMMAND(AVRC_ID_MUTE),       /* mute */
  PASSTHROUGH_COMMAND(AVRC_ID_PLAY),       /* play */
  PASSTHROUGH_COMMAND(AVRC_ID_STOP),       /* stop */
  PASSTHROUGH_COMMAND(AVRC_ID_PAUSE),      /* pause */
  PASSTHROUGH_COMMAND(AVRC_ID_RECORD),     /* record */
  PASSTHROUGH_COMMAND(AVRC_ID_REWIND),     /* rewind */
  PASSTHROUGH_COMMAND(AVRC_ID_FAST_FOR),   /* fast forward */
  PASSTHROUGH_COMMAND(AVRC_ID_EJECT),      /* eject */
  PASSTHROUGH_COMMAND(AVRC_ID_FORWARD),    /* forward */
  PASSTHROUGH_COMMAND(AVRC_ID_BACKWARD),   /* backward */
  PASSTHROUGH_COMMAND(AVRC_ID_ANGLE),      /* angle */
  PASSTHROUGH_COMMAND(AVRC_ID_SUBPICT),    /* subpicture */
  PASSTHROUGH_COMMAND(AVRC_ID_F1),         /* F1 */
  PASSTHROUGH_COMMAND(AVRC_ID_F2),         /* F2 */
  PASSTHROUGH_COMMAND(AVRC_ID_F3),         /* F3 */
  PASSTHROUGH_COMMAND(AVRC_ID_F4),         /* F4 */
  PASSTHROUGH_COMMAND(AVRC_ID_F5),         /* F5 */
  PASSTHROUGH_COMMAND(AVRC_ID_VENDOR),     /* vendor unique */
  PASSTHROUGH_COMMAND(AVRC_KEYPRESSED_RELEASE),
};

typedef int (*flrdcmd_func)(struct fluoride_s *flrd, int argc, char **argv);

struct fluoride_cmd_s
{
  const char     *cmd;
  flrdcmd_func    func;
  const char     *help;
};

static void fluoride_avrcp_help(void)
{
  int i;
  for (i = 0; i < sizeof(g_passthrough_command_maps) /
      sizeof(g_passthrough_command_maps[0]); i++)
    printf("[%3d] KEY CODE:  %3d  (%s)\n", i,
        g_passthrough_command_maps[i].value,
        g_passthrough_command_maps[i].command);
}

static int fluoride_command_key(struct fluoride_s *flrd, int argc, char **argv)
{
  int command;

  if (argc == 0 || strlen(argv[0]) <= 0) {
    fluoride_avrcp_help();
    return -1;
  }

  command = atoi(argv[0]);

  if (flrd->ctrl) {
    flrd->ctrl->send_pass_through_cmd(flrd->avrcp_addr, command, AVRC_STATE_PRESS);
    flrd->ctrl->send_pass_through_cmd(flrd->avrcp_addr, command, AVRC_STATE_RELEASE);
  }

  return 0;
}

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
  char buf[64] = {};
  char *env;
  int ret;

  ret = snprintf(buf, sizeof(buf), "%s", CONFIG_FLUORIDE_DEVICE_NAME);

  if (argc == 0 || strlen(argv[0]) <= 0) {
    env = getenv("SN");
    if (env != NULL && !memcmp(env, "00000", 5))
      env = NULL;

    if (env == NULL) {
      env = getenv("WMAC");
      if (env != NULL)
        snprintf(buf + ret, sizeof(buf) - ret, "-%c%c%c%c",
            env[12], env[13], env[15], env[16]);
    } else
      snprintf(buf + ret, sizeof(buf) - ret, "-%s", env + 11);
  } else
    snprintf(buf + ret, sizeof(buf) - ret, "%s", argv[0]);

  property = property_new_name(buf);
  ret = flrd->interface->set_adapter_property(property);
  property_free(property);

  return ret;
}

static int fluoride_command_volume(struct fluoride_s *flrd, int argc, char **argv)
{
  if (argc == 0 || strlen(argv[0]) <= 0)
    return -1;

  if (flrd->avrcp)
    return flrd->avrcp->set_volume(atoi(argv[0]));

  return -1;
}

static int fluoride_command_disconnect(struct fluoride_s *flrd, int argc, char **argv)
{
  if (flrd->avrcs)
    flrd->avrcs->DisconnectDevice(flrd->addr);

  if (flrd->sink)
    flrd->sink->disconnect(flrd->addr);

  return 0;
}

static int fluoride_command_playback_state(struct fluoride_s *flrd, int argc, char **argv)
{
  if (flrd->ctrl)
    flrd->ctrl->get_playback_state_cmd(flrd->avrcp_addr);

  return 0;
}

static int fluoride_command_dump(struct fluoride_s *flrd, int argc, char **argv)
{
  flrd->interface->dump(STDOUT_FILENO, NULL);
  return 0;
}

static int fluoride_command_connect(struct fluoride_s *flrd, int argc, char **argv)
{
  if (argc == 0 || strlen(argv[0]) <= 0) {
    return -1;
  }

  std::string str = argv[0];
  RawAddress bd_addr;

  if (!bd_addr.IsValidAddress(str))
    return -1;

  bd_addr.FromString(str, bd_addr);

  if (flrd->sink)
    flrd->sink->connect(bd_addr);

  return 0;
}

static struct fluoride_cmd_s g_shell_cmds[] =
{
  {
    "scan",
    fluoride_command_scan,
    "< 0-2 >    ( 0: off, 1: connectable, 2: connectable + discoverable )",
  },
  {
    "bdname",
    fluoride_command_bdname,
    "< bdname > ( bluetooth device name )",
  },
  {
    "volume",
    fluoride_command_volume,
    "< 0-127 >  ( AVRCP Volume - 1.4 enhancements )",
  },
  {
    "dump",
    fluoride_command_dump,
    "< N/A >    ( DumpSYS as Android Subsystem )",
  },
  {
    "connect",
    fluoride_command_connect,
    "< 00:11:22:33:44:55 > ( Peer MAC Address )",
  },
  {
    "disconnect",
    fluoride_command_disconnect,
    "< N/A >    ( Disconnect the current active connection )",
  },
  {
    "key",
    fluoride_command_key,
    "< 0-128 >  ( Passthrough Key Code )",
  },
  {
    "playback_state",
    fluoride_command_playback_state,
    "< N/A >    ( PTS: AVRCP/CT/MDI/BV-01-C )",
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
        printf("[%02d]: %s %16s : %s\n", i, argv[0], g_shell_cmds[i].cmd, g_shell_cmds[i].help);
  }

  pthread_mutex_unlock(&flrd->mutex);

  return ret;
}
