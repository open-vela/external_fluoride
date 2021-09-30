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
  const char *command;
  uint8_t     value;
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

// Simplified US Keyboard with Shift modifier

#define ID_KEYBOARD      1
#define ID_MOUSE         2
#define ID_CONTROL       3

#define CHAR_ILLEGAL     0xff
#define CHAR_RETURN     '\n'
#define CHAR_ESCAPE      27
#define CHAR_TAB         '\t'
#define CHAR_BACKSPACE   0x7f

static const uint8_t keytable_us_none [] = {
  CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /*   0-3 */
  'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j',                   /*  4-13 */
  'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't',                   /* 14-23 */
  'u', 'v', 'w', 'x', 'y', 'z',                                       /* 24-29 */
  '1', '2', '3', '4', '5', '6', '7', '8', '9', '0',                   /* 30-39 */
  CHAR_RETURN, CHAR_ESCAPE, CHAR_BACKSPACE, CHAR_TAB, ' ',            /* 40-44 */
  '-', '=', '[', ']', '\\', CHAR_ILLEGAL, ';', '\'', 0x60, ',',       /* 45-54 */
  '.', '/', CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,   /* 55-60 */
  CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 61-64 */
  CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 65-68 */
  CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 69-72 */
  CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 73-76 */
  CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 77-80 */
  CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 81-84 */
  '*', '-', '+', '\n', '1', '2', '3', '4', '5',                       /* 85-97 */
  '6', '7', '8', '9', '0', '.', 0xa7,                                 /* 97-100 */
};

static const uint8_t keytable_us_shift[] = {
  CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /*  0-3  */
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',                   /*  4-13 */
  'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',                   /* 14-23 */
  'U', 'V', 'W', 'X', 'Y', 'Z',                                       /* 24-29 */
  '!', '@', '#', '$', '%', '^', '&', '*', '(', ')',                   /* 30-39 */
  CHAR_RETURN, CHAR_ESCAPE, CHAR_BACKSPACE, CHAR_TAB, ' ',            /* 40-44 */
  '_', '+', '{', '}', '|', CHAR_ILLEGAL, ':', '"', 0x7E, '<',         /* 45-54 */
  '>', '?', CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,   /* 55-60 */
  CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 61-64 */
  CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 65-68 */
  CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 69-72 */
  CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 73-76 */
  CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 77-80 */
  CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 81-84 */
  '*', '-', '+', '\n', '1', '2', '3', '4', '5',                       /* 85-97 */
  '6', '7', '8', '9', '0', '.', 0xb1,                                 /* 97-100 */
};

typedef int (*flrdcmd_func)(struct fluoride_s *flrd, int argc, char **argv);

struct fluoride_cmd_s
{
  const char     *cmd;
  flrdcmd_func    func;
  const char     *help;
};

static void command_bta_avrcp_key_help(void)
{
  unsigned int i;

  for (i = 0; i < sizeof(g_passthrough_command_maps) /
      sizeof(g_passthrough_command_maps[0]); i++)
    printf("[%3d] KEY CODE:  %3d  (%s)\n", i,
        g_passthrough_command_maps[i].value,
        g_passthrough_command_maps[i].command);
}

static int command_bta_key(struct fluoride_s *flrd, int argc, char **argv)
{
  int command;

  if (argc == 0 || strlen(argv[0]) <= 0) {
    command_bta_avrcp_key_help();
    return -1;
  }

  command = atoi(argv[0]);

  if (flrd->rcctrl) {
    flrd->rcctrl->send_pass_through_cmd(flrd->avrcp_addr, command, AVRC_STATE_PRESS);
    flrd->rcctrl->send_pass_through_cmd(flrd->avrcp_addr, command, AVRC_STATE_RELEASE);
  }

  return 0;
}

static int command_bta_scan(struct fluoride_s *flrd, int argc, char **argv)
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

static int command_bta_bdname(struct fluoride_s *flrd, int argc, char **argv)
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

static int command_bta_laddr(struct fluoride_s *flrd, int argc, char **argv)
{
  printf("Local Address: %s\n", flrd->laddr->ToString().c_str());
  return 0;
}

static int command_bta_discovery(struct fluoride_s *flrd, int argc, char **argv)
{
  if (argc == 0 || atoi(argv[0]))
    return flrd->interface->start_discovery();
  return flrd->interface->cancel_discovery();
}

static int command_bta_volume(struct fluoride_s *flrd, int argc, char **argv)
{
  if (argc == 0 || strlen(argv[0]) <= 0)
    return -1;

  if (flrd->avrcp)
    return flrd->avrcp->set_volume(atoi(argv[0]));

  return -1;
}

static int command_bta_disconnect(struct fluoride_s *flrd, int argc, char **argv)
{
  if (flrd->avrcs)
    flrd->avrcs->DisconnectDevice(flrd->addr);

  if (flrd->hfc)
    flrd->hfc->disconnect(&flrd->addr);

  if (flrd->arole == AVDT_TSEP_SNK) {
    if (flrd->sink)
      flrd->sink->disconnect(flrd->addr);
  } else {
    if (flrd->source)
      flrd->source->disconnect(flrd->addr);
  }

  return 0;
}

static int command_bta_audiorole(struct fluoride_s *flrd, int argc, char **argv)
{
  int role;

  if (argc == 0)
    return -1;

  role = atoi(argv[0]);

  if (role != AVDT_TSEP_SRC && role != AVDT_TSEP_SNK)
    return -1;

  flrd->arole = role;

  return 0;
}

static int command_bta_playback_state(struct fluoride_s *flrd, int argc, char **argv)
{
  if (flrd->rcctrl)
    flrd->rcctrl->get_playback_state_cmd(flrd->avrcp_addr);

  return 0;
}

static int command_bta_dump(struct fluoride_s *flrd, int argc, char **argv)
{
  flrd->interface->dump(STDOUT_FILENO, NULL);
  return 0;
}

static int command_bta_connect(struct fluoride_s *flrd, int argc, char **argv)
{
  if (argc == 0 || strlen(argv[0]) <= 0) {
    return -1;
  }

  std::string str = argv[0];
  RawAddress bd_addr;

  if (!bd_addr.IsValidAddress(str))
    return -1;

  bd_addr.FromString(str, bd_addr);

  if (flrd->hfc)
    flrd->hfc->connect(&flrd->addr);

  if (flrd->arole == AVDT_TSEP_SNK) {
    if (flrd->sink)
      flrd->sink->connect(bd_addr);
  } else {
    if (flrd->source)
      flrd->source->connect(bd_addr);
  }

  return 0;
}

static bool lookup_keycode(uint8_t character,
                           const uint8_t *table,
                           int size, uint8_t *keycode)
{
  int i;

  for (i = 0; i < size; i++) {
    if (table[i] != character)
      continue;
    *keycode = i;
    return true;
  }

  return false;
}

static int command_bta_hkey(struct fluoride_s *flrd, int argc, char **argv)
{
  uint8_t report[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t code;
  int count;
  int i;

  if (argc == 0 || strlen(argv[0]) <= 0)
    return -1;

  count = strlen(argv[0]);

  for (i = 0; i < count; i++) {
    if (lookup_keycode(argv[0][i], keytable_us_none,
          sizeof(keytable_us_none), &code)) {
      report[0] = 0;
      report[3] = code;
      flrd->hid->send_report(BTHD_REPORT_TYPE_INTRDATA,
          ID_KEYBOARD, sizeof(report), report);
      report[3] = 0;
      flrd->hid->send_report(BTHD_REPORT_TYPE_INTRDATA,
          ID_KEYBOARD, sizeof(report), report);
      continue;
    }

    if (lookup_keycode(argv[0][i], keytable_us_shift,
          sizeof(keytable_us_shift), &code)) {
      report[0] = 2;
      report[3] = code;
      flrd->hid->send_report(BTHD_REPORT_TYPE_INTRDATA,
          ID_KEYBOARD, sizeof(report), report);
      report[3] = 0;
      flrd->hid->send_report(BTHD_REPORT_TYPE_INTRDATA,
          ID_KEYBOARD, sizeof(report), report);
    }
  }

  return 0;
}

static int command_bta_hmkey(struct fluoride_s *flrd, int argc, char **argv)
{
  uint8_t report[] = {0, 0, 0, 0};

  if (argc == 0 || strlen(argv[0]) <= 0)
    return -1;

  report[0] = atoi(argv[0]);
  for (int i = 1; i < argc; i++) {
    if (strlen(argv[i]) <= 0)
      return -1;

    if (atoi(argv[i]) > 127)
      report[i] = 127;
    else if (atoi(argv[i]) < -127)
      report[i] = -127;
    else
      report[i] = atoi(argv[i]);

    if (atoi(argv[i]) < 0)
      report[i] |= 0x80;
  }

  flrd->hid->send_report(BTHD_REPORT_TYPE_INTRDATA, ID_MOUSE, sizeof(report), report);

  return 0;
}

static int command_bta_hckey(struct fluoride_s *flrd, int argc, char **argv)
{
  uint8_t code;

  if (argc == 0 || strlen(argv[0]) <= 0)
    return -1;

  code = atoi(argv[0]);

  flrd->hid->send_report(BTHD_REPORT_TYPE_INTRDATA, ID_CONTROL, 1, &code);

  return 0;
}

static struct fluoride_cmd_s g_bta_cmds[] =
{
  {
    "scan",
    command_bta_scan,
    "< 0-2 >    ( 0: off, 1: connectable, 2: connectable + discoverable )",
  },
  {
    "bdname",
    command_bta_bdname,
    "< bdname > ( bluetooth device name )",
  },
  {
    "laddr",
    command_bta_laddr,
    "< N/A >    ( Get Local Address )",
  },
  {
    "discovery",
    command_bta_discovery,
    "< 0-1 >    ( 0: cancel, 1: start )",
  },
  {
    "volume",
    command_bta_volume,
    "< 0-127 >  ( AVRCP Volume - 1.4 enhancements )",
  },
  {
    "dump",
    command_bta_dump,
    "< N/A >    ( DumpSYS as Android Subsystem )",
  },
  {
    "connect",
    command_bta_connect,
    "< 00:11:22:33:44:55 > ( Peer MAC Address )",
  },
  {
    "disconnect",
    command_bta_disconnect,
    "< N/A >    ( Disconnect the current active connection )",
  },
  {
    "arole",
    command_bta_audiorole,
    "< 0-1 >    ( A2DP audio role: 0: source, 1: sink )",
  },
  {
    "key",
    command_bta_key,
    "< 0-128 >  ( Passthrough Key Code )",
  },
  {
    "playback_state",
    command_bta_playback_state,
    "< N/A >    ( PTS: AVRCP/CT/MDI/BV-01-C )",
  },
  {
    "hkey",
    command_bta_hkey,
    "< string > ( HID keyboard Usage )",
  },
  {
    "hmkey",
    command_bta_hmkey,
    "< value... > ( HID mouse Usage )\n"
    "E.G:\n"
    "\tLeftmouse press  :  1\n"
    "\tRightmouse press :  2\n"
    "\tMiddlemouse press:  4\n"
    "\tMove left 20 unit:  0 -20\n"
    "\tMove down 20 unit:  0 0 20\n"
    "\tRoller up 20 unit:  0 0 0 -20\n",
  },
  {
    "hckey",
    command_bta_hckey,
    "< value  > ( HID Control key )\n"
    "E.G:\n"
    "\tVoleme Up  :  16 (0x10)\n"
    "\tVoleme Down:  32 (0x20)\n",
  },
};

static int command_ble_scan(struct fluoride_s *flrd, int argc, char **argv)
{
  if (argc == 0)
    return -1;

  flrd->gatt->scanner->Scan(!!atoi(argv[0]));

  return 0;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
  printf("Connected, err: %x\n", err);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
  printf("Disconnected (reason 0x%02x)\n", reason);
}

static struct bt_conn_cb conn_callbacks = {
  .connected    = connected,
  .disconnected = disconnected,
};

static const struct bt_data advdata[] =
{
  BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
                BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
                BT_UUID_16_ENCODE(BT_UUID_CTS_VAL)),
  BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12),
};

static const struct bt_data scandata[] =
{
  BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_FLUORIDE_DEVICE_NAME,
          sizeof(CONFIG_FLUORIDE_DEVICE_NAME)),
};

static uint8_t ct[10];
static uint8_t ct_update;
static uint8_t hrs_blsc;
static uint8_t battery_level = 100U;

static void ct_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) TRACE_CALLBACK_BODY

static ssize_t read_ct(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
  return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(ct));
}

static ssize_t write_ct(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
  uint8_t *value = (uint8_t *)attr->user_data;

  if (offset + len > sizeof(ct))
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);

  memcpy(value + offset, buf, len);
  ct_update = 1U;

  return len;
}

static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

  printf("HRS notifications %s\n", notif_enabled ? "enabled" : "disabled");
}

static ssize_t read_blsc(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         void *buf, uint16_t len, uint16_t offset)
{
  return bt_gatt_attr_read(conn, attr, buf, len,
                           offset, &hrs_blsc, sizeof(hrs_blsc));
}

static void blvl_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                 uint16_t value)
{
  bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

  printf("BAS Notifications %s\n", notif_enabled ? "enabled" : "disabled");
}

static ssize_t read_blvl(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr, void *buf,
                         uint16_t len, uint16_t offset)
{
  uint8_t lvl8 = battery_level;

  return bt_gatt_attr_read(conn, attr, buf, len, offset, &lvl8,
         sizeof(lvl8));
}

/* GATT DEMO */
static struct bt_gatt_attr cts_attrs[] =
{
  BT_GATT_PRIMARY_SERVICE(BT_UUID_CTS),
  BT_GATT_CHARACTERISTIC(BT_UUID_CTS_CURRENT_TIME, BT_GATT_CHRC_READ |
                         BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE,
                         BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                         read_ct, write_ct, ct),

  BT_GATT_CCC(ct_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

  BT_GATT_SECONDARY_SERVICE(BT_UUID_HRS),
  BT_GATT_CHARACTERISTIC(BT_UUID_HRS_MEASUREMENT, BT_GATT_CHRC_NOTIFY,
                         BT_GATT_PERM_NONE, NULL, NULL, NULL),
  BT_GATT_CCC(hrmc_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  BT_GATT_CHARACTERISTIC(BT_UUID_HRS_BODY_SENSOR, BT_GATT_CHRC_READ,
                         BT_GATT_PERM_READ, read_blsc, NULL, NULL),
  BT_GATT_CHARACTERISTIC(BT_UUID_HRS_CONTROL_POINT, BT_GATT_CHRC_WRITE,
                         BT_GATT_PERM_NONE, NULL, NULL, NULL),

  BT_GATT_SECONDARY_SERVICE(BT_UUID_BAS),
  BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
                         BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                         BT_GATT_PERM_READ, read_blvl, NULL,
                         &battery_level),
  BT_GATT_CCC(blvl_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service cts_service = BT_GATT_SERVICE(cts_attrs);

static int command_ble_adv(struct fluoride_s *flrd, int argc, char **argv)
{
  bt_le_adv_param param;

  if (argc == 0 || atoi(argv[0]) == 0) {
    bt_le_adv_stop();
    return 0;
  }

  bt_conn_cb_register(&conn_callbacks);
  bt_gatt_service_register(&cts_service);

  param = BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE |
                               BT_LE_ADV_OPT_SCANNABLE |
                               BT_LE_ADV_OPT_USE_NAME,
                               BT_GAP_ADV_FAST_INT_MIN_2,
                               BT_GAP_ADV_FAST_INT_MAX_2, NULL);

  return bt_le_adv_start(&param, advdata, ARRAY_SIZE(advdata),
                         scandata, ARRAY_SIZE(scandata));
}
/* GATT DEMO END */

static struct fluoride_cmd_s g_ble_cmds[] =
{
  {
    "scan",
    command_ble_scan,
    "< 0-1 >    ( 0: off, 1: on )",
  },
  {
    "adv",
    command_ble_adv,
    "< 0-1 >    ( 0: off, 1: on )",
  },
};

struct fluoride_cmd_table_s
{
  const char            *name;
  int                    msize;
  struct fluoride_cmd_s *commands;
};

struct fluoride_cmd_table_s g_cmd_tables[] =
{
  { "bta", ARRAY_SIZE(g_bta_cmds), g_bta_cmds },
#ifdef CONFIG_FLUORIDE_BLE_ENABLED
  { "ble", ARRAY_SIZE(g_ble_cmds), g_ble_cmds }
#endif
};

void fluoride_shell_help(void)
{
  struct fluoride_cmd_s *commands;
  unsigned int i;
  int j;

  for (i = 0; i < ARRAY_SIZE(g_cmd_tables); i++) {
    commands = g_cmd_tables[i].commands;
    printf("----------------------------------------\n");
    for (j = 0; j < g_cmd_tables[i].msize; j++) {
      if (commands[j].help)
        printf("[%02d]: %s %16s : %s\n", j, g_cmd_tables[i].name,
                                         commands[j].cmd, commands[j].help);
    }
  }
}

int fluoride_shell(struct fluoride_s *flrd, int argc, char **argv)
{
  struct fluoride_cmd_s *commands;
  unsigned int size = 0;
  unsigned int i;
  int ret = -1;

  while (!btif_is_enabled())
    usleep(100 * 1000);

  if (argc == 1)
    goto bail;

  for (i = 0; i < ARRAY_SIZE(g_cmd_tables); i++) {
    if (!strncmp(argv[0], g_cmd_tables[i].name, strlen(g_cmd_tables[i].name))) {
      commands = g_cmd_tables[i].commands;
      size = g_cmd_tables[i].msize;
      break;
    }
  }

  for (i = 0; i < size; i++) {
    if (!strncmp(commands[i].cmd, argv[1], strlen(commands[i].cmd))) {
      ret = commands[i].func(flrd, argc - 2, &argv[2]);
      break;
    }
  }

bail:
  if (ret < 0) {
    printf("%s shell commands :\n", argv[0]);
    fluoride_shell_help();
  }

  return ret;
}
