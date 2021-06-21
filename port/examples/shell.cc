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

const bluetooth::Uuid kCCCDescriptorUuid      = bluetooth::Uuid::FromString("2902");
const bluetooth::Uuid kHRServiceUuid          = bluetooth::Uuid::FromString("180D");
const bluetooth::Uuid kHRMeasurementUuid      = bluetooth::Uuid::FromString("2A37");
const bluetooth::Uuid kBodySensorLocationUuid = bluetooth::Uuid::FromString("2A38");
const bluetooth::Uuid kHRControlPointUuid     = bluetooth::Uuid::FromString("2A39");

typedef int (*flrdcmd_func)(struct fluoride_s *flrd, int argc, char **argv);

struct fluoride_cmd_s
{
  const char     *cmd;
  flrdcmd_func    func;
  const char     *help;
};

static void fluoride_avrcp_help(void)
{
  unsigned int i;

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

  if (flrd->hfc)
    flrd->hfc->disconnect(&flrd->addr);

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

static int fluoride_hkey(struct fluoride_s *flrd, int argc, char **argv)
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

static int fluoride_hmkey(struct fluoride_s *flrd, int argc, char **argv)
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

static int fluoride_hckey(struct fluoride_s *flrd, int argc, char **argv)
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
  {
    "hkey",
    fluoride_hkey,
    "< string > ( HID keyboard Usage )",
  },
  {
    "hmkey",
    fluoride_hmkey,
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
    fluoride_hckey,
    "< value  > ( HID Control key )\n"
    "E.G:\n"
    "\tVoleme Up  :  16 (0x10)\n"
    "\tVoleme Down:  32 (0x20)\n",
  },
};

static int fluoride_command_blescan(struct fluoride_s *flrd, int argc, char **argv)
{
  if (argc == 0)
    return -1;

  flrd->gatt->scanner->Scan(!!atoi(argv[0]));

  return 0;
}

static void ble_advertising_enable_cb(uint8_t rid, bool enable, uint8_t status) TRACE_CALLBACK_BODY
static void ble_advertising_set_timeout_cb(uint8_t advertiser_id, uint8_t status) TRACE_CALLBACK_BODY
static void ble_advertising_set_started_cb(int reg_id,
    uint8_t advertiser_id, int8_t tx_power, uint8_t status)
{
  struct fluoride_s *flrd = fluoride_interface_get();
  flrd->aid = advertiser_id;

  flrd->gatt->advertiser->Enable(flrd->aid, true,
      base::Bind(&ble_advertising_enable_cb, flrd->aid, true), 0, 0,
      base::Bind(&ble_advertising_enable_cb, flrd->aid, false));
}

static int fluoride_command_bleadv(struct fluoride_s *flrd, int argc, char **argv)
{
  PeriodicAdvertisingParameters pparams = {};
  std::string name("Fluoride BLE");
  AdvertiseParameters params = {};
  std::vector<uint8_t> sdata;
  std::vector<uint8_t> pdata;
  int reg_id = -2;

  /* Advertising data: 16-bit Service Uuid: Heart Rate Service, Tx power */

  std::vector<uint8_t> adata { 0x03, bluetooth::kEIRTypeComplete16BitUuids,
    0x0D, 0x18, 0x02, bluetooth::kEIRTypeTxPower, 0x00 };

  if (argc == 0)
    return -1;

  if (atoi(argv[0]) == 0) {
    if (flrd->aid >= 0) {
      flrd->gatt->advertiser->Enable(flrd->aid, false,
          base::Bind(&ble_advertising_enable_cb, flrd->aid, true), 0, 0,
          base::Bind(&ble_advertising_enable_cb, flrd->aid, false));
    }
    return 0;
  }

  if (!flrd->element)
    {
      flrd->element = (btgatt_db_element_t *)calloc(5, sizeof(btgatt_db_element_t));
      if (!flrd->element)
        return -ENOMEM;

      flrd->element_size = 5;

      flrd->element[0].uuid        = kHRServiceUuid;
      flrd->element[0].type        = BTGATT_DB_PRIMARY_SERVICE;
      flrd->element[1].uuid        = kHRMeasurementUuid;
      flrd->element[1].type        = BTGATT_DB_CHARACTERISTIC;
      flrd->element[1].properties  = bluetooth::kCharacteristicPropertyNotify;
      flrd->element[2].uuid        = kCCCDescriptorUuid;
      flrd->element[2].type        = BTGATT_DB_DESCRIPTOR;
      flrd->element[2].permissions = bluetooth::kAttributePermissionRead |
                                     bluetooth::kAttributePermissionWrite;
      flrd->element[3].uuid        = kBodySensorLocationUuid;
      flrd->element[3].type        = BTGATT_DB_CHARACTERISTIC;
      flrd->element[3].properties  = bluetooth::kCharacteristicPropertyRead;
      flrd->element[3].permissions = bluetooth::kAttributePermissionRead;
      flrd->element[4].uuid        = kHRControlPointUuid;
      flrd->element[4].type        = BTGATT_DB_CHARACTERISTIC;
      flrd->element[4].properties  = bluetooth::kCharacteristicPropertyWrite;
      flrd->element[4].permissions = bluetooth::kAttributePermissionWrite;
    }

  if (flrd->sid == Uuid::kEmpty) {
    flrd->sid = bluetooth::Uuid::GetRandom();
    flrd->gatt->server->register_server(flrd->sid, false);
  }

  params.advertising_event_properties     = 0x13;
  params.min_interval                     = 160;
  params.max_interval                     = 240;
  params.channel_map                      = 0x07; /* all channels */
  params.tx_power                         = -15;
  params.primary_advertising_phy          = 1;
  params.secondary_advertising_phy        = 1;
  params.scan_request_notification_enable = false;

  pparams.enable                          = false;
  pparams.min_interval                    = 80;
  pparams.max_interval                    = 80 + 16; /* 20ms difference betwen min and max */

  adata.push_back(name.length() + 1);
  adata.push_back(bluetooth::kEIRTypeCompleteLocalName);
  adata.insert(adata.end(), name.c_str(), name.c_str() + name.length());

  flrd->gatt->advertiser->StartAdvertisingSet(reg_id,
      base::Bind(&ble_advertising_set_started_cb, reg_id), params, adata,
      sdata, pparams, pdata, UINT16_MAX, 0, base::Bind(ble_advertising_set_timeout_cb));

  return 0;
}

static struct fluoride_cmd_s g_ble_cmds[] =
{
  {
    "scan",
    fluoride_command_blescan,
    "< 0-1 >    ( 0: off, 1: on )",
  },
  {
    "adv",
    fluoride_command_bleadv,
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

  pthread_mutex_lock(&flrd->mutex);

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

  pthread_mutex_unlock(&flrd->mutex);

  return ret;
}
