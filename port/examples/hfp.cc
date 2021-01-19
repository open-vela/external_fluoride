#include "fluoride.h"

static void connection_state_cb(const RawAddress* bd_addr, bthf_client_connection_state_t state, unsigned int peer_feat, unsigned int chld_feat) TRACE_CALLBACK_BODY
static void audio_state_cb(const RawAddress* bd_addr, bthf_client_audio_state_t state) TRACE_CALLBACK_BODY
static void vr_cmd_cb(const RawAddress* bd_addr, bthf_client_vr_state_t state) TRACE_CALLBACK_BODY
static void network_state_cb(const RawAddress* bd_addr, bthf_client_network_state_t state) TRACE_CALLBACK_BODY
static void network_roaming_cb(const RawAddress* bd_addr, bthf_client_service_type_t type) TRACE_CALLBACK_BODY
static void network_signal_cb(const RawAddress* bd_addr, int signal) TRACE_CALLBACK_BODY
static void battery_level_cb(const RawAddress* bd_addr, int level) TRACE_CALLBACK_BODY
static void current_operator_cb(const RawAddress* bd_addr, const char* name) TRACE_CALLBACK_BODY
static void call_cb(const RawAddress* bd_addr, bthf_client_call_t call) TRACE_CALLBACK_BODY
static void callsetup_cb(const RawAddress* bd_addr, bthf_client_callsetup_t callsetup) TRACE_CALLBACK_BODY
static void callheld_cb(const RawAddress* bd_addr, bthf_client_callheld_t callheld) TRACE_CALLBACK_BODY
static void resp_and_hold_cb(const RawAddress* bd_addr, bthf_client_resp_and_hold_t resp_and_hold) TRACE_CALLBACK_BODY
static void clip_cb(const RawAddress* bd_addr, const char* number) TRACE_CALLBACK_BODY
static void call_waiting_cb(const RawAddress* bd_addr, const char* number) TRACE_CALLBACK_BODY
static void current_calls_cb(const RawAddress* bd_addr, int index, bthf_client_call_direction_t dir, bthf_client_call_state_t state, bthf_client_call_mpty_type_t mpty, const char* number) TRACE_CALLBACK_BODY
static void volume_change_cb(const RawAddress* bd_addr, bthf_client_volume_type_t type, int volume) TRACE_CALLBACK_BODY
static void cmd_complete_cb(const RawAddress* bd_addr, bthf_client_cmd_complete_t type, int cme) TRACE_CALLBACK_BODY
static void subscriber_info_cb(const RawAddress* bd_addr, const char* name, bthf_client_subscriber_service_type_t type) TRACE_CALLBACK_BODY
static void in_band_ring_cb(const RawAddress* bd_addr, bthf_client_in_band_ring_state_t in_band) TRACE_CALLBACK_BODY
static void last_voice_tag_number_cb(const RawAddress* bd_addr, const char* number) TRACE_CALLBACK_BODY
static void ring_indication_cb(const RawAddress* bd_addr) TRACE_CALLBACK_BODY
static void unknown_event_cb(const RawAddress* bd_addr, const char* eventString) TRACE_CALLBACK_BODY

static bthf_client_callbacks_t sBluetoothHfpClientCallbacks =
{
  sizeof(sBluetoothHfpClientCallbacks),
  connection_state_cb,
  audio_state_cb,
  vr_cmd_cb,
  network_state_cb,
  network_roaming_cb,
  network_signal_cb,
  battery_level_cb,
  current_operator_cb,
  call_cb,
  callsetup_cb,
  callheld_cb,
  resp_and_hold_cb,
  clip_cb,
  call_waiting_cb,
  current_calls_cb,
  volume_change_cb,
  cmd_complete_cb,
  subscriber_info_cb,
  in_band_ring_cb,
  last_voice_tag_number_cb,
  ring_indication_cb,
  unknown_event_cb,
};

const bthf_client_interface_t *bt_profile_handsfree_init(struct fluoride_s *flrd)
{
  const bthf_client_interface_t *hfc;

  hfc = (const bthf_client_interface_t *)
    flrd->interface->get_profile_interface(BT_PROFILE_HANDSFREE_CLIENT_ID);
  if (hfc == NULL)
    return hfc;

  hfc->init(&sBluetoothHfpClientCallbacks);

  return hfc;
}
