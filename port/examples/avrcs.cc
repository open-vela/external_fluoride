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

class AvrcpMediaInterfaceImpl : public MediaInterface {
  public:
    void SendKeyEvent(uint8_t key, KeyState state) TRACE_CALLBACK_BODY
      void GetSongInfo(SongInfoCallback cb) override TRACE_CALLBACK_BODY
      void GetPlayStatus(PlayStatusCallback cb) override TRACE_CALLBACK_BODY
      void GetNowPlayingList(NowPlayingCallback cb) override TRACE_CALLBACK_BODY
      void GetMediaPlayerList(MediaListCallback cb) override TRACE_CALLBACK_BODY
      void GetFolderItems(uint16_t player_id, std::string media_id, FolderItemsCallback folder_cb) override TRACE_CALLBACK_BODY
      void SetBrowsedPlayer(uint16_t player_id, SetBrowsedPlayerCallback browse_cb) override TRACE_CALLBACK_BODY
      void RegisterUpdateCallback(MediaCallbacks *callback) override TRACE_CALLBACK_BODY
      void UnregisterUpdateCallback(MediaCallbacks *callback) override TRACE_CALLBACK_BODY
      void PlayItem(uint16_t player_id, bool now_playing, std::string media_id) override TRACE_CALLBACK_BODY
      void SetActiveDevice(const RawAddress& address) override TRACE_CALLBACK_BODY
};

class VolumeInterfaceImpl : public VolumeInterface {
  public:
    void DeviceConnected(const RawAddress& bdaddr) override TRACE_CALLBACK_BODY
    void DeviceConnected(const RawAddress& bdaddr, VolumeChangedCb cb) override TRACE_CALLBACK_BODY
    void DeviceDisconnected(const RawAddress& bdaddr) override TRACE_CALLBACK_BODY
    void SetVolume(int8_t volume) override TRACE_CALLBACK_BODY
};

static AvrcpMediaInterfaceImpl mAvrcpInterface;
static VolumeInterfaceImpl     mVolumeInterface;

ServiceInterface *bt_profile_avrcp_service_init(struct fluoride_s *flrd)
{
  ServiceInterface *avrcs;

  avrcs = flrd->interface->get_avrcp_service();
  if (avrcs == NULL)
    return avrcs;

  avrcs->Init(&mAvrcpInterface, &mVolumeInterface);

  return avrcs;
}
