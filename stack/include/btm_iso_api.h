/*
 * Copyright 2020 HIMSA II K/S - www.himsa.com.
 * Represented by EHIMA - www.ehima.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "btm_iso_api_types.h"

namespace bluetooth {
namespace hci {
namespace iso_manager {
struct CigCallbacks {
  virtual ~CigCallbacks() = default;
  virtual void OnSetupIsoDataPath(uint8_t status, uint16_t conn_handle,
                                  uint8_t cig_id) = 0;
  virtual void OnRemoveIsoDataPath(uint8_t status, uint16_t conn_handle,
                                   uint8_t cig_id) = 0;

  virtual void OnCisEvent(uint8_t event, void* data) = 0;
  virtual void OnCigEvent(uint8_t event, void* data) = 0;
};
}  // namespace iso_manager

class IsoManager {
 public:
  IsoManager();
  virtual ~IsoManager();

  static IsoManager* GetInstance() {
    static IsoManager* instance = new IsoManager();
    return instance;
  }

  /**
   * Set CIG and CIS related callbacks
   *
   * <p> Shall be set by the Le Audio Unicaster implementation
   *
   * @param callbacks CigCallbacks implementation
   */
  virtual void RegisterCigCallbacks(iso_manager::CigCallbacks* callbacks) const;

  /**
   * Creates connected isochronous group (CIG) according to given params.
   *
   * @param cig_id connected isochronous group id
   * @param cig_params CIG parameters
   */
  virtual void CreateCig(uint8_t cig_id,
                         struct iso_manager::cig_create_params cig_params);

  /**
   * Reconfigures connected isochronous group (CIG) according to given params.
   *
   * @param cig_id connected isochronous group id
   * @param cig_params CIG parameters
   */
  virtual void ReconfigureCig(uint8_t cig_id,
                              struct iso_manager::cig_create_params cig_params);

  /**
   * Initiates removing of connected isochronous group (CIG).
   *
   * @param cig_id connected isochronous group id
   */
  virtual void RemoveCig(uint8_t cig_id);

  /**
   * Initiates creation of connected isochronous stream (CIS).
   *
   * @param conn_params A set of cis and acl connection handles
   */
  virtual void EstablishCis(
      struct iso_manager::cis_establish_params conn_params);

  /**
   * Initiates disconnection of connected isochronous stream (CIS).
   *
   * @param conn_handle CIS connection handle
   * @param reason HCI reason for disconnection
   */
  virtual void DisconnectCis(uint16_t conn_handle, uint8_t reason);

  /**
   * Initiates creation of isochronous data path for connected isochronous
   * stream.
   *
   * @param conn_handle handle of CIS connection
   * @param path_params iso data path parameters
   */
  virtual void SetupIsoDataPath(
      uint16_t conn_handle,
      struct iso_manager::iso_data_path_params path_params);

  /**
   * Initiates removement of isochronous data path for connected isochronous
   * stream.
   *
   * @param conn_handle handle of CIS connection
   * @param data_path_dir iso data path direction
   */
  virtual void RemoveIsoDataPath(uint16_t conn_handle, uint8_t data_path_dir);

  /**
   * Sends iso data to the controller
   *
   * @param conn_handle handle of CIS connection
   * @param data data buffer. The ownership of data is not being transferred.
   * @param data_len data buffer length
   */
  virtual void SendIsoData(uint16_t conn_handle, const uint8_t* data,
                           uint16_t data_len);

  /* Below are defined handlers called by the legacy code in btu_hcif.cc */

  /**
   * Handles Iso Data packets from the controller
   *
   * @param p_msg raw data packet. The ownership of p_msg is not being
   * transferred.
   */
  virtual void HandleIsoData(void* p_msg);

  /**
   * Handles disconnect HCI event
   *
   * <p> This callback can be called with handles other than ISO connection
   * handles.
   *
   * @param conn_handle connection handle
   * @param reason HCI reason for disconnection
   */
  virtual void HandleDisconnect(uint16_t conn_handle, uint8_t reason);

  /**
   * Handles HCI event for the number of completed packets
   *
   * @param p raw packet buffer for the event. The ownership of p is not being
   * transferred.
   * @param evt_len event packet buffer length
   */
  virtual void HandleNumComplDataPkts(uint8_t* p, uint8_t evt_len);

  /**
   * Handle CIS related HCI events
   *
   * @param sub_code ble subcode for the HCI event
   * @param params raw packet buffer for the event. The ownership of params is
   * not being transferred
   * @param length event packet buffer length
   */
  virtual void HandleHciEvent(uint8_t sub_code, uint8_t* params,
                              uint16_t length);

  /**
   * Starts the IsoManager module
   */
  void Start();

  /**
   * Stops the IsoManager module
   */
  void Stop();

 private:
  struct impl;
  std::unique_ptr<impl> pimpl_;

  DISALLOW_COPY_AND_ASSIGN(IsoManager);
};

}  // namespace hci
}  // namespace bluetooth
