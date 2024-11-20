/*
 * Copyright 2020 The Android Open Source Project
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

#include "gd/hci/acl_manager/connection_callbacks.h"
#include "gd/hci/acl_manager/le_connection_callbacks.h"
#include "gd/hci/address.h"
#include "gd/hci/address_with_type.h"
#include "gd/os/handler.h"
#include "gd/packet/raw_builder.h"

namespace bluetooth {
namespace shim {
namespace legacy {

class Acl : public hci::acl_manager::ConnectionCallbacks,
            public hci::acl_manager::LeConnectionCallbacks {
 public:
  explicit Acl(os::Handler* handler);
  ~Acl();

  void CreateClassicConnection(const bluetooth::hci::Address& address);
  void CreateLeConnection(
      const bluetooth::hci::AddressWithType& address_with_type);

  void OnLeConnectSuccess(
      hci::AddressWithType,
      std::unique_ptr<hci::acl_manager::LeAclConnection>) override;
  void OnLeConnectFail(hci::AddressWithType, hci::ErrorCode reason) override;

  void OnConnectSuccess(
      std::unique_ptr<hci::acl_manager::ClassicAclConnection>) override;
  void OnConnectFail(hci::Address, hci::ErrorCode reason) override;

  void WriteData(uint16_t hci_handle,
                 std::unique_ptr<bluetooth::packet::RawBuilder> packet);
  void OnRead();  // TODO

 private:
  os::Handler* handler_;
  struct impl;
  std::unique_ptr<impl> pimpl_;
  DISALLOW_COPY_AND_ASSIGN(Acl);
};

}  // namespace legacy
}  // namespace shim
}  // namespace bluetooth
