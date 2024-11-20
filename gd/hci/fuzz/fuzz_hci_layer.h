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

#include "hci/command_interface.h"
#include "hci/hci_layer.h"
#include "os/fuzz/dev_null_queue.h"
#include "os/fuzz/fuzz_inject_queue.h"
#include "os/log.h"

#include <fuzzer/FuzzedDataProvider.h>
#include "fuzz/helpers.h"

namespace bluetooth {
namespace hci {
namespace fuzz {

template <typename T>
class FuzzCommandInterface : public CommandInterface<T> {
 public:
  void EnqueueCommand(std::unique_ptr<T> command,
                      common::ContextualOnceCallback<void(hci::CommandCompleteView)> on_complete) override {}

  void EnqueueCommand(std::unique_ptr<T> command,
                      common::ContextualOnceCallback<void(hci::CommandStatusView)> on_status) override {}
};

class FuzzHciLayer : public HciLayer {
 public:
  void TurnOnAutoReply(FuzzedDataProvider* fdp) {
    auto_reply_fdp = fdp;
  }

  void TurnOffAutoReply() {
    auto_reply_fdp = nullptr;
  }

  void EnqueueCommand(std::unique_ptr<hci::CommandPacketBuilder> command,
                      common::ContextualOnceCallback<void(hci::CommandCompleteView)> on_complete) override {
    on_command_complete_ = std::move(on_complete);
    if (auto_reply_fdp != nullptr) {
      injectCommandComplete(bluetooth::fuzz::GetArbitraryBytes(auto_reply_fdp));
    }
  }

  void EnqueueCommand(std::unique_ptr<CommandPacketBuilder> command,
                      common::ContextualOnceCallback<void(hci::CommandStatusView)> on_status) override {
    on_command_status_ = std::move(on_status);
    if (auto_reply_fdp != nullptr) {
      injectCommandStatus(bluetooth::fuzz::GetArbitraryBytes(auto_reply_fdp));
    }
  }

  common::BidiQueueEnd<hci::AclPacketBuilder, hci::AclPacketView>* GetAclQueueEnd() override {
    return acl_queue_.GetUpEnd();
  }

  void RegisterEventHandler(hci::EventCode event,
                            common::ContextualCallback<void(hci::EventPacketView)> handler) override {
    event_handlers_[event] = handler;
  }

  void UnregisterEventHandler(hci::EventCode event) override {
    auto it = event_handlers_.find(event);
    if (it != event_handlers_.end()) {
      event_handlers_.erase(it);
    }
  }

  void RegisterLeEventHandler(hci::SubeventCode event,
                              common::ContextualCallback<void(hci::LeMetaEventView)> handler) override {
    le_event_handlers_[event] = handler;
  }

  void UnregisterLeEventHandler(hci::SubeventCode event) override {
    auto it = le_event_handlers_.find(event);
    if (it != le_event_handlers_.end()) {
      le_event_handlers_.erase(it);
    }
  }

  hci::SecurityInterface* GetSecurityInterface(
      common::ContextualCallback<void(hci::EventPacketView)> event_handler) override;

  hci::LeSecurityInterface* GetLeSecurityInterface(
      common::ContextualCallback<void(hci::LeMetaEventView)> event_handler) override;

  hci::AclConnectionInterface* GetAclConnectionInterface(
      common::ContextualCallback<void(hci::EventPacketView)> event_handler,
      common::ContextualCallback<void(uint16_t, hci::ErrorCode)> on_disconnect) override;

  hci::LeAclConnectionInterface* GetLeAclConnectionInterface(
      common::ContextualCallback<void(hci::LeMetaEventView)> event_handler,
      common::ContextualCallback<void(uint16_t, hci::ErrorCode)> on_disconnect) override;

  hci::LeAdvertisingInterface* GetLeAdvertisingInterface(
      common::ContextualCallback<void(hci::LeMetaEventView)> event_handler) override;

  hci::LeScanningInterface* GetLeScanningInterface(
      common::ContextualCallback<void(hci::LeMetaEventView)> event_handler) override;

  void injectArbitrary(FuzzedDataProvider& fdp);

  std::string ToString() const override {
    return "FuzzHciLayer";
  }

  static const ModuleFactory Factory;

 protected:
  void ListDependencies(ModuleList* list) override {}
  void Start() override;
  void Stop() override;

 private:
  void injectAclData(std::vector<uint8_t> data);

  void injectCommandComplete(std::vector<uint8_t> data);
  void injectCommandStatus(std::vector<uint8_t> data);

  void injectEvent(FuzzedDataProvider& fdp);
  void injectLeEvent(FuzzedDataProvider& fdp);

  void injectSecurityEvent(std::vector<uint8_t> data);
  void injectLeSecurityEvent(std::vector<uint8_t> data);

  void injectAclEvent(std::vector<uint8_t> data);
  void injectAclDisconnect(FuzzedDataProvider& fdp);
  void injectLeAclEvent(std::vector<uint8_t> data);
  void injectLeAclDisconnect(FuzzedDataProvider& fdp);

  void injectLeAdvertisingEvent(std::vector<uint8_t> data);

  void injectLeScanningEvent(std::vector<uint8_t> data);

  FuzzedDataProvider* auto_reply_fdp;

  common::BidiQueue<hci::AclPacketView, hci::AclPacketBuilder> acl_queue_{3};
  os::fuzz::DevNullQueue<AclPacketBuilder>* acl_dev_null_;
  os::fuzz::FuzzInjectQueue<AclPacketView>* acl_inject_;

  FuzzCommandInterface<ConnectionManagementCommandBuilder> acl_connection_interface_{};
  FuzzCommandInterface<LeConnectionManagementCommandBuilder> le_acl_connection_interface_{};
  FuzzCommandInterface<SecurityCommandBuilder> security_interface_{};
  FuzzCommandInterface<LeSecurityCommandBuilder> le_security_interface_{};
  FuzzCommandInterface<LeAdvertisingCommandBuilder> le_advertising_interface_{};
  FuzzCommandInterface<LeScanningCommandBuilder> le_scanning_interface_{};

  common::ContextualOnceCallback<void(hci::CommandCompleteView)> on_command_complete_;
  common::ContextualOnceCallback<void(hci::CommandStatusView)> on_command_status_;

  std::map<hci::EventCode, common::ContextualCallback<void(hci::EventPacketView)>> event_handlers_;
  std::map<hci::SubeventCode, common::ContextualCallback<void(hci::LeMetaEventView)>> le_event_handlers_;

  common::ContextualCallback<void(hci::EventPacketView)> security_event_handler_;
  common::ContextualCallback<void(hci::LeMetaEventView)> le_security_event_handler_;
  common::ContextualCallback<void(hci::EventPacketView)> acl_event_handler_;
  common::ContextualCallback<void(uint16_t, hci::ErrorCode)> acl_on_disconnect_;
  common::ContextualCallback<void(hci::LeMetaEventView)> le_acl_event_handler_;
  common::ContextualCallback<void(uint16_t, hci::ErrorCode)> le_acl_on_disconnect_;
  common::ContextualCallback<void(hci::LeMetaEventView)> le_advertising_event_handler_;
  common::ContextualCallback<void(hci::LeMetaEventView)> le_scanning_event_handler_;
};

}  // namespace fuzz
}  // namespace hci
}  // namespace bluetooth
