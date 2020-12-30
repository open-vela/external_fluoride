/******************************************************************************
 *
 *  Copyright 2019 The Android Open Source Project
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#include "common/init_flags.h"

namespace bluetooth {
  namespace common {

    bool InitFlags::btaa_hci_log_enabled = false;
    bool InitFlags::gd_core_enabled = false;
    bool InitFlags::gd_advertising_enabled = false;
    bool InitFlags::gd_security_enabled = false;
    bool InitFlags::gd_acl_enabled = false;
    bool InitFlags::gd_l2cap_enabled = false;
    bool InitFlags::gd_hci_enabled = false;
    bool InitFlags::gd_controller_enabled = false;
    bool InitFlags::gatt_robust_caching_enabled = false;
    bool InitFlags::logging_debug_enabled_for_all = false;
    std::unordered_map<std::string, bool> InitFlags::logging_debug_explicit_tag_settings = {};

    void InitFlags::Load(const char** flags) { }
    void InitFlags::SetAll(bool value) { }
    void InitFlags::SetAllForTesting() { }

  }  // namespace common
}  // namespace bluetooth
