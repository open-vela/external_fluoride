/*
 * Copyright 2019 The Android Open Source Project
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

#include <string>

namespace bluetooth {
namespace common {

class init_flags {
	public:
	 static bool gd_core_is_enabled() { return false; }
	 static bool gd_security_is_enabled() { return false; }
	 static bool gd_advertising_is_enabled() { return false; }
	 static bool gd_scanning_is_enabled() { return false; }
	 static bool gd_acl_is_enabled() { return false; }
	 static bool gd_l2cap_is_enabled() { return false; }
	 static bool gd_hci_is_enabled() { return false; }
	 static bool gd_controller_is_enabled() { return false; }
	 static bool gatt_robust_caching_is_enabled() { return false; }
	 static bool btaa_hci_is_enabled() { return false; }
	 static bool gd_rust_is_enabled() { return false; }
	 static bool gd_link_policy_is_enabled() { return false; }
};

}  // namespace common
}  // namespace bluetooth
