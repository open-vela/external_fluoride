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

#include "main/shim/config.h"

namespace bluetooth {
  namespace shim {

    bool BtifConfigInterface::HasSection(const std::string& section) { return false; }
    bool BtifConfigInterface::HasProperty(const std::string& section, const std::string& property) { return false; }
    bool BtifConfigInterface::GetInt(const std::string& section, const std::string& property, int* value) { return false; }
    bool BtifConfigInterface::SetInt(const std::string& section, const std::string& property, int value) { return true; }
    bool BtifConfigInterface::GetUint64(const std::string& section, const std::string& property, uint64_t* value) { return false; }
    bool BtifConfigInterface::SetUint64(const std::string& section, const std::string& property, uint64_t value) { return true; }
    bool BtifConfigInterface::GetStr(const std::string& section, const std::string& property, char* value, int* size_bytes) { return false; }
    std::optional<std::string> BtifConfigInterface::GetStr( const std::string& section, const std::string& property) { return std::nullopt; }
    bool BtifConfigInterface::SetStr(const std::string& section, const std::string& property, const std::string& value) { return true; }
    bool BtifConfigInterface::GetBin(const std::string& section, const std::string& property, uint8_t* value, size_t* length) { return true; }
    size_t BtifConfigInterface::GetBinLength(const std::string& section, const std::string& property) { return 0; }
    bool BtifConfigInterface::SetBin(const std::string& section, const std::string& property, const uint8_t* value, size_t length) { return true; }
    bool BtifConfigInterface::RemoveProperty(const std::string& section, const std::string& property) { return true; }
    std::vector<std::string> BtifConfigInterface::GetPersistentDevices() { std::vector<std::string> paired_devices; return paired_devices; }
    void BtifConfigInterface::Save() { }
    void BtifConfigInterface::Flush() { }
    void BtifConfigInterface::Clear() { }

  }  // namespace shim
}  // namespace bluetooth
