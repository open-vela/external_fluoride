/*
 * Copyright 2016 The Android Open Source Project
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

/*******************************************************************************
 *
 *  Filename:      btif_uid.cc
 *
 *  Description:   Contains data structures and functions for keeping track of
 *                 socket usage per app UID.
 *
 ******************************************************************************/
#include "bt_common.h"
#include "btif_uid.h"

uid_set_t* uid_set_create(void) { return nullptr; }
void uid_set_destroy(uid_set_t* set) { }
void uid_set_add_tx(uid_set_t* set, int32_t app_uid, uint64_t bytes) { }
void uid_set_add_rx(uid_set_t* set, int32_t app_uid, uint64_t bytes) { }
bt_uid_traffic_t* uid_set_read_and_clear(uid_set_t* set) { return nullptr; }
