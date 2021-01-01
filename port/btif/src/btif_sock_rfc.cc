/******************************************************************************
 *
 *  Copyright 2009-2012 Broadcom Corporation
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

#include "bt_types.h"

int bta_co_rfc_data_incoming(uint32_t id, BT_HDR* p_buf) { return 0; }
int bta_co_rfc_data_outgoing_size(uint32_t id, int* size) { return false; }
int bta_co_rfc_data_outgoing(uint32_t id, uint8_t* buf, uint16_t size) { return false; }
