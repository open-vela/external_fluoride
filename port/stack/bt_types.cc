/******************************************************************************
 *
 *  Copyright 2016 Google, Inc.
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

#include <string.h>
#include <string>
#include "bnep_api.h"
#include "bt_common.h"
#include "bt_types.h"

#include <bluetooth/uuid.h>
#include <include/hardware/bluetooth.h>

void _UINT32_TO_BE_STREAM(uint8_t **ptr, uint32_t u32)
{
  uint8_t *p = *ptr;

  *(p)++ = (uint8_t)((u32) >> 24);
  *(p)++ = (uint8_t)((u32) >> 16);
  *(p)++ = (uint8_t)((u32) >> 8);
  *(p)++ = (uint8_t)(u32);

  *ptr = p;
}

void _UINT24_TO_BE_STREAM(uint8_t **ptr, uint32_t u24)
{
  uint8_t *p = *ptr;
  *(p)++ = (uint8_t)((u24) >> 16);
  *(p)++ = (uint8_t)((u24) >> 8);
  *(p)++ = (uint8_t)(u24);
  *ptr = p;
}

void _UINT16_TO_BE_STREAM(uint8_t **ptr, uint16_t u16)
{
  uint8_t *p = *ptr;

  *(p)++ = (uint8_t)((u16) >> 8);
  *(p)++ = (uint8_t)(u16);

  *ptr = p;
}

void _UINT8_TO_BE_STREAM(uint8_t **ptr, uint8_t u8)
{
  uint8_t *p = *ptr;
  *(p)++ = u8;
  *ptr = p;
}

void _ARRAY_TO_BE_STREAM(uint8_t **ptr, uint8_t *a, int len)
{
  uint8_t *p = *ptr;
  int ijk;
  for (ijk = 0; ijk < (len); ijk++)
    *(p)++ = (uint8_t)(a)[ijk];

  *ptr = p;
}

void _ARRAY_TO_BE_STREAM_REVERSE(uint8_t **ptr, uint8_t *a, int len)
{
  uint8_t *p = *ptr;
  int ijk;
  for (ijk = 0; ijk < (len); ijk++)
    *(p)++ = (uint8_t)(a)[(len)-ijk - 1];

  *ptr = p;
}

void _BE_STREAM_TO_UINT8(uint8_t *u8, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  (*u8) = (uint8_t)(*(p));
  *ptr = p + 1;
}

void _BE_STREAM_TO_UINT16(uint16_t *u16, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  (*u16) = (uint16_t)(((uint16_t)(*(p)) << 8) + (uint16_t)(*((p) + 1)));
  *ptr = p + 2;
}

void _BE_STREAM_TO_UINT24(uint32_t *u32, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  (*u32) = (((uint32_t)(*((p) + 2))) + ((uint32_t)(*((p) + 1)) << 8) +
      ((uint32_t)(*(p)) << 16));
  *ptr = p + 3;
}

void _BE_STREAM_TO_UINT32(uint32_t *u32, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  (*u32) = ((uint32_t)(*((p) + 3)) + ((uint32_t)(*((p) + 2)) << 8) +
      ((uint32_t)(*((p) + 1)) << 16) + ((uint32_t)(*(p)) << 24));
  *ptr = p + 4;
}

void _BE_STREAM_TO_UINT64(uint64_t *u64, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  (*u64) = ((uint64_t)(*((p) + 7)) + ((uint64_t)(*((p) + 6)) << 8) +
      ((uint64_t)(*((p) + 5)) << 16) + ((uint64_t)(*((p) + 4)) << 24) +
      ((uint64_t)(*((p) + 3)) << 32) + ((uint64_t)(*((p) + 2)) << 40) +
      ((uint64_t)(*((p) + 1)) << 48) + ((uint64_t)(*(p)) << 56));
  *ptr = p + 8;
}

void _BE_STREAM_TO_ARRAY(uint8_t **ptr, uint8_t *a, int len)
{
  uint8_t *p = *ptr;
  int ijk;
  for (ijk = 0; ijk < (len); ijk++)
    ((uint8_t*)(a))[ijk] = *(p)++;
  *ptr = p;
}

void _UINT64_TO_BE_STREAM(uint8_t **ptr, uint64_t u64)
{
  uint8_t *p = *ptr;

  *(p)++ = (uint8_t)((u64) >> 56);
  *(p)++ = (uint8_t)((u64) >> 48);
  *(p)++ = (uint8_t)((u64) >> 40);
  *(p)++ = (uint8_t)((u64) >> 32);
  *(p)++ = (uint8_t)((u64) >> 24);
  *(p)++ = (uint8_t)((u64) >> 16);
  *(p)++ = (uint8_t)((u64) >> 8);
  *(p)++ = (uint8_t) (u64);

  *ptr = p;
}
void _UINT32_TO_STREAM(uint8_t **ptr, uint32_t u32)
{
  uint8_t *p = *ptr;

  *(p)++ = (uint8_t) (u32);
  *(p)++ = (uint8_t)((u32) >> 8);
  *(p)++ = (uint8_t)((u32) >> 16);
  *(p)++ = (uint8_t)((u32) >> 24);

  *ptr = p;
}

void _UINT24_TO_STREAM(uint8_t **ptr, uint32_t u24)
{
  uint8_t *p = *ptr;
  *(p)++ = (uint8_t) (u24);
  *(p)++ = (uint8_t)((u24) >> 8);
  *(p)++ = (uint8_t)((u24) >> 16);
  *ptr = p;
}

void _UINT16_TO_STREAM(uint8_t **ptr, uint16_t u16)
{
  uint8_t *p = *ptr;

  *(p)++ = (uint8_t) (u16);
  *(p)++ = (uint8_t)((u16) >> 8);
  *ptr = p;
}

void _UINT8_TO_STREAM(uint8_t **ptr, uint8_t u8)
{
  uint8_t *p = *ptr;
  *(p)++ = u8;
  *ptr = p;
}

void _INT8_TO_STREAM(uint8_t **ptr, int8_t u8)
{
  uint8_t *p = *ptr;
  *(p)++ = (int8_t)u8;
  *ptr = p;
}

void _ARRAY32_TO_STREAM(uint8_t **ptr, uint8_t *a)
{
  uint8_t *p = *ptr;
  int ijk;
  for (ijk = 0; ijk < 32; ijk++)
    *(p)++ = (uint8_t)(a)[31 - ijk];

  *ptr = p;
}

void _ARRAY16_TO_STREAM(uint8_t **ptr, uint8_t *a)
{
  uint8_t *p = *ptr;
  int ijk;
  for (ijk = 0; ijk < 16; ijk++)
    *(p)++ = (uint8_t)(a)[15 - ijk];

  *ptr = p;
}

void _ARRAY8_TO_STREAM(uint8_t **ptr, uint8_t *a)
{
  uint8_t *p = *ptr;
  int ijk;
  for (ijk = 0; ijk < 8; ijk++)
    *(p)++ = (uint8_t)(a)[7 - ijk];

  *ptr = p;
}

void _LAP_TO_STREAM(uint8_t **ptr, uint8_t *a)
{
  uint8_t *p = *ptr;
  int ijk;
  for (ijk = 0; ijk < LAP_LEN; ijk++)
    *(p)++ = (uint8_t)(a)[LAP_LEN - 1 - ijk];

  *ptr = p;
}

void _DEVCLASS_TO_STREAM(uint8_t **ptr, uint8_t *a)
{
  uint8_t *p = *ptr;
  int ijk;
  for (ijk = 0; ijk < DEV_CLASS_LEN; ijk++)
    *(p)++ = (uint8_t)(a)[DEV_CLASS_LEN - 1 - ijk];

  *ptr = p;
}

void _ARRAY_TO_STREAM(uint8_t **ptr, uint8_t *a, int len)
{
  uint8_t *p = *ptr;
  int ijk;
  for (ijk = 0; ijk < len; ijk++)
    *(p)++ = (uint8_t)(a)[ijk];

  *ptr = p;
}

void _REVERSE_ARRAY_TO_STREAM(uint8_t **ptr, uint8_t *a, int len)
{
  uint8_t *p = *ptr;
  int ijk;
  for (ijk = 0; ijk < (len); ijk++)
    *(p)++ = (uint8_t)(a)[(len)-1 - ijk];

  *ptr = p;
}

void _STREAM_TO_UINT32(uint32_t *u32, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  (*u32) = (((uint32_t)(*(p))) + ((((uint32_t)(*((p) + 1)))) << 8) +
      ((((uint32_t)(*((p) + 2)))) << 16) +
      ((((uint32_t)(*((p) + 3)))) << 24));
  *ptr = p + 4;
}

void _STREAM_TO_UINT64(uint64_t *u64, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  (*u64) = (((uint64_t)(*(p))) + ((((uint64_t)(*((p) + 1)))) << 8) +
      ((((uint64_t)(*((p) + 2)))) << 16) +
      ((((uint64_t)(*((p) + 3)))) << 24) +
      ((((uint64_t)(*((p) + 4)))) << 32) +
      ((((uint64_t)(*((p) + 5)))) << 40) +
      ((((uint64_t)(*((p) + 6)))) << 48) +
      ((((uint64_t)(*((p) + 7)))) << 56));
  *ptr = p + 8;
}

void _STREAM_TO_UINT24(uint32_t *u32, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  (*u32) = (((uint32_t)(*(p))) + ((((uint32_t)(*((p) + 1)))) << 8) +
      ((((uint32_t)(*((p) + 2)))) << 16));
  *ptr = p + 3;
}

void _STREAM_TO_UINT16(uint16_t *u16, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  (*u16) = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8));
  *ptr = p + 2;
}

void _STREAM_TO_UINT8(uint8_t *u8, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  (*u8) = (uint8_t)(*(p));
  *ptr = p + 1;
}

void _STREAM_TO_INT8(int8_t *u8, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  (*u8) = (int8_t)(*(p));
  *ptr = p + 1;
}

void _STREAM_TO_ARRAY32(uint8_t *a, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  int ijk;
  uint8_t* _pa = (uint8_t*)(a) + 31;
  for (ijk = 0; ijk < 32; ijk++)
    *_pa-- = *(p)++;

  *ptr = p;
}

void _STREAM_TO_ARRAY16(uint8_t *a, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  int ijk;
  uint8_t* _pa = (uint8_t*)(a) + 15;
  for (ijk = 0; ijk < 16; ijk++)
    *_pa-- = *(p)++;

  *ptr = p;
}

void _STREAM_TO_ARRAY8(uint8_t *a, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  int ijk;
  uint8_t* _pa = (uint8_t*)(a) + 7;
  for (ijk = 0; ijk < 8; ijk++)
    *_pa-- = *(p)++;
  *ptr = p;
}

void _STREAM_TO_DEVCLASS(uint8_t *a, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  int ijk;
  uint8_t* _pa = (uint8_t*)(a) + DEV_CLASS_LEN - 1;
  for (ijk = 0; ijk < DEV_CLASS_LEN; ijk++)
    *_pa-- = *(p)++;

  *ptr = p;
}

void _STREAM_TO_LAP(uint8_t *a, uint8_t **ptr)
{
  uint8_t *p = *ptr;
  int ijk;
  uint8_t* plap = (uint8_t*)(a) + LAP_LEN - 1;
  for (ijk = 0; ijk < LAP_LEN; ijk++)
    *plap-- = *(p)++;
  *ptr = p;
}

void _STREAM_TO_ARRAY(uint8_t *a, uint8_t **ptr, int len)
{
  uint8_t *p = *ptr;
  int ijk;
  for (ijk = 0; ijk < (len); ijk++)
    ((uint8_t*)(a))[ijk] = *(p)++;
  *ptr = p;
}

void _REVERSE_STREAM_TO_ARRAY(uint8_t *a, uint8_t **ptr, int len)
{
  uint8_t *p = *ptr;
  int ijk;
  uint8_t* _pa = (uint8_t*)(a) + (len)-1;
  for (ijk = 0; ijk < (len); ijk++) *_pa-- = *(p)++;
  *ptr = p;
}

void BDADDR_TO_STREAM(uint8_t*& p, const RawAddress& a)
{
  for (int ijk = 0; ijk < BD_ADDR_LEN; ijk++)
    *(p)++ = (uint8_t)(a.address)[BD_ADDR_LEN - 1 - ijk];
}

void STREAM_TO_BDADDR(RawAddress& a, uint8_t*& p)
{
  uint8_t* pbda = (uint8_t*)(a.address) + BD_ADDR_LEN - 1;
  for (int ijk = 0; ijk < BD_ADDR_LEN; ijk++) *pbda-- = *(p)++;
}

bool is_sample_ltk(const Octet16& ltk)
{
  constexpr Octet16 SAMPLE_LTK = {0xbf, 0x01, 0xfb, 0x9d, 0x4e, 0xf3, 0xbc, 0x36,
    0xd8, 0x74, 0xf5, 0x39, 0x41, 0x38, 0x68, 0x4c};
  return ltk == SAMPLE_LTK;
}

std::string bd_features_text(const BD_FEATURES& features)
{
  uint8_t len = BD_FEATURES_LEN;
  char buf[255];
  char* pbuf = buf;
  const uint8_t* b = features;
  while (len--) {
    pbuf += sprintf(pbuf, "0x%02x ", *b++);
  }
  return std::string(buf);
}
