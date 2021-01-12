/******************************************************************************
 *
 *  Copyright 2014 Google, Inc.
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
#include <base/logging.h>
#include <stdlib.h>
#include <string.h>

#include "osi/include/allocation_tracker.h"
#include "osi/include/allocator.h"

char* osi_strdup(const char* str) {
  return strdup(str);
}

char* osi_strndup(const char* str, size_t len) {
  return strndup(str, len);
}

void* osi_malloc(size_t size) {
  return malloc(size);
}

void* osi_calloc(size_t size) {
  return calloc(1, size);
}

void osi_free(void* ptr) {
  free(ptr);
}

void osi_free_and_reset(void** p_ptr) {
  free(*p_ptr);
  *p_ptr = NULL;
}

const allocator_t allocator_calloc = {osi_calloc, osi_free};
const allocator_t allocator_malloc = {osi_malloc, osi_free};
