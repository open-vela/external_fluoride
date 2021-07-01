/******************************************************************************
 *
 *  Copyright 2019 Google, Inc.
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

#pragma once

#include <cstdlib>

#ifndef LOG_TAG
#define LOG_TAG "bluetooth"
#endif

static_assert(LOG_TAG != nullptr, "LOG_TAG should never be NULL");

#if defined(OS_ANDROID)

#include <log/log.h>

#ifndef OSI_INCLUDE_LOG_H
// Do not use GD include path if included from legacy OSI layer
#include "common/init_flags.h"
#endif

#ifdef FUZZ_TARGET
#define LOG_VERBOSE(...)
#define LOG_DEBUG(...)
#define LOG_INFO(...)
#define LOG_WARN(...)
#else

static_assert(LOG_TAG != nullptr, "LOG_TAG is null after header inclusion");

#define LOG_VERBOSE(fmt, args...)                                             \
  do {                                                                        \
    if (bluetooth::common::InitFlags::IsDebugLoggingEnabledForTag(LOG_TAG)) { \
      ALOGV("%s:%d %s: " fmt, __FILE__, __LINE__, __func__, ##args);          \
    }                                                                         \
  } while (false)

#define LOG_DEBUG(fmt, args...)                                               \
  do {                                                                        \
    if (bluetooth::common::InitFlags::IsDebugLoggingEnabledForTag(LOG_TAG)) { \
      ALOGD("%s:%d %s: " fmt, __FILE__, __LINE__, __func__, ##args);          \
    }                                                                         \
  } while (false)

#define LOG_INFO(fmt, args...) ALOGI("%s:%d %s: " fmt, __FILE__, __LINE__, __func__, ##args)
#define LOG_WARN(fmt, args...) ALOGW("%s:%d %s: " fmt, __FILE__, __LINE__, __func__, ##args)
#endif /* FUZZ_TARGET */
#define LOG_ERROR(fmt, args...) ALOGE("%s:%d %s: " fmt, __FILE__, __LINE__, __func__, ##args)

#else

/* syslog didn't work well here since we would be redefining LOG_DEBUG. */
#include <chrono>
#include <cstdio>
#include <ctime>

#if !defined(CONFIG_FLUORIDE_LOG_LEVEL)
#define LOGWRAPPER(level, fmt, args...)                                                                                    \
  do {                                                                                                              \
    auto _now = std::chrono::system_clock::now();                                                                   \
    auto _now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(_now);                                   \
    auto _now_t = std::chrono::system_clock::to_time_t(_now);                                                       \
    /* YYYY-MM-DD_HH:MM:SS.sss is 23 byte long, plus 1 for null terminator */                                       \
    char _buf[24];                                                                                                  \
    auto l = std::strftime(_buf, sizeof(_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&_now_t));                       \
    snprintf(                                                                                                       \
        _buf + l, sizeof(_buf) - l, ".%03u", static_cast<unsigned int>(_now_ms.time_since_epoch().count() % 1000)); \
    fprintf(stderr, "%s %s - %s:%d - %s: " fmt "\n", _buf, LOG_TAG, __FILE__, __LINE__, __func__, ##args);          \
  } while (false)
#else
#include <syslog.h>
#define LOGWRAPPER(level, fmt, args...) syslog(level, fmt "\n", ##args)
#endif

#undef LOG_VERBOSE
#undef LOG_DEBUG
#undef LOG_INFO
#undef LOG_WARNING
#undef LOG_ERROR
#undef LOG_FATAL

#if defined(CONFIG_FLUORIDE_LOG_LEVEL)
#define LOG_TRACE_TYPE_ERROR   0x00000003
#define LOG_TRACE_TYPE_WARNING 0x00000004
#define LOG_TRACE_TYPE_API     0x00000006
#define LOG_TRACE_TYPE_EVENT   0x00000006
#define LOG_TRACE_TYPE_DEBUG   0x00000007
#define LOG_TRACE_TYPE_VERBOSE 0x00000007

#define LOG_VERBOSE(...) \
  do { if (CONFIG_FLUORIDE_LOG_LEVEL >= LOG_TRACE_TYPE_VERBOSE) LOGWRAPPER(LOG_TRACE_TYPE_VERBOSE, __VA_ARGS__); } while (false)
#define LOG_DEBUG(...) \
  do { if (CONFIG_FLUORIDE_LOG_LEVEL >= LOG_TRACE_TYPE_DEBUG) LOGWRAPPER(LOG_TRACE_TYPE_DEBUG, __VA_ARGS__); } while (false)
#define LOG_INFO(...) \
  do { if (CONFIG_FLUORIDE_LOG_LEVEL >= LOG_TRACE_TYPE_API) LOGWRAPPER(LOG_TRACE_TYPE_API, __VA_ARGS__); } while (false)
#define LOG_WARN(...) \
  do { if (CONFIG_FLUORIDE_LOG_LEVEL >= LOG_TRACE_TYPE_WARNING) LOGWRAPPER(LOG_TRACE_TYPE_WARNING, __VA_ARGS__); } while (false)
#define LOG_ERROR(...) \
  do { if (CONFIG_FLUORIDE_LOG_LEVEL >= LOG_TRACE_TYPE_ERROR) LOGWRAPPER(LOG_TRACE_TYPE_ERROR, __VA_ARGS__); } while (false)
#define LOG_FATAL(...) \
  do { if (CONFIG_FLUORIDE_LOG_LEVEL >= LOG_TRACE_TYPE_ERROR) LOGWRAPPER(LOG_TRACE_TYPE_ERROR, __VA_ARGS__); } while (false)
#else
#ifdef FUZZ_TARGET
#define LOG_VERBOSE(...)
#define LOG_DEBUG(...)
#define LOG_INFO(...)
#define LOG_WARN(...)
#else
#define LOG_VERBOSE(...) LOGWRAPPER(__VA_ARGS__)
#define LOG_DEBUG(...) LOGWRAPPER(__VA_ARGS__)
#define LOG_INFO(...) LOGWRAPPER(__VA_ARGS__)
#define LOG_WARN(...) LOGWRAPPER(__VA_ARGS__)
#endif /* FUZZ_TARGET */
#define LOG_ERROR(...) LOGWRAPPER(__VA_ARGS__)
#endif /* CONFIG_FLUORIDE_LOG_LEVEL */
#define LOG_ALWAYS_FATAL(...) \
  do {                        \
    LOGWRAPPER(LOG_TRACE_TYPE_ERROR, __VA_ARGS__);  \
    abort();                  \
  } while (false)
#define android_errorWriteLog(tag, subTag) LOG_ERROR("ERROR tag: 0x%x, sub_tag: %s", tag, subTag)
#define LOG_EVENT_INT(...)

#endif /* defined(OS_ANDROID) */

#undef ASSERT

#define ASSERT(condition)                                    \
  do {                                                       \
    if (!(condition)) {                                      \
      LOG_ALWAYS_FATAL("assertion '" #condition "' failed"); \
    }                                                        \
  } while (false)

#define ASSERT_LOG(condition, fmt, args...)                                 \
  do {                                                                      \
    if (!(condition)) {                                                     \
      LOG_ALWAYS_FATAL("assertion '" #condition "' failed - " fmt, ##args); \
    }                                                                       \
  } while (false)
