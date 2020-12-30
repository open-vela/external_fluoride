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

#define LOG_TAG "bt_core_module"

#include <base/logging.h>
#include <dlfcn.h>
#include <string.h>

#include <mutex>
#include <unordered_map>

#include "btcore/include/module.h"
#include "common/message_loop_thread.h"
#include "osi/include/allocator.h"
#include "osi/include/log.h"
#include "osi/include/osi.h"

using bluetooth::common::MessageLoopThread;

extern const module_t stack_config_module;
extern const module_t osi_module;
extern const module_t bt_utils_module;
extern const module_t hci_module;
extern const module_t bte_logmsg_module;
extern const module_t controller_module;
extern module_t btif_config_module;
extern module_t interop_module;
extern module_t btsnoop_module;

void module_management_start(void) {}
void module_management_stop(void) {}

const module_t* get_module(const char* name)
{
  if (!strcmp(stack_config_module.name, name))
    return &stack_config_module;

  if (!strcmp(osi_module.name, name))
    return &osi_module;

  if (!strcmp(bt_utils_module.name, name))
    return &bt_utils_module;

  if (!strcmp(btif_config_module.name, name))
    return &btif_config_module;

  if (!strcmp(interop_module.name, name))
    return &interop_module;

  if (!strcmp(btsnoop_module.name, name))
    return &btsnoop_module;

  if (!strcmp(hci_module.name, name))
    return &hci_module;

  if (!strcmp(bte_logmsg_module.name, name))
    return &bte_logmsg_module;

  if (!strcmp(controller_module.name, name))
    return &controller_module;

  LOG_ERROR("%s unable to load module config '%s'.", __func__, name);

  return NULL;
}

bool module_init(const module_t* module)
{
  if (module->init)
    return module->init();
  return true;
}

bool module_start_up(const module_t* module)
{
  if (module->start_up)
    return module->start_up();
  return true;
}

void module_shut_down(const module_t* module)
{
  if (module->shut_down)
    module->shut_down();
}

void module_clean_up(const module_t* module)
{
  if (module->clean_up)
    module->clean_up();
}

class CallbackWrapper {
  public:
    explicit CallbackWrapper(const module_t* module,
        MessageLoopThread* callback_thread,
        thread_fn callback)
      : module(module),
      lifecycle_thread("bt_module_lifecycle_thread"),
      callback_thread(callback_thread),
      callback(callback),
      success(false) {}
    const module_t* module;
    MessageLoopThread lifecycle_thread;
    MessageLoopThread* callback_thread;
    thread_fn callback;
    bool success;
};

static void post_result_to_callback(std::shared_ptr<CallbackWrapper> wrapper) {
  CHECK(wrapper);
  wrapper->lifecycle_thread.ShutDown();
  wrapper->callback(wrapper->success ? FUTURE_SUCCESS : FUTURE_FAIL);
}

static void run_wrapped_start_up(std::shared_ptr<CallbackWrapper> wrapper) {
  CHECK(wrapper);
  wrapper->success = module_start_up(wrapper->module);
  wrapper->callback_thread->DoInThread(
      FROM_HERE, base::BindOnce(post_result_to_callback, wrapper));
}

void module_start_up_callbacked_wrapper(const module_t* module,
    MessageLoopThread* callback_thread,
    thread_fn callback) {
  std::shared_ptr<CallbackWrapper> wrapper =
    std::make_shared<CallbackWrapper>(module, callback_thread, callback);
  wrapper->lifecycle_thread.StartUp();
  wrapper->lifecycle_thread.DoInThread(
      FROM_HERE, base::BindOnce(run_wrapped_start_up, wrapper));
}
