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

#define LOG_TAG "bt_osi_thread"

#include "osi/include/thread.h"

#include <atomic>

#include <base/logging.h>
#include <errno.h>
#include <malloc.h>
#include <pthread.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>

#include "osi/include/allocator.h"
#include "osi/include/compat.h"
#include "osi/include/fixed_queue.h"
#include "osi/include/log.h"
#include "osi/include/reactor.h"
#include "osi/include/semaphore.h"

struct thread_t {
  std::atomic_bool is_joined{false};
  pthread_t pthread;
  pid_t tid;
  char name[THREAD_NAME_MAX + 1];
  reactor_t* reactor;
  fixed_queue_t* work_queue;
};

struct start_arg {
  thread_t* thread;
  semaphore_t* start_sem;
  int error;
};

typedef struct {
  thread_fn func;
  void* context;
} work_item_t;

static void* run_thread(void* start_arg);
static void work_queue_read_cb(void* context);

static const size_t DEFAULT_WORK_QUEUE_CAPACITY = 128;

thread_t* thread_new_sized(const char* name, size_t work_queue_capacity, size_t stack_size) {
  CHECK(name != NULL);
  CHECK(work_queue_capacity != 0);

  thread_t* ret = static_cast<thread_t*>(osi_calloc(sizeof(thread_t)));
  pthread_attr_t pattr;

  ret->reactor = reactor_new();
  if (!ret->reactor) goto error;

  ret->work_queue = fixed_queue_new(work_queue_capacity);
  if (!ret->work_queue) goto error;

  // Start is on the stack, but we use a semaphore, so it's safe
  struct start_arg start;
  start.start_sem = semaphore_new(0);
  if (!start.start_sem) goto error;

  strncpy(ret->name, name, THREAD_NAME_MAX);
  start.thread = ret;
  start.error = 0;
  pthread_attr_init(&pattr);
  if (stack_size)
    pthread_attr_setstacksize(&pattr, stack_size);
  pthread_create(&ret->pthread, &pattr, run_thread, &start);
  pthread_attr_destroy(&pattr);
  semaphore_wait(start.start_sem);
  semaphore_free(start.start_sem);

  if (start.error) goto error;

  return ret;

error:;
  if (ret) {
    fixed_queue_free(ret->work_queue, osi_free);
    reactor_free(ret->reactor);
  }
  osi_free(ret);
  return NULL;
}

thread_t* thread_new(const char* name, size_t stack_size) {
  return thread_new_sized(name, DEFAULT_WORK_QUEUE_CAPACITY, stack_size);
}

void thread_free(thread_t* thread) {
  if (!thread) return;

  thread_stop(thread);
  thread_join(thread);

  fixed_queue_free(thread->work_queue, osi_free);
  reactor_free(thread->reactor);
  osi_free(thread);
}

void thread_join(thread_t* thread) {
  CHECK(thread != NULL);

  if (!std::atomic_exchange(&thread->is_joined, true))
    pthread_join(thread->pthread, NULL);
}

bool thread_post(thread_t* thread, thread_fn func, void* context) {
  CHECK(thread != NULL);
  CHECK(func != NULL);

  // TODO(sharvil): if the current thread == |thread| and we've run out
  // of queue space, we should abort this operation, otherwise we'll
  // deadlock.

  // Queue item is freed either when the queue itself is destroyed
  // or when the item is removed from the queue for dispatch.
  work_item_t* item = (work_item_t*)osi_malloc(sizeof(work_item_t));
  item->func = func;
  item->context = context;
  fixed_queue_enqueue(thread->work_queue, item);
  return true;
}

void thread_stop(thread_t* thread) {
  CHECK(thread != NULL);
  reactor_stop(thread->reactor);
}

bool thread_set_priority(thread_t* thread, int priority) {
  if (!thread) return false;

  const int rc = setpriority(PRIO_PROCESS, thread->tid, priority);
  if (rc < 0) {
    LOG_ERROR("%s unable to set thread priority %d for tid %d, error %d",
              __func__, priority, thread->tid, rc);
    return false;
  }

  return true;
}

bool thread_set_rt_priority(thread_t* thread, int priority) {
  if (!thread) return false;

  struct sched_param rt_params;
  rt_params.sched_priority = priority;

  const int rc = sched_setscheduler(thread->tid, SCHED_FIFO, &rt_params);
  if (rc != 0) {
    LOG_ERROR("%s unable to set SCHED_FIFO priority %d for tid %d, error %s",
              __func__, priority, thread->tid, strerror(errno));
    return false;
  }

  return true;
}

bool thread_is_self(const thread_t* thread) {
  CHECK(thread != NULL);
  return !!pthread_equal(pthread_self(), thread->pthread);
}

reactor_t* thread_get_reactor(const thread_t* thread) {
  CHECK(thread != NULL);
  return thread->reactor;
}

const char* thread_name(const thread_t* thread) {
  CHECK(thread != NULL);
  return thread->name;
}

static void* run_thread(void* start_arg) {
  CHECK(start_arg != NULL);

  struct start_arg* start = static_cast<struct start_arg*>(start_arg);
  thread_t* thread = start->thread;

  CHECK(thread != NULL);

  if (prctl(PR_SET_NAME, (unsigned long)thread->name) == -1) {
    LOG_ERROR("%s unable to set thread name: %s", __func__, strerror(errno));
    start->error = errno;
    semaphore_post(start->start_sem);
    return NULL;
  }
  thread->tid = gettid();

  LOG_INFO("%s: thread id %d, thread name %s started", __func__, thread->tid,
           thread->name);

  semaphore_post(start->start_sem);

  int fd = fixed_queue_get_dequeue_fd(thread->work_queue);
  void* context = thread->work_queue;

  reactor_object_t* work_queue_object =
      reactor_register(thread->reactor, fd, context, work_queue_read_cb, NULL);
  reactor_start(thread->reactor);
  reactor_unregister(work_queue_object);

  // Make sure we dispatch all queued work items before exiting the thread.
  // This allows a caller to safely tear down by enqueuing a teardown
  // work item and then joining the thread.
  size_t count = 0;
  work_item_t* item =
      static_cast<work_item_t*>(fixed_queue_try_dequeue(thread->work_queue));
  while (item && count <= fixed_queue_capacity(thread->work_queue)) {
    item->func(item->context);
    osi_free(item);
    item =
        static_cast<work_item_t*>(fixed_queue_try_dequeue(thread->work_queue));
    ++count;
  }

  if (count > fixed_queue_capacity(thread->work_queue))
    LOG_INFO("%s growing event queue on shutdown.", __func__);

  LOG_WARN("%s: thread id %d, thread name %s exited", __func__, thread->tid,
           thread->name);
  return NULL;
}

static void work_queue_read_cb(void* context) {
  CHECK(context != NULL);

  fixed_queue_t* queue = (fixed_queue_t*)context;
  work_item_t* item = static_cast<work_item_t*>(fixed_queue_dequeue(queue));
  item->func(item->context);
  osi_free(item);
}
