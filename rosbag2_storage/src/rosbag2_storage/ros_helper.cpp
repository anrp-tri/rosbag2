// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rosbag2_storage_backport/ros_helper.hpp"

#include <memory>
#include <string>

#include "rcutils/types.h"
#include "rosbag2_storage_backport/logging.hpp"

namespace rosbag2_storage
{

static rcutils_allocator_t allocator = rcutils_get_default_allocator();

std::shared_ptr<rcutils_uint8_array_t>
make_serialized_message(const void * data, size_t size)
{
  auto serialized_message = make_empty_serialized_message(size);
  memcpy(serialized_message->buffer, data, size);
  serialized_message->buffer_length = size;

  return serialized_message;
}

std::shared_ptr<rcutils_uint8_array_t>
make_empty_serialized_message(size_t size)
{
  auto msg = new rcutils_uint8_array_t;
  *msg = rcutils_get_zero_initialized_uint8_array();
  auto ret = rcutils_uint8_array_init(msg, size, &allocator);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error(
            "Error allocating resources for serialized message: " +
            std::string(rcutils_get_error_string().str));
  }

  auto serialized_message = std::shared_ptr<rcutils_uint8_array_t>(
    msg,
    [](rcutils_uint8_array_t * msg) {
      int error = rcutils_uint8_array_fini(msg);
      delete msg;
      if (error != RCUTILS_RET_OK) {
        ROSBAG2_STORAGE_LOG_ERROR_STREAM(
          "Leaking memory. Error: " << rcutils_get_error_string().str);
      }
    });

  return serialized_message;
}

}  // namespace rosbag2_storage

#ifdef __GNUC__
#include "boost/asio.hpp"

// https://stackoverflow.com/questions/11665829/how-can-i-print-stack-trace-for-caught-exceptions-in-c-code-injection-in-c
#include <iostream>
#include <dlfcn.h>
#include <execinfo.h>
#include <typeinfo>
#include <string>
#include <memory>
#include <cxxabi.h>
#include <cstdlib>
#include <unistd.h>

namespace {
struct Dumper {
  Dumper() {
  }

  ~Dumper() {
    ios_.run();
  }

  boost::asio::io_service ios_;

  static Dumper instance_;
};

Dumper Dumper::instance_;
}  // namespace


extern "C" {
  void __cxa_throw(void *ex, void *info, void (*dest)(void *)) {
    void *last_frames[64];
    size_t last_size;

    const char* name = reinterpret_cast<const std::type_info*>(info)->name();
    
    std::ostringstream oss;
    if (name) {
      oss << "Exception type: " << name << "\n";
    } else {
      oss << "No exception type\n";
    }

    last_size = backtrace(last_frames, sizeof last_frames/sizeof(void*));
    char** symbols = backtrace_symbols(last_frames, last_size);

    for (size_t i = 0; i < last_size; i++) {
      oss << "[" << i << "] " << symbols[i] << "\n";
    }

    free(symbols);

    Dumper::instance_.ios_.post([str(oss.str())] {
      std::cerr << str;
    });

    static void (*const rethrow)(void*,void*,void(*)(void*)) __attribute__ ((noreturn)) = (void (*)(void*,void*,void(*)(void*)))dlsym(RTLD_NEXT, "__cxa_throw");
    rethrow(ex,info,dest);
  }
}

#endif
