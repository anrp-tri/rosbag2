// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2_CPP_BACKPORT__STORAGE_OPTIONS_HPP_
#define ROSBAG2_CPP_BACKPORT__STORAGE_OPTIONS_HPP_

#include "rosbag2_storage_backport/storage_options.hpp"

namespace rosbag2_cpp
{

using StorageOptions [[deprecated("use rosbag2_storage::StorageOptions instead")]] =
  rosbag2_storage::StorageOptions;

}  // namespace rosbag2_cpp
#endif  // ROSBAG2_CPP_BACKPORT__STORAGE_OPTIONS_HPP_
