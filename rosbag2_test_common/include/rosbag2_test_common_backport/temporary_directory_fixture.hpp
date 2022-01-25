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

#ifndef ROSBAG2_TEST_COMMON_BACKPORT__TEMPORARY_DIRECTORY_FIXTURE_HPP_
#define ROSBAG2_TEST_COMMON_BACKPORT__TEMPORARY_DIRECTORY_FIXTURE_HPP_

#include <gmock/gmock.h>

#include <string>

#include "rcpputils/filesystem_helper.hpp"

using namespace ::testing;  // NOLINT

namespace rosbag2_test_common
{

class TemporaryDirectoryFixture : public Test
{
public:
  TemporaryDirectoryFixture()
  {
    std::string full_template_str =
      (rcpputils::fs::temp_directory_path() / "tmp_test_dir_XXXXXX").string();
#ifdef _WIN32
    const char * dir_name = _mktemp(&full_template_str[0]);
#else
    const char * dir_name = mkdtemp(&full_template_str[0]);
#endif
    if (dir_name == nullptr) {
      std::error_code ec{errno, std::system_category()};
      errno = 0;
      throw std::system_error(ec, "could not format or create the temp directory");
    }
#ifdef _WIN32
    if (_mkdir(dir_name) != 0) {
      std::error_code ec{errno, std::system_category()};
      errno = 0;
      throw std::system_error(ec, "could not create the temp directory");
    }
#endif
    temporary_dir_path_ = dir_name;
  }

  ~TemporaryDirectoryFixture() override
  {
    rcpputils::fs::remove_all(rcpputils::fs::path(temporary_dir_path_));
  }

  std::string temporary_dir_path_;
};

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON_BACKPORT__TEMPORARY_DIRECTORY_FIXTURE_HPP_
