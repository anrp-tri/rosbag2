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

#ifndef ROSBAG2_TRANSPORT_BACKPORT__PLAYER_HPP_
#define ROSBAG2_TRANSPORT_BACKPORT__PLAYER_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <optional> // NOLINT  -- cpplint complains about the inclusion order.
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>

#include "moodycamel/readerwriterqueue.h"

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"

#include "rosbag2_cpp_backport/clocks/player_clock.hpp"
#include "rosbag2_interfaces_backport/srv/get_rate.hpp"
#include "rosbag2_interfaces_backport/srv/is_paused.hpp"
#include "rosbag2_interfaces_backport/srv/pause.hpp"
#include "rosbag2_interfaces_backport/srv/play_for.hpp"
#include "rosbag2_interfaces_backport/srv/play_next.hpp"
#include "rosbag2_interfaces_backport/srv/play_until.hpp"
#include "rosbag2_interfaces_backport/srv/resume.hpp"
#include "rosbag2_interfaces_backport/srv/set_rate.hpp"
#include "rosbag2_interfaces_backport/srv/toggle_paused.hpp"
#include "rosbag2_storage_backport/serialized_bag_message.hpp"
#include "rosbag2_storage_backport/storage_options.hpp"

#include "rosbag2_transport_backport/generic_publisher.hpp"
#include "rosbag2_transport_backport/play_options.hpp"
#include "rosbag2_transport_backport/visibility_control.hpp"

#include "rosgraph_msgs/msg/clock.hpp"

namespace rosbag2_cpp
{
class Reader;
}  // namespace rosbag2_cpp

namespace rosbag2_transport
{

class Player : public rclcpp::Node
{
public:
  ROSBAG2_TRANSPORT_PUBLIC
  explicit Player(
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ROSBAG2_TRANSPORT_PUBLIC
  Player(
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options,
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ROSBAG2_TRANSPORT_PUBLIC
  Player(
    std::unique_ptr<rosbag2_cpp::Reader> reader,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options,
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ROSBAG2_TRANSPORT_PUBLIC
  virtual ~Player();

  ROSBAG2_TRANSPORT_PUBLIC
  void play(const std::optional<rcutils_duration_value_t> & duration = std::nullopt);

  ROSBAG2_TRANSPORT_PUBLIC
  void play_until(const rcutils_time_point_value_t & timestamp);

  ROSBAG2_TRANSPORT_PUBLIC
  rosbag2_cpp::Reader * release_reader();

  // Playback control interface
  /// Pause the flow of time for playback.
  ROSBAG2_TRANSPORT_PUBLIC
  void pause();

  /// Start the flow of time for playback.
  ROSBAG2_TRANSPORT_PUBLIC
  void resume();

  /// Pause if time running, resume if paused.
  ROSBAG2_TRANSPORT_PUBLIC
  void toggle_paused();

  /// Return whether the playback is currently paused.
  ROSBAG2_TRANSPORT_PUBLIC
  bool is_paused() const;

  /// Return current playback rate.
  ROSBAG2_TRANSPORT_PUBLIC
  double get_rate() const;

  /// Set the playback rate.
  /**
   * Set the playback rate.
   * \return false if an invalid value was provided (<= 0).
   */
  ROSBAG2_TRANSPORT_PUBLIC
  bool set_rate(double);

  /// \brief Playing next message from queue when in pause.
  /// \param num_messages The number of messages to play from the queue. It is
  /// nullopt by default which makes it just take one. When zero, it'll just
  /// make the method return true.
  /// \details This is blocking call and it will wait until next available message will be
  /// published or rclcpp context shut down.
  /// \note If internal player queue is starving and storage has not been completely loaded,
  /// this method will wait until new element will be pushed to the queue.
  /// \return true if Player::play() has been started, player in pause mode and successfully
  /// played next message, otherwise false.
  ROSBAG2_TRANSPORT_PUBLIC
  bool play_next(const std::optional<uint64_t> num_messages = std::nullopt);

  /// \brief Play the next \p duration of messages from the queue when paused.
  /// \details This is a blocking call and it will wait up to \p duration for available messages
  /// to be published or the rclcpp context shut down.
  /// \param duration The time the player must play messages.
  /// \note If the internal player queue is starved and storage has not been completely loaded,
  /// this method will wait until a new element is pushed to the queue.
  /// \return true if Player::play() has been started, the player is in pause mode and successfully
  /// played messages (at least one) for the specified \p duration, otherwise false.
  ROSBAG2_TRANSPORT_PUBLIC
  bool play_for_the_next(const rcutils_duration_value_t duration);

protected:
  std::atomic<bool> playing_messages_from_queue_{false};
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  std::unordered_map<std::string, std::shared_ptr<GenericPublisher>> publishers_;

private:
  rosbag2_storage::SerializedBagMessageSharedPtr * peek_next_message_from_queue();
  void load_storage_content();
  bool is_storage_completely_loaded() const;
  void enqueue_up_to_boundary(uint64_t boundary);
  void wait_for_filled_queue() const;
  void play_messages_from_queue(const std::optional<rcutils_duration_value_t> & play_until_time);
  void prepare_publishers();
  bool publish_message(rosbag2_storage::SerializedBagMessageSharedPtr message);
  void do_play(
    const std::optional<rcutils_duration_value_t> & duration,
    const std::optional<rcutils_time_point_value_t> & timestamp);
  static constexpr double read_ahead_lower_bound_percentage_ = 0.9;
  static const std::chrono::milliseconds queue_read_wait_period_;

  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_transport::PlayOptions play_options_;
  moodycamel::ReaderWriterQueue<rosbag2_storage::SerializedBagMessageSharedPtr> message_queue_;
  mutable std::future<void> storage_loading_future_;
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides_;
  std::unique_ptr<rosbag2_cpp::PlayerClock> clock_;
  std::shared_ptr<rclcpp::TimerBase> clock_publish_timer_;
  std::mutex skip_message_in_main_play_loop_mutex_;
  bool skip_message_in_main_play_loop_ RCPPUTILS_TSA_GUARDED_BY
    (skip_message_in_main_play_loop_mutex_) = false;
  std::atomic_bool is_in_play_{false};

  rclcpp::Service<rosbag2_interfaces_backport::srv::Pause>::SharedPtr srv_pause_;
  rclcpp::Service<rosbag2_interfaces_backport::srv::Resume>::SharedPtr srv_resume_;
  rclcpp::Service<rosbag2_interfaces_backport::srv::TogglePaused>::SharedPtr srv_toggle_paused_;
  rclcpp::Service<rosbag2_interfaces_backport::srv::IsPaused>::SharedPtr srv_is_paused_;
  rclcpp::Service<rosbag2_interfaces_backport::srv::GetRate>::SharedPtr srv_get_rate_;
  rclcpp::Service<rosbag2_interfaces_backport::srv::SetRate>::SharedPtr srv_set_rate_;
  rclcpp::Service<rosbag2_interfaces_backport::srv::PlayNext>::SharedPtr srv_play_next_;
  rclcpp::Service<rosbag2_interfaces_backport::srv::PlayFor>::SharedPtr srv_play_for_;
  rclcpp::Service<rosbag2_interfaces_backport::srv::PlayFor>::SharedPtr srv_play_for_the_next_;
  rclcpp::Service<rosbag2_interfaces_backport::srv::PlayUntil>::SharedPtr srv_play_until_;

  template<typename AllocatorT = std::allocator<void>>
  std::shared_ptr<GenericPublisher> create_generic_publisher(
    const std::string & topic_name,
    const std::string & topic_type,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options = (
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>()
    )
  )
  {
    auto ts_lib = rosbag2_cpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
    auto pub = std::make_shared<GenericPublisher>(
      get_node_topics_interface()->get_node_base_interface(),
      std::move(ts_lib),
      topic_name,
      topic_type,
      qos,
      options);
    get_node_topics_interface()->add_publisher(pub, options.callback_group);
    return pub;
  }
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT_BACKPORT__PLAYER_HPP_
