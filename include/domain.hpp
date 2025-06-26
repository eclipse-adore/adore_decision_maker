#pragma once
#include <deque>
#include <map>
#include <vector>

#include "adore_dynamics_adapters.hpp"
#include "adore_dynamics_conversions.hpp"
#include "adore_map/traffic_light.hpp"
#include "adore_map_adapters.hpp"
#include "adore_ros2_msgs/msg/assistance_request.hpp"
#include "adore_ros2_msgs/msg/caution_zone.hpp"
#include "adore_ros2_msgs/msg/safety_corridor.hpp"
#include "adore_ros2_msgs/msg/state_monitor.hpp"
#include "adore_ros2_msgs/msg/traffic_signals.hpp"
#include "adore_ros2_msgs/msg/waypoints.hpp"

#include "std_msgs/msg/bool.hpp"

namespace adore
{
struct Domain
{
  using CautionZones = std::map<std::string, math::Polygon2d>;

  std::optional<dynamics::VehicleStateDynamic>        vehicle_state;
  std::optional<map::Route>                           route;
  std::optional<adore_ros2_msgs::msg::SafetyCorridor> safety_corridor;
  std::optional<dynamics::VehicleInfo>                vehicle_info;
  std::optional<dynamics::Trajectory>                 suggested_trajectory;
  std::optional<dynamics::Trajectory>                 reference_trajectory;
  dynamics::TrafficParticipantSet                     traffic_participants;

  std::map<size_t, adore_ros2_msgs::msg::TrafficSignal> traffic_signals;
  std::optional<adore_ros2_msgs::msg::Waypoints>        waypoints;

  bool suggested_trajectory_acceptance;
  bool sent_assistance_request = false;

  CautionZones caution_zones;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers;

  template<typename MsgT, typename CallbackT>
  void
  add_subscription( rclcpp::Node& node, const std::string& topic_name, CallbackT&& callback )
  {
    auto sub = node.create_subscription<MsgT>( topic_name, 1, std::forward<CallbackT>( callback ) );
    subscribers.emplace_back( sub );
  }

  void setup( rclcpp::Node& node );
};
} // namespace adore
