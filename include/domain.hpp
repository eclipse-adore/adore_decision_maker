#pragma once
#include "adore_dynamics_conversions.hpp"
#include "adore_map/traffic_light.hpp"
#include "adore_map_conversions.hpp"
#include "adore_math/angles.h"
#include "adore_math/distance.h"
#include "adore_math/polygon.h"
#include "adore_ros2_msgs/msg/assistance_request.hpp"
#include "adore_ros2_msgs/msg/caution_zone.hpp"
#include "adore_ros2_msgs/msg/goal_point.hpp"
#include "adore_ros2_msgs/msg/map.hpp"
#include "adore_ros2_msgs/msg/route.hpp"
#include "adore_ros2_msgs/msg/safety_corridor.hpp"
#include "adore_ros2_msgs/msg/state_monitor.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/traffic_signals.hpp"
#include "adore_ros2_msgs/msg/vehicle_info.hpp"
#include "adore_ros2_msgs/msg/waypoints.hpp"

#include "latest.hpp"
#include "std_msgs/msg/bool.hpp"

namespace adore
{
struct Domain
{
  void setup( rclcpp::Node& n );

  Latest<dynamics::VehicleStateDynamic, adore_ros2_msgs::msg::VehicleStateDynamic>     latest_vehicle_state;
  Latest<map::Route, adore_ros2_msgs::msg::Route>                                      latest_route;
  Latest<map::Map, adore_ros2_msgs::msg::Map>                                          latest_local_map;
  Latest<adore_ros2_msgs::msg::SafetyCorridor, adore_ros2_msgs::msg::SafetyCorridor>   latest_safety_corridor;
  Latest<adore_ros2_msgs::msg::VehicleInfo, adore_ros2_msgs::msg::VehicleInfo>         latest_vehicle_info;
  Latest<dynamics::TrafficParticipantSet, adore_ros2_msgs::msg::TrafficParticipantSet> latest_traffic_participants;
  Latest<std::deque<adore::math::Point2d>, adore_ros2_msgs::msg::Waypoints>            latest_waypoints;
  Latest<std::vector<map::TrafficLight>, adore_ros2_msgs::msg::TrafficSignals>         latest_traffic_signals;
  Latest<math::Point2d, adore_ros2_msgs::msg::GoalPoint>                               latest_goal;
  Latest<bool, std_msgs::msg::Bool>                                                    latest_suggested_trajectory_acceptance;
  Latest<dynamics::Trajectory, adore_ros2_msgs::msg::Trajectory>                       latest_suggested_trajectory;
  Latest<dynamics::Trajectory, adore_ros2_msgs::msg::Trajectory>                       latest_reference_trajectory;

  bool need_assistance = false;
};
} // namespace adore