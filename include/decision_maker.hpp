/********************************************************************************
 * Copyright (C) 2024-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 *    Mikkel Skov Maarssø
 *    Sanath Himasekhar Konthala
 ********************************************************************************/

#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adore_dynamics_conversions.hpp"
#include "adore_map/traffic_light.hpp"
#include "adore_map_conversions.hpp"
#include "adore_math/angles.h"
#include "adore_math/distance.h"
#include "adore_ros2_msgs/msg/map.hpp"
#include "adore_ros2_msgs/msg/route.hpp"
#include "adore_ros2_msgs/msg/safety_corridor.hpp"
#include "adore_ros2_msgs/msg/state_monitor.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/traffic_signals.hpp"

#include "decision_states.hpp"
#include "json_helpers.hpp"
#include "planning/lane_follow_planner.hpp"
#include "planning/optinlc_trajectory_optimizer.hpp"
#include "planning/optinlc_trajectory_planner.hpp"
#include "planning/planning_helpers.hpp"
#include "planning/safety_corridor_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace adore
{


class DecisionMaker : public rclcpp::Node
{
private:

  // Define the priority order with a constexpr array
  constexpr static StateRequirement state_priority[] = {
    {       SAFETY_CORRIDOR,               VEHICLE_STATE_OK | SAFETY_CORRIDOR_PRESENT },
    {      REMOTE_OPERATION,                   VEHICLE_STATE_OK | WAYPOINTS_AVAILABLE },
    { REQUESTING_ASSISTANCE,                       VEHICLE_STATE_OK | NEED_ASSISTANCE },
    {          FOLLOW_ROUTE, VEHICLE_STATE_OK | ROUTE_AVAILABLE | LOCAL_MAP_AVAILABLE },
    {      FOLLOW_REFERENCE,            VEHICLE_STATE_OK | REFERENCE_TRAJECTORY_VALID },
    {            STANDSTILL,                                         VEHICLE_STATE_OK },
    {        EMERGENCY_STOP,                                                        0 },
  };

  DecisionState state = EMERGENCY_STOP;

  void run();
  void update_state();

  // State dependent functions
  void emergency_stop();
  void follow_reference();
  void follow_route();
  void standstill();
  void remote_operation();
  void safety_corridor();
  void request_assistance();


  // TIMER
  rclcpp::TimerBase::SharedPtr main_timer;

  // PUBLISHERS
  rclcpp::Publisher<adore_ros2_msgs::msg::Trajectory>::SharedPtr publisher_trajectory;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            publisher_trajectory_suggestion;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            publisher_request_assistance_remote_operations;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            publisher_position_remote_operation;

  // SUBSCRIBERS
  rclcpp::Subscription<adore_ros2_msgs::msg::Trajectory>::SharedPtr            subscriber_reference_trajectory;
  rclcpp::Subscription<adore_ros2_msgs::msg::Route>::SharedPtr                 subscriber_route;
  rclcpp::Subscription<adore_ros2_msgs::msg::VehicleStateDynamic>::SharedPtr   subscriber_vehicle_state;
  rclcpp::Subscription<adore_ros2_msgs::msg::Map>::SharedPtr                   subscriber_local_map;
  rclcpp::Subscription<adore_ros2_msgs::msg::StateMonitor>::SharedPtr          subscriber_state_monitor;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                         subscriber_acknowledgement;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                         subscriber_help_requested;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                       subscriber_waypoints;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficSignals>::SharedPtr        subscriber_traffic_signals;
  rclcpp::Subscription<adore_ros2_msgs::msg::SafetyCorridor>::SharedPtr        subscriber_safety_corridor;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr subscriber_traffic_participants;

  // LATEST RECEIVED DATA
  std::optional<dynamics::Trajectory>                 latest_reference_trajectory;
  std::optional<map::Route>                           latest_route;
  std::optional<map::Map>                             latest_local_map;
  std::optional<dynamics::VehicleStateDynamic>        latest_vehicle_state;
  std::optional<adore_ros2_msgs::msg::SafetyCorridor> latest_safety_corridor;
  std::deque<adore::math::Point2d>                    latest_waypoints;
  std::optional<dynamics::TrafficParticipantSet>      latest_traffic_participants;

  bool latest_trajectory_valid();

  // CALLBACKS
  void route_callback( const adore_ros2_msgs::msg::Route& msg );
  void vehicle_state_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg );
  void local_map_callback( const adore_ros2_msgs::msg::Map& msg );
  void safety_corridor_callback( const adore_ros2_msgs::msg::SafetyCorridor& msg );
  void state_monitor_callback( const adore_ros2_msgs::msg::StateMonitor& msg );
  void waypoints_callback( const std_msgs::msg::String& waypoints );
  void requester_callback( const std_msgs::msg::Bool& msg );
  void reference_trajectory_callback( const adore_ros2_msgs::msg::Trajectory& msg );
  void traffic_signals_callback( const adore_ros2_msgs::msg::TrafficSignals& msg );
  void traffic_participants_callback( const adore_ros2_msgs::msg::TrafficParticipantSet& msg );

  // OTHER MEMBERS
  bool                           default_use_reference_trajectory_as_is = true;
  double                         dt                                     = 0.05;
  double                         remote_operation_speed                 = 2.0;
  dynamics::VehicleCommandLimits command_limits                         = { 0.7, -2.0, 2.0 };
  std::map<std::string, double>  planner_settings;

  // OptiNLC related members
  planner::OptiNLCTrajectoryOptimizer opti_nlc_trajectory_optimizer;
  planner::OptiNLCTrajectoryPlanner   opti_nlc_trajectory_planner;
  planner::SafetyCorridorPlanner      safety_corridor_planner;
  bool                                use_opti_nlc_route_following = false;

  planner::LaneFollowPlanner lane_follow_planner;

  bool need_assistance = false;

  std::vector<adore::math::Point2d> stopping_points;

  double gps_fix_standard_deviation = 99999;
  bool   debug_mode_active          = true;

  void print_init_info();
  void print_debug_info();
  void create_subscribers();
  void create_publishers();
  void load_parameters();


public:

  DecisionMaker();
};

} // namespace adore
