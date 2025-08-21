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

#include "decision_states.hpp"
#include "planning/optinlc_trajectory_optimizer.hpp"
#include "planning/optinlc_trajectory_planner.hpp"
#include "planning/planning_helpers.hpp"
#include "planning/safety_corridor_planner.hpp"
#include "planning/multi_agent_PID.hpp"
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
    {      FOLLOW_REFERENCE,            VEHICLE_STATE_OK | REFERENCE_TRAJECTORY_VALID },
    {          FOLLOW_ROUTE, VEHICLE_STATE_OK | ROUTE_AVAILABLE | LOCAL_MAP_AVAILABLE },
    { MINIMUM_RISK_MANEUVER, VEHICLE_STATE_OK | ROUTE_AVAILABLE | LOCAL_MAP_AVAILABLE },
    {            STANDSTILL,                                         VEHICLE_STATE_OK },
    {        EMERGENCY_STOP,                                                        0 },
  };

  DecisionState state = EMERGENCY_STOP;

  void run();
  void update_state();

  void check_caution_zones();


  // State dependent functions
  void emergency_stop();
  void follow_reference();
  void follow_route();
  void standstill();
  void remote_operation();
  void safety_corridor();
  void request_assistance();
  void minimum_risk_maneuver();


  // TIMER
  rclcpp::TimerBase::SharedPtr main_timer;

  // PUBLISHERS
  rclcpp::Publisher<adore_ros2_msgs::msg::Trajectory>::SharedPtr         publisher_trajectory;
  rclcpp::Publisher<adore_ros2_msgs::msg::Trajectory>::SharedPtr         publisher_trajectory_suggestion;
  rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr publisher_traffic_participant_with_trajectory_prediction;
  rclcpp::Publisher<adore_ros2_msgs::msg::AssistanceRequest>::SharedPtr  publisher_request_assistance_remote_operations;
  rclcpp::Publisher<adore_ros2_msgs::msg::CautionZone>::SharedPtr        publisher_caution_zones;
  rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipant>::SharedPtr publisher_traffic_participant;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_overview;


  // SUBSCRIBERS
  rclcpp::Subscription<adore_ros2_msgs::msg::Route>::SharedPtr                 subscriber_route;
  rclcpp::Subscription<adore_ros2_msgs::msg::GoalPoint>::SharedPtr             subscriber_goal;
  rclcpp::Subscription<adore_ros2_msgs::msg::VehicleStateDynamic>::SharedPtr   subscriber_vehicle_state;
  rclcpp::Subscription<adore_ros2_msgs::msg::Map>::SharedPtr                   subscriber_local_map;
  rclcpp::Subscription<adore_ros2_msgs::msg::VehicleInfo>::SharedPtr           subscriber_vehicle_info;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                         subscriber_suggested_trajectory_acceptance;
  rclcpp::Subscription<adore_ros2_msgs::msg::Waypoints>::SharedPtr             subscriber_waypoints;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficSignals>::SharedPtr        subscriber_traffic_signals;
  rclcpp::Subscription<adore_ros2_msgs::msg::SafetyCorridor>::SharedPtr        subscriber_safety_corridor;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr subscriber_traffic_participant_set;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr subscriber_infrastructure_traffic_participants;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_user_input;

  // LATEST RECEIVED DATA
  std::optional<dynamics::Trajectory>                 latest_reference_trajectory = std::nullopt;
  std::optional<map::Route>                           latest_route                = std::nullopt;
  std::optional<map::Map>                             latest_local_map            = std::nullopt;
  std::optional<dynamics::VehicleStateDynamic>        latest_vehicle_state        = std::nullopt;
  std::optional<adore_ros2_msgs::msg::SafetyCorridor> latest_safety_corridor      = std::nullopt;
  std::optional<adore_ros2_msgs::msg::VehicleInfo>    latest_vehicle_info         = std::nullopt;
  std::deque<adore::math::Point2d>                    latest_waypoints;
  dynamics::TrafficParticipantSet                     traffic_participants;

  double turn_off_participants_duration = 5.0;
  std::optional<double> turn_off_participants_untill = std::nullopt;

  bool latest_trajectory_valid();
  bool latest_route_valid();


  // CALLBACKS
  void route_callback( const adore_ros2_msgs::msg::Route& msg );
  void goal_callback( const adore_ros2_msgs::msg::GoalPoint& msg );
  void vehicle_state_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg );
  void local_map_callback( const adore_ros2_msgs::msg::Map& msg );
  void safety_corridor_callback( const adore_ros2_msgs::msg::SafetyCorridor& msg );
  void vehicle_info_callback( const adore_ros2_msgs::msg::VehicleInfo& msg );
  void waypoints_callback( const adore_ros2_msgs::msg::Waypoints& waypoints );
  void suggested_trajectory_acceptance_callback( const std_msgs::msg::Bool& msg );
  void traffic_signals_callback( const adore_ros2_msgs::msg::TrafficSignals& msg );
  void traffic_participants_callback( const adore_ros2_msgs::msg::TrafficParticipantSet& msg );
  void infrastructure_traffic_participants_callback( const adore_ros2_msgs::msg::TrafficParticipantSet& msg );
  void user_input_callback( const std_msgs::msg::String& msg );

  void compute_routes_for_traffic_participant_set( dynamics::TrafficParticipantSet& traffic_participant_set );
  void compute_trajectories_for_traffic_participant_set( dynamics::TrafficParticipantSet& traffic_participant_set );

  // OTHER MEMBERS
  bool                           default_use_reference_trajectory_as_is = true;
  bool                           only_follow_reference_trajectories     = false;
  double                         min_route_length                       = 4.0;
  double                         dt                                     = 0.1;
  double                         remote_operation_speed                 = 2.0;
  int                            ego_id                                 = 777; // can be changed to a standard value
  dynamics::VehicleCommandLimits command_limits                         = { 0.7, -2.0, 2.0 };
  std::map<std::string, double>  planner_settings;
  size_t                         min_reference_trajectory_size = 5;
  bool allow_remote_participant_detection = true;
  bool allow_remote_trajectory_execution = true;
  std::string overview;

  // OptiNLC related members
  planner::OptiNLCTrajectoryOptimizer opti_nlc_trajectory_optimizer;
  planner::OptiNLCTrajectoryPlanner   opti_nlc_trajectory_planner;
  planner::SafetyCorridorPlanner      safety_corridor_planner;
  planner::MultiAgentPID              multi_agent_PID_planner;
  bool                                use_opti_nlc_route_following = false;


  // remote operations
  bool need_assistance         = false;
  bool sent_suggestion         = false;
  bool sent_assistance_request = false;

  // intermediate solution for traffic lights
  std::vector<adore::math::Point2d> stopping_points;

  double gps_fix_standard_deviation = 99999;
  bool   debug_mode_active          = true;

  void print_init_info();
  void debug_info(bool print);
  void create_subscribers();
  void create_publishers();
  void load_parameters();
  void publish_traffic_participant();

  math::Point2d goal;

  std::unordered_map<std::string, math::Polygon2d> caution_zones;

  dynamics::PhysicalVehicleModel model;


public:

  explicit DecisionMaker( const rclcpp::NodeOptions& options );
};

} // namespace adore
