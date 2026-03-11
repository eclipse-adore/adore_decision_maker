/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "decision_maker.hpp"

namespace adore
{

DecisionMaker::DecisionMaker( const rclcpp::NodeOptions& opts ) :
  rclcpp::Node{ "decision_maker", opts }

{
  latest_driving_behavior.driving_mode = adore_ros2_msgs::msg::DrivingBehavior::WAITING_FOR_MISSION;

  load_parameters();

  domain = Domain();

  // domain.setup( *this, params.domain_params, params.in_topics );
  // publisher.setup( *this, params.out_topics );
}

void DecisionMaker::load_parameters()
{  
  std::cerr << "error 1" << std::endl;
  std::string planner_parameters_file;
  declare_parameter<std::string>( "planner_parameters_file", planner_parameters_file );

  std::cerr << "path : " << planner_parameters_file << std::endl;
  planner.set_parameters_from_file(planner_parameters_file);
  std::cerr << "error 2" << std::endl;

  std::string physical_vehicle_parameters_file = declare_parameter( "physical_vehicle_parameters_file", "" );
  physical_vehicle_parameters = dynamics::PhysicalVehicleParameters(physical_vehicle_parameters_file); 
  std::cerr << "error 3" << std::endl;

  std::string comfort_settings_file = declare_parameter( "comfort_settings_file", "" );
  comfort_settings = std::make_shared<dynamics::ComfortSettings>(comfort_settings_file); 
  std::cerr << "error 4" << std::endl;

  planner.set_vehicle_parameters( physical_vehicle_parameters );
  planner.set_comfort_settings( comfort_settings );
  std::cerr << "error 5" << std::endl;

  // params = load_params( *this );
}

void DecisionMaker::setup_subscribers()
{
  timer = create_wall_timer( std::chrono::milliseconds( static_cast<int>( params.run_delta_time * 1000 ) ),
                             std::bind( &DecisionMaker::timer_callback, this ) );

  subscriber_driving_behavior = create_subscription<adore_ros2_msgs::msg::DrivingBehavior>( "driving_behavior", 1,
                                      [this](const adore_ros2_msgs::msg::DrivingBehavior& msg) {  latest_driving_behavior = msg; });

  subscriber_vehicle_state_dynamic = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>( "vehicle_state_dynamic", 1,
                                      [this](const adore_ros2_msgs::msg::VehicleStateDynamic& msg) {  latest_vehicle_state_dynamic = dynamics::conversions::to_cpp_type(msg); });

  subscriber_route = create_subscription<adore_ros2_msgs::msg::Route>( "route", 1,
                                      [this](const adore_ros2_msgs::msg::Route& msg) {  latest_route = map::conversions::to_cpp_type(msg); });
}

void DecisionMaker::setup_publishers()
{
  publisher_trajectory_decision = create_publisher<adore_ros2_msgs::msg::Trajectory>( "trajectory_decision", 1 );
}

void DecisionMaker::timer_callback()
{
  auto trajectory_and_signals = plan_based_on_driving_behavior();

  
  // auto     condition_state = conditions::evaluate_conditions( domain, params.condition_params, condition_map );
  // auto     behaviour       = rules::choose_behaviour( condition_state, rules );
  // Decision decision        = behaviour_map[behaviour.value()]( domain, params.planning_params );
  // publisher.publish( *this, decision );
}

behavior::TrajectoryAndSignals DecisionMaker::plan_based_on_driving_behavior()
{
  switch ( latest_driving_behavior.driving_mode )
  {
    case adore_ros2_msgs::msg::DrivingBehavior::DRIVE_MISSION:
    {
      return behavior::driving_mission(
                                  planner,
                                  latest_vehicle_state_dynamic,
                                  latest_route,
                                  traffic_participants
                                );
    }
    case adore_ros2_msgs::msg::DrivingBehavior::DRIVING_MISSION_CAREFULLY: // @TODO, this is for a future situation
    {
     break; 
    }
    case adore_ros2_msgs::msg::DrivingBehavior::WAITING_FOR_MISSION:
    {
     break; 
    }
    case adore_ros2_msgs::msg::DrivingBehavior::AVOIDING_SAFETY_CORRIDOR:
    {
     break; 
    }
    case adore_ros2_msgs::msg::DrivingBehavior::MINIMUM_RISK_MANOUEVRE:
    {
     break; 
    }
    case adore_ros2_msgs::msg::DrivingBehavior::EMERGENCY_MANOUEVRE:
    {
     break; 
    }
  }
}

// void
// DecisionMaker::run()
// {
//   auto     condition_state = conditions::evaluate_conditions( domain, params.condition_params, condition_map );
//   auto     behaviour       = rules::choose_behaviour( condition_state, rules );
//   Decision decision        = behaviour_map[behaviour.value()]( domain, params.planning_params );
//   publisher.publish( *this, decision );
// }

// void
// DecisionMaker::setup()
// {
//   params = load_params( *this );

//   timer = create_wall_timer( std::chrono::milliseconds( static_cast<int>( params.run_delta_time * 1000 ) ),
//                              std::bind( &DecisionMaker::run, this ) );

//   const std::string           pkg        = ament_index_cpp::get_package_share_directory( "decision_maker" );
//   const std::filesystem::path rules_path = std::filesystem::path( pkg ) / "config" / "rules.yaml";

//   std::string rules_file = declare_parameter( "rules_file", rules_path.string() );
//   rules                  = rules::load_rules_yaml( rules_file );
// }

} // namespace adore

/* Register as component --------------------------------------------- */
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::DecisionMaker )
