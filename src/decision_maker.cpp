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
  load_parameters();
  setup_subscribers();
  setup_publishers();
}

void DecisionMaker::load_parameters()
{  
  v2x_id = declare_parameter( "v2x_id", v2x_id );

  std::vector<std::string> keys   = declare_parameter( "planner_settings_keys", std::vector<std::string>{} );
  std::vector<double>      values = declare_parameter( "planner_settings_values", std::vector<double>{} );

  std::map<std::string, double> planner_settings;
  if( keys.size() == values.size() )
  {
    for( size_t i = 0; i < keys.size(); ++i )
    {
      planner_settings[keys[i]] = values[i];
    }
  }

  std::string vehicle_model_file = declare_parameter( "vehicle_model_file", "" );

  auto vehicle_model    = std::make_shared<dynamics::PhysicalVehicleModel>( vehicle_model_file, false );
  auto comfort_settings = dynamics::ComfortSettings(); // default value comfort settings

  planner.set_vehicle_parameters( vehicle_model->params );
  planner.set_comfort_settings( comfort_settings );
  planner.set_parameters( planner_settings );

  // planning_params.v2x_id = node.declare_parameter( "v2x_id", planning_params.v2x_id );



  // std::cerr << "error 1" << std::endl;

  // std::string planner_parameters_file = declare_parameter( "planner_parameters_file", "" );
  // std::cerr << "path : " << planner_parameters_file << std::endl;
  // std::cerr << "error 1.1" << std::endl;
  // get_parameter<std::string>( "planner_parameters_file", planner_parameters_file );
  // std::cerr << "error 1.2" << std::endl;

  // planner.set_parameters_from_file(planner_parameters_file);
  // std::cerr << "error 2" << std::endl;

  // std::string physical_vehicle_parameters_file = declare_parameter( "physical_vehicle_parameters_file", "" );
  // physical_vehicle_parameters = dynamics::PhysicalVehicleParameters(physical_vehicle_parameters_file); 
  // std::cerr << "error 3" << std::endl;

  // std::string comfort_settings_file = declare_parameter( "comfort_settings_file", "" );
  // comfort_settings = std::make_shared<dynamics::ComfortSettings>(comfort_settings_file); 
  // std::cerr << "error 4" << std::endl;

  // planner.set_vehicle_parameters( physical_vehicle_parameters );
  // planner.set_comfort_settings( comfort_settings );
  // std::cerr << "error 5" << std::endl;

  // params = load_params( *this );
}

void DecisionMaker::setup_subscribers()
{
  timer = create_wall_timer( std::chrono::milliseconds( static_cast<int>( 100 ) ), // 10 Hz
                             std::bind( &DecisionMaker::timer_callback, this ) );

  subscriber_vehicle_state_dynamic = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>( "vehicle_state_dynamic", 1,
                                      [this](const adore_ros2_msgs::msg::VehicleStateDynamic& msg) {  latest_vehicle_state_dynamic = dynamics::conversions::to_cpp_type(msg); });

  subscriber_route = create_subscription<adore_ros2_msgs::msg::Route>( "route", 1,
                                      [this](const adore_ros2_msgs::msg::Route& msg) {  latest_route = map::conversions::to_cpp_type(msg); });

  subscriber_caution_zones = create_subscription<adore_ros2_msgs::msg::CautionZone>( "caution_zones", 1,
                                      [this](const adore_ros2_msgs::msg::CautionZone& msg) {  caution_zones[msg.label] = math::conversions::to_cpp_type(msg.polygon); });

  subscriber_remote_operator_drive_approval = create_subscription<std_msgs::msg::Bool>( "suggested_trajectory_accepted", 1,
                                      [this](const std_msgs::msg::Bool& msg) {  remote_operator_drive_approval = msg.data; });

  subscriber_suggested_remote_operator_trajectory = create_subscription<adore_ros2_msgs::msg::Trajectory>( "suggested_remote_operator_trajectory", 1,
                                      [this](const adore_ros2_msgs::msg::Trajectory& msg) { 

                                        if ( !latest_vehicle_state_dynamic.has_value() )
                                          return;

                                        suggested_remote_operator_trajectory = dynamics::conversions::to_cpp_type(msg); 
                                        suggested_remote_operator_trajectory.value().adjust_start_time( latest_vehicle_state_dynamic.value().time );
                                       });
}

void DecisionMaker::setup_publishers()
{
  publisher_trajectory_decision = create_publisher<adore_ros2_msgs::msg::Trajectory>( "trajectory_decision", 1 );
  publisher_traffic_participant = create_publisher<adore_ros2_msgs::msg::TrafficParticipant>( "traffic_participant", 1 );
}

void DecisionMaker::timer_callback()
{
  auto trajectory_and_signals = choose_and_plan_driving_behavior();
  publisher_trajectory_decision->publish(trajectory_and_signals.trajectory);

  publisher_traffic_participant->publish( make_default_participant() );


//  dynamics::TrafficParticipant
// make_default_participant( const Domain& domain, const PlanningParams& planning_tools )
// {
//   dynamics::TrafficParticipant participant;
//   if( domain.vehicle_state )
//     participant.state = domain.vehicle_state.value();
//   if( domain.route )
//   {
//     participant.goal_point = domain.route->destination;
//     participant.route      = domain.route.value();
//   }
//   participant.id                  = planning_tools.v2x_id;
//   participant.v2x_id              = planning_tools.v2x_id;
//   participant.classification      = dynamics::CAR;
//   participant.physical_parameters = planning_tools.vehicle_model->params;
//   return participant;
// }
 




  // @TODO, add a cleanup step, that removes old caution zones and old suggested trajectories
  
  // auto     condition_state = conditions::evaluate_conditions( domain, params.condition_params, condition_map );
  // auto     behaviour       = rules::choose_behaviour( condition_state, rules );
  // Decision decision        = behaviour_map[behaviour.value()]( domain, params.planning_params );
  // publisher.publish( *this, decision );
}

behavior::TrajectoryAndSignals DecisionMaker::choose_and_plan_driving_behavior()
{
  double time_now = now().seconds();

  bool state_ok = conditions::state_ok(latest_vehicle_state_dynamic, time_now);
  bool route_ok = conditions::route_ok(latest_vehicle_state_dynamic, latest_route);
  bool needs_remote_operator_assistance = conditions::need_remote_operator_assitance(latest_vehicle_state_dynamic, caution_zones);

  if (
      state_ok &&
      route_ok &&
      needs_remote_operator_assistance
  )
  {
    return behavior::remote_operations(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                latest_route.value(),
                                traffic_participants,
                                remote_operator_drive_approval,
                                suggested_remote_operator_trajectory
    );
  }

  if (
      state_ok &&
      route_ok
  )
  {
    return behavior::driving_mission(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                latest_route.value(),
                                traffic_participants
                              );
  }

  if ( 
      state_ok
  )
  {
    return behavior::waiting_for_mission(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                traffic_participants
                              );
  }

  // @TODO Needs to be emergency
  return behavior::TrajectoryAndSignals {};
}

// behavior::TrajectoryAndSignals DecisionMaker::plan_based_on_driving_behavior(const behavior::DrivingBehavior& driving_mode)
// {
//   // There is a bunch of access to the optional values directly here, however, due to the previous checks, this should be safe

//   switch ( driving_mode )
//   {
//     case behavior::DrivingBehavior::DrivingMission:
//     {
//       return behavior::driving_mission(
//                                   planner,
//                                   latest_vehicle_state_dynamic.value(),
//                                   latest_route.value(),
//                                   traffic_participants
//                                 );
//     }
//     case behavior::DrivingBehavior::WaitingForMission:
//     {
//       return behavior::waiting_for_mission(
//                                   planner,
//                                   latest_vehicle_state_dynamic.value(),
//                                   traffic_participants
//                                 );

//     }
//     case behavior::DrivingBehavior::RemoteOperations:
//     {
//       return behavior::remote_operations(
//                                   planner,
//                                   latest_vehicle_state_dynamic.value(),
//                                   latest_route.value(),
//                                   traffic_participants
//       );
//     }
//     case behavior::DrivingBehavior::Emergency:
//     {
      
//     }
//   }
// }

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

adore_ros2_msgs::msg::TrafficParticipant DecisionMaker::make_default_participant()
{
  dynamics::TrafficParticipant participant;
  if( latest_vehicle_state_dynamic.has_value() )
    participant.state = latest_vehicle_state_dynamic.value();

  if( latest_route.has_value() )
  {
    participant.goal_point = latest_route->destination;
    participant.route      = latest_route.value();
  }

  participant.id                  = v2x_id;
  participant.v2x_id              = v2x_id;
  participant.classification      = dynamics::CAR;
  participant.physical_parameters = planner.get_physical_vehicle_parameters();

  return dynamics::conversions::to_ros_msg( participant );
}

} // namespace adore

/* Register as component --------------------------------------------- */
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::DecisionMaker )
