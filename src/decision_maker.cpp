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
#include "adore_ros2_msgs/msg/remote_operation_status.hpp"
#include "adore_ros2_msgs/msg/traffic_participant.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/passenger_request.hpp"
#include <adore_dynamics_conversions.hpp>
#include "behaviors.hpp"
#include "conditions.hpp"
#include "open_odd_ros2_msgs/msg/odd_evaluation.hpp"

#include <chrono>
#include <stdexcept>

namespace adore
{

adore::planner::ObstacleAvoidanceParams
load_obstacle_avoidance_params( rclcpp::Node& node )
{
  adore::planner::ObstacleAvoidanceParams params;

  params.enabled = node.declare_parameter<bool>( "obstacle_avoidance.enabled", params.enabled );
  params.max_static_object_speed = node.declare_parameter<double>( "obstacle_avoidance.max_static_object_speed", params.max_static_object_speed );
  params.ego_corridor_safety_margin = node.declare_parameter<double>( "obstacle_avoidance.ego_corridor_safety_margin", params.ego_corridor_safety_margin );
  params.side_clearance = node.declare_parameter<double>( "obstacle_avoidance.side_clearance", params.side_clearance );
  params.side_clearance_replan_tolerance = node.declare_parameter<double>( "obstacle_avoidance.side_clearance_replan_tolerance", params.side_clearance_replan_tolerance );
  params.front_clearance = node.declare_parameter<double>( "obstacle_avoidance.front_clearance", params.front_clearance );
  params.avoidance_lateral_accel = node.declare_parameter<double>( "obstacle_avoidance.avoidance_lateral_accel", params.avoidance_lateral_accel );
  params.rear_clearance = node.declare_parameter<double>( "obstacle_avoidance.rear_clearance", params.rear_clearance );
  params.stop_before_obstacle = node.declare_parameter<double>( "obstacle_avoidance.stop_before_obstacle", params.stop_before_obstacle );
  params.in_lane_shift_enabled = node.declare_parameter<bool>( "obstacle_avoidance.in_lane_shift_enabled", params.in_lane_shift_enabled );
  params.adjacent_lane_enabled = node.declare_parameter<bool>( "obstacle_avoidance.adjacent_lane_enabled", params.adjacent_lane_enabled );
  params.intersection_drivable_area_enabled = node.declare_parameter<bool>( "obstacle_avoidance.intersection_drivable_area_enabled", params.intersection_drivable_area_enabled );
  params.opposite_lane_enabled = node.declare_parameter<bool>( "obstacle_avoidance.opposite_lane_enabled", params.opposite_lane_enabled );
  params.max_speed_during_avoidance = node.declare_parameter<double>( "obstacle_avoidance.max_speed_during_avoidance", params.max_speed_during_avoidance );
  params.blinker_lead_distance = node.declare_parameter<double>( "obstacle_avoidance.blinker_lead_distance", params.blinker_lead_distance );
  params.prediction_time_horizon = node.declare_parameter<double>( "obstacle_avoidance.prediction_time_horizon", params.prediction_time_horizon );

  // Internal/advanced parameters
  params.min_oncoming_heading_diff = node.declare_parameter<double>( "obstacle_avoidance.min_oncoming_heading_diff", params.min_oncoming_heading_diff );
  params.prefer_left_shift = node.declare_parameter<bool>( "obstacle_avoidance.prefer_left_shift", params.prefer_left_shift );
  params.lane_s_overlap_slack = node.declare_parameter<double>( "obstacle_avoidance.lane_s_overlap_slack", params.lane_s_overlap_slack );
  params.lane_boundary_join_slack = node.declare_parameter<double>( "obstacle_avoidance.lane_boundary_join_slack", params.lane_boundary_join_slack );
  params.intersection_drivable_area_search_radius = node.declare_parameter<double>( "obstacle_avoidance.intersection_drivable_area_search_radius", params.intersection_drivable_area_search_radius );

  // Oncoming traffic gap-acceptance parameters
  params.oncoming_time_margin = node.declare_parameter<double>( "obstacle_avoidance.oncoming_time_margin", params.oncoming_time_margin );
  params.min_oncoming_route_speed = node.declare_parameter<double>( "obstacle_avoidance.min_oncoming_route_speed", params.min_oncoming_route_speed );
  params.oncoming_spatial_margin = node.declare_parameter<double>( "obstacle_avoidance.oncoming_spatial_margin", params.oncoming_spatial_margin );
  // Ego-lane oncoming stop behavior parameters
  params.ego_lane_oncoming_lateral_margin = node.declare_parameter<double>( "obstacle_avoidance.ego_lane_oncoming_lateral_margin", params.ego_lane_oncoming_lateral_margin );
  params.ego_lane_oncoming_stop_distance = node.declare_parameter<double>( "obstacle_avoidance.ego_lane_oncoming_stop_distance", params.ego_lane_oncoming_stop_distance );

  // Active modified-route safety monitor parameters
  params.modified_route_braking_safety_margin = node.declare_parameter<double>( "obstacle_avoidance.modified_route_braking_safety_margin", params.modified_route_braking_safety_margin );

  // Trajectory and geometry parameters
  params.route_window_min = node.declare_parameter<double>( "obstacle_avoidance.route_window_min", params.route_window_min );
  params.projection_progress_tolerance = node.declare_parameter<double>( "obstacle_avoidance.projection_progress_tolerance", params.projection_progress_tolerance );
  params.stopped_ego_speed = node.declare_parameter<double>( "obstacle_avoidance.stopped_ego_speed", params.stopped_ego_speed );
  params.planned_braking_deceleration = node.declare_parameter<double>( "obstacle_avoidance.planned_braking_deceleration", params.planned_braking_deceleration );

  const std::string validation_error =
    planner::validate_obstacle_avoidance_params( params );
  if( !validation_error.empty() )
  {
    throw std::invalid_argument(
      "invalid obstacle avoidance parameters: " + validation_error );
  }

  return params;
}

DecisionMaker::DecisionMaker( const rclcpp::NodeOptions& opts ) :
  rclcpp::Node{ "decision_maker", opts }

{
  load_parameters();
  setup_subscribers();
  setup_publishers();
  obstacle_avoidance_params = load_obstacle_avoidance_params( *this );
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

  const std::string vehicle_validation_error =
    ::adore::planner::validate_obstacle_avoidance_vehicle_params(
      vehicle_model->params );
  if( !vehicle_validation_error.empty() )
  {
    throw std::invalid_argument(
      "invalid physical vehicle parameters for obstacle avoidance: " +
      vehicle_validation_error );
  }

  planner.set_vehicle_parameters( vehicle_model->params );
  planner.set_comfort_settings( comfort_settings );
  planner.set_parameters( planner_settings );
}

void DecisionMaker::setup_subscribers()
{
  timer = create_wall_timer(
    std::chrono::milliseconds(
      static_cast<int>(
        1000.0 * ::adore::planner::obstacle_avoidance_cycle_time_s ) ),
                             std::bind( &DecisionMaker::timer_callback, this ) );

  subscriber_vehicle_state_dynamic = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>( "vehicle_state_dynamic", 1,
                                      [this](const adore_ros2_msgs::msg::VehicleStateDynamic& msg) {  latest_vehicle_state_dynamic = dynamics::conversions::to_cpp_type(msg); });

  subscriber_route = create_subscription<adore_ros2_msgs::msg::Route>( "route", 1,
                                      [this](const adore_ros2_msgs::msg::Route& msg) {
                                        auto new_route = map::conversions::to_cpp_type(msg);

                                        // A running OA maneuver retains its stored geometric
                                        // baseline and modified route until release_s. Keep the
                                        // newest mission route here so it becomes active only
                                        // after that lifecycle transition succeeds.
                                        latest_route = new_route;
                                      });

  subscriber_odd = create_subscription<open_odd_ros2_msgs::msg::OddEvaluation>( "odd_evaluation", 1,
                                      [this](const open_odd_ros2_msgs::msg::OddEvaluation& msg) {  latest_odd = msg; });

  subscriber_traffic_participants = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>( "traffic_participants", 1,
                                      [this](const adore_ros2_msgs::msg::TrafficParticipantSet& msg) 
                                      {  
                                        auto participants = dynamics::conversions::to_cpp_type(msg);
                                        for( const auto& [id, participant] : participants.participants )
                                        {
                                          (void)id;
                                          // Perception owns the sensor range. Keep every
                                          // participant it publishes instead of imposing a
                                          // second planner-side Euclidean distance limit.
                                          traffic_participants.update_traffic_participants( participant );
                                        }

                                        double max_participant_age = 1.0;
                                        traffic_participants.remove_old_participants( max_participant_age, now().seconds() );
                                      });

  subscriber_v2x_traffic_participants = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>( "/infrastructure/observed_traffic", 1,
                                      [this](const adore_ros2_msgs::msg::TrafficParticipantSet& msg) 
                                      {  
                                        auto participants = dynamics::conversions::to_cpp_type(msg);

                                        if ( participants.validity_area.has_value() )
                                        {
                                          latest_managed_zone = participants.validity_area;
                                        }
                                        
                                        for( const auto& [id, participant] : participants.participants )
                                        {
                                          if ( !participant.v2x_id.has_value() || v2x_id != participant.v2x_id.value())
                                          {
                                            if ( !participant.is_same_as( make_default_participant() ) )
                                            {
                                              traffic_participants.update_traffic_participants( participant );
                                            }
                                            continue;
                                          }

                                          if ( participant.trajectory.has_value() )
                                          {
                                            latest_managed_trajectory = participant.trajectory; 
                                          }
                                        }

                                        double max_participant_age = 1.0;
                                        traffic_participants.remove_old_participants( max_participant_age, now().seconds() );
                                      });

  subscriber_reference_trajectory = create_subscription<adore_ros2_msgs::msg::Trajectory>( "reference_trajectory", 1,
                                      [this](const adore_ros2_msgs::msg::Trajectory& msg) { latest_reference_trajectory = dynamics::conversions::to_cpp_type(msg); });

  subscriber_traffic_signals = create_subscription<adore_ros2_msgs::msg::TrafficSignals>( "traffic_signals", 1,
                                      [this](const adore_ros2_msgs::msg::TrafficSignals& msg) { traffic_signals = msg; });

  subscriber_safety_corridor = create_subscription<adore_ros2_msgs::msg::SafetyCorridor>( "safety_corridor", 1,
                                      [this](const adore_ros2_msgs::msg::SafetyCorridor& msg) { latest_safety_corridor = msg; });

  subscriber_weather = create_subscription<adore_ros2_msgs::msg::Weather>( "weather", 1,
                                      [this](const adore_ros2_msgs::msg::Weather& msg) {  latest_weather = msg; });

  subscriber_suggested_remote_operator_trajectory = create_subscription<adore_ros2_msgs::msg::Trajectory>( "suggested_remote_operator_trajectory", 1,
                                      [this](const adore_ros2_msgs::msg::Trajectory& msg) { 

                                        if ( !latest_vehicle_state_dynamic.has_value() )
                                          return;

                                        suggested_remote_operator_trajectory = dynamics::conversions::to_cpp_type(msg); 
                                        suggested_remote_operator_trajectory.value().adjust_start_time( latest_vehicle_state_dynamic.value().time );
                                       });

  //passenger requests
                                      
  subscriber_passenger_request =
    create_subscription<adore_ros2_msgs::msg::PassengerRequest>(
        "passenger_request", 10,
        [this]( const adore_ros2_msgs::msg::PassengerRequest& msg ) {
            handle_passenger_request( msg );
        } );

  // Runtime operator gate for every maneuver that leaves the current ego lane.
  // Keep obstacle_avoidance.enabled untouched: when the gate is closed, OA still
  // detects blocking objects and plans a stop (or an in-lane shift, if feasible).
  // An already active lane-crossing maneuver retains its committed route until a
  // safe return; the new gate state applies to subsequent candidate generation
  // and any optional active-maneuver replanning.
  subscriber_user_input = create_subscription<std_msgs::msg::String>(
    "/user_input",
    1,
    [this]( const std_msgs::msg::String& msg )
    {
      bool lane_avoidance_enabled = false;
      if( msg.data == "overtake now" )
      {
        lane_avoidance_enabled = true;
      }
      else if( msg.data != "overtake no more" )
      {
        return;
      }

      obstacle_avoidance_params.adjacent_lane_enabled =
        lane_avoidance_enabled;
      obstacle_avoidance_params.intersection_drivable_area_enabled =
        lane_avoidance_enabled;
      obstacle_avoidance_params.opposite_lane_enabled =
        lane_avoidance_enabled;

      RCLCPP_INFO(
        get_logger(),
        "[OA][OPERATOR] lane avoidance %s: adjacent=%d intersection=%d "
        "opposite=%d, obstacle_avoidance.enabled=%d",
        lane_avoidance_enabled ? "released" : "blocked",
        static_cast<int>( obstacle_avoidance_params.adjacent_lane_enabled ),
        static_cast<int>(
          obstacle_avoidance_params.intersection_drivable_area_enabled ),
        static_cast<int>( obstacle_avoidance_params.opposite_lane_enabled ),
        static_cast<int>( obstacle_avoidance_params.enabled ) );
    } );

  subscriber_unstructured_drivable_area = create_subscription<adore_ros2_msgs::msg::CautionZone>( "unstructured_drivable_area", 1,
                                    [this](const adore_ros2_msgs::msg::CautionZone& msg) {  unstructured_drivable_area = math::conversions::to_cpp_type(msg.polygon); });

  subscriber_remote_operation_status = create_subscription<adore_ros2_msgs::msg::RemoteOperationStatus>( "remote_operation_status", 1,
                                    [this](const adore_ros2_msgs::msg::RemoteOperationStatus& msg) {  remote_operation_status = msg; });
}

void DecisionMaker::setup_publishers()
{
  publisher_trajectory_decision = create_publisher<adore_ros2_msgs::msg::Trajectory>( "trajectory_decision", 1 );
  publisher_alternative_trajectory_decision = create_publisher<adore_ros2_msgs::msg::Trajectory>( "alternative_trajectory_decision", 1 );
    publisher_modified_route = create_publisher<adore_ros2_msgs::msg::Route>( "modified_route", 1 );
  publisher_v2x_traffic_participant = create_publisher<adore_ros2_msgs::msg::TrafficParticipant>( "v2x_traffic_participant", 1 );
  publisher_planning_cycle_runtime_ms =
    create_publisher<std_msgs::msg::Float64>( "planning_cycle_runtime_ms", 10 );
}

void DecisionMaker::timer_callback()
{
  const auto planning_start = std::chrono::steady_clock::now();
  auto behavior = choose_and_plan_driving_behavior();
  const auto planning_end = std::chrono::steady_clock::now();

  std_msgs::msg::Float64 runtime_message;
  runtime_message.data =
    std::chrono::duration<double, std::milli>( planning_end - planning_start ).count();
  publisher_planning_cycle_runtime_ms->publish( runtime_message );

  publisher_trajectory_decision->publish(behavior.trajectory);
    if ( behavior.modified_route.has_value() )
    {
        publisher_modified_route->publish( behavior.modified_route.value() );
    }

  if ( behavior.alternative_trajectory.has_value() )
  {
    publisher_alternative_trajectory_decision->publish(behavior.alternative_trajectory.value());
  }

  // @TODO, add publisher and behavior for signals

  if ( v2x_id != 0 )
  {
    publisher_v2x_traffic_participant->publish( dynamics::conversions::to_ros_msg(make_default_participant()) );
  }
  // @TODO, add a cleanup step, that removes old caution zones and old suggested trajectories, old safety corridors
}

behavior::Behavior DecisionMaker::choose_and_plan_driving_behavior()
{
  double time_now = now().seconds();

  bool has_localization = conditions::has_localization(latest_vehicle_state_dynamic, time_now);
  bool has_mission = conditions::has_mission(latest_vehicle_state_dynamic, latest_route);
  bool needs_to_avoid_safety_corridor = conditions::needs_to_avoid_safety_corridor(latest_vehicle_state_dynamic, latest_safety_corridor);
  bool can_drive_managed = conditions::can_drive_managed(latest_vehicle_state_dynamic, time_now, latest_managed_zone, latest_managed_trajectory);
  bool odd_conditions_satisfied = conditions::odd_conditions_satisfied(latest_odd, time_now);
  bool must_drive_unstructured = conditions::must_drive_unstructured( latest_vehicle_state_dynamic, unstructured_drivable_area );
  bool remote_operation_is_available = conditions::remote_operations_is_available( remote_operation_status, time_now );
  bool passenger_wants_vehicle_to_stand_still = conditions::passenger_wants_vehicle_to_stop( passenger_emergency_stop, resume_ride_requested, time_now );

  if (
      has_localization &&
      passenger_wants_vehicle_to_stand_still 
  )
  {
      RCLCPP_INFO(get_logger(), "Behavior decision: has_localization=%d, has_mission=%d, needs_to_avoid_safety_corridor=%d, can_drive_managed=%d, odd_conditions_satisfied=%d , passenger_emergency_stop=%d",
              has_localization, has_mission, needs_to_avoid_safety_corridor, can_drive_managed, odd_conditions_satisfied, passenger_emergency_stop);

    return behavior::minimum_risk(
                                  planner, 
                                  latest_vehicle_state_dynamic.value(), 
                                  latest_route.value(), 
                                  traffic_participants,
                                  latest_odd
                              );
  }

  if (
    has_localization &&
    needs_to_avoid_safety_corridor
  )
  {
    active_avoidance_state.reset();
    return behavior::avoiding_safety_corridor(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                traffic_participants,
                                latest_safety_corridor.value()
    );
  }

  if ( 
      must_drive_unstructured &&
      has_localization 
    )
  {
    active_avoidance_state.reset();
    return behavior::driving_unstructured(
                                unstructured_planner,
                                latest_vehicle_state_dynamic.value(),
                                latest_route.value(),
                                traffic_participants,
                                unstructured_drivable_area
    );
  }

  if (
      has_localization &&
      has_mission &&
      !odd_conditions_satisfied &&
      remote_operation_is_available 
    )
  {
    active_avoidance_state.reset();
    return behavior::remote_operations(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                latest_route.value(),
                                traffic_participants,
                                suggested_remote_operator_trajectory
    );
  }

  if (
    has_localization &&
    has_mission &&
    odd_conditions_satisfied && 
    can_drive_managed

  )
  {
    active_avoidance_state.reset();
    return behavior::driving_mission_following_managed(
                  planner,
                  latest_vehicle_state_dynamic.value(),
                  latest_managed_trajectory.value(),
                  latest_managed_zone.value()
    );
  }

  if (
      has_localization &&
      has_mission &&
      odd_conditions_satisfied
  )
  {
    return behavior::driving_mission(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                latest_route.value(),
                                traffic_participants,
                                comfort_settings,
                                traffic_signals,
                                latest_weather,
                                obstacle_avoidance_params,
                                active_avoidance_state
                              );
  }

  if ( 
      has_localization &&
      odd_conditions_satisfied
  )
  {
    active_avoidance_state.reset();
    return behavior::waiting_for_mission(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                traffic_participants
                              );
  }

  if (
    has_localization &&
    has_mission
  )
  {
    active_avoidance_state.reset();
    return behavior::minimum_risk(
                                  planner, 
                                  latest_vehicle_state_dynamic.value(), 
                                  latest_route.value(), 
                                  traffic_participants,
                                  latest_odd
                              );
  }

  active_avoidance_state.reset();
  return behavior::emergency(planner, latest_vehicle_state_dynamic);
}

dynamics::TrafficParticipant DecisionMaker::make_default_participant()
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

  // return dynamics::conversions::to_ros_msg( participant );
  return participant;
}

void DecisionMaker::handle_passenger_request(
    const adore_ros2_msgs::msg::PassengerRequest& msg)
{
    using PR = adore_ros2_msgs::msg::PassengerRequest;

    const float increase_factor = (!msg.numerical_detail.empty())
                                  ? static_cast<float>(msg.numerical_detail[0])
                                  : 1.3f;

    const float decrease_factor = (!msg.numerical_detail.empty())
                                      ? static_cast<float>(msg.numerical_detail[0])
                                      : 0.7f;

    auto& cs = comfort_settings;
    bool changed = false;

    switch (msg.type)
    {

      case PR::EXECUTE_EMERGENCY_STOP:
            passenger_emergency_stop = true;
            resume_ride_requested = false;
            RCLCPP_WARN(get_logger(), "Passenger emergency stop requested!");
            changed = true;
            break;

        case PR::RESUME_RIDE:
            passenger_emergency_stop = false;
            resume_ride_requested = true;

            suggested_remote_operator_trajectory.reset();
            RCLCPP_INFO(
                get_logger(),
                "Resume ride requested via PassengerRequest. Emergency stop cleared." );

            break;

        case PR::INCREASE_VELOCITY:
            cs.max_speed *= increase_factor;
            changed = true;
            break;

        case PR::DECREASE_VELOCITY:
            cs.max_speed *= decrease_factor;
            changed = true;
            break;

        case PR::DRIVE_MORE_SPORTILY:
            cs.max_acceleration         *= increase_factor;
            cs.min_acceleration         *= increase_factor;
            cs.max_lateral_acceleration *= increase_factor;
            changed = true;
            break;

        case PR::DRIVE_MORE_COMFORTABLY:
            cs.max_acceleration         *= decrease_factor;
            cs.min_acceleration         *= decrease_factor;
            cs.max_lateral_acceleration *= decrease_factor;
            changed = true;
            break;

        case PR::KEEP_MORE_DISTANCE:
            cs.time_headway     *= increase_factor;
            cs.distance_headway *= increase_factor;
            changed = true;
            break;
            
        default:
            return;
    }

    if (changed)
    {
        cs.clamp(planner.get_physical_vehicle_parameters()); 
        RCLCPP_INFO(get_logger(),
            "Comfort settings updated: max_speed=%.1f time_headway=%.1f",
            cs.max_speed, cs.time_headway);
    }
}

} // namespace adore

/* Register as component --------------------------------------------- */
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::DecisionMaker )
