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
 ********************************************************************************/
#pragma once
#include <optional>

#include "adore_dynamics_conversions.hpp"

#include "dynamics/comfort_settings.hpp"
#include "planning/trajectory_planner.hpp"

namespace adore
{

struct Decision
{
  std::optional<dynamics::Trajectory>         trajectory;
  std::optional<dynamics::TrafficParticipant> traffic_participant;
  std::optional<dynamics::Trajectory>         trajectory_suggestion;
  bool                                        request_assistance = false;
};

struct PlanningParams
{
  planner::TrajectoryPlanner                      planner;
  std::shared_ptr<dynamics::PhysicalVehicleModel> vehicle_model;
  std::shared_ptr<dynamics::ComfortSettings>      comfort_settings;
  std::map<std::string, double>                   planner_settings;
  int                                             v2x_id = 0;
};

// define condition parameters
struct ConditionParams
{
  // reference trajectory
  size_t min_ref_traj_size = 5;
  double max_ref_traj_age  = 1.0; // [s]
  size_t min_route_length  = 20;  // [m]
  double gps_sigma_ok      = 1.0; // [m]s
};

struct DomainParams
{
  double max_participant_age = 1.0;
  int    v2x_id              = 0;
};

struct InTopics
{
  std::string state                           = "vehicle_state_dynamic";
  std::string route                           = "route";
  std::string safety_corridor                 = "safety_corridor";
  std::string suggested_trajectory            = "suggested_trajectory";
  std::string reference_trajectory            = "reference_trajectory";
  std::string sensor_participants             = "traffic_participants";
  std::string infrastructure_participants     = "/planned_traffic";
  std::string traffic_signals                 = "traffic_signals";
  std::string waypoints                       = "remote_operation_waypoints";
  std::string suggested_trajectory_acceptance = "suggested_trajectory_accepted";
  std::string caution_zones                   = "caution_zones";
};

struct OutTopics
{
  std::string trajectory_decision   = "trajectory_decision";
  std::string trajectory_suggestion = "trajectory_suggestion";
  std::string assistance_request    = "assistance_request";
  std::string traffic_participant   = "traffic_participant";
};

struct DecisionParams
{
  double          run_delta_time = 0.1; // seconds, how often to run the decision maker
  bool            debug          = false;
  ConditionParams condition_params;
  PlanningParams  planning_params;
  DomainParams    domain_params;
  InTopics        in_topics;
  OutTopics       out_topics;
};

inline DecisionParams
load_params( rclcpp::Node& node )
{
  DecisionParams params;
  params.run_delta_time = node.declare_parameter( "run_delta_time", params.run_delta_time );
  params.debug          = node.declare_parameter( "debug", params.debug );
  // ---------------------------------------------------------------------------------------------------------
  // -------------------------------------------- Conditions --------------------------------------------------
  // ---------------------------------------------------------------------------------------------------------
  params.condition_params.gps_sigma_ok      = node.declare_parameter( "gps_sigma_ok", params.condition_params.gps_sigma_ok );
  params.condition_params.min_ref_traj_size = node.declare_parameter( "min_ref_traj_size", 20 );
  params.condition_params.max_ref_traj_age  = node.declare_parameter( "max_ref_traj_age", params.condition_params.max_ref_traj_age );
  params.condition_params.min_route_length  = node.declare_parameter( "min_route_length", 10 );

  // ---------------------------------------------------------------------------------------------------------
  // -------------------------------------------- Planning --------------------------------------------------
  // ---------------------------------------------------------------------------------------------------------
  std::vector<std::string> keys   = node.declare_parameter( "planner_settings_keys", std::vector<std::string>() );
  std::vector<double>      values = node.declare_parameter( "planner_settings_values", std::vector<double>() );
  if( keys.size() == values.size() )
  {
    for( size_t i = 0; i < keys.size(); i++ )
      params.planning_params.planner_settings[keys[i]] = values[i];
  }
  std::string vehicle_model_file          = node.declare_parameter( "vehicle_model_file", "" );
  params.planning_params.vehicle_model    = std::make_shared<dynamics::PhysicalVehicleModel>( vehicle_model_file, false );
  params.planning_params.comfort_settings = std::make_shared<dynamics::ComfortSettings>(); // default value comfort settings
  params.planning_params.planner.set_vehicle_parameters( params.planning_params.vehicle_model->params );
  params.planning_params.planner.set_comfort_settings( params.planning_params.comfort_settings );
  params.planning_params.planner.set_parameters( params.planning_params.planner_settings );
  params.planning_params.v2x_id = node.declare_parameter( "v2x_id", params.planning_params.v2x_id );

  // ---------------------------------------------------------------------------------------------------------
  // -------------------------------------------- Domain ----------------------------------------------------
  // ---------------------------------------------------------------------------------------------------------
  params.domain_params.max_participant_age = node.declare_parameter( "max_participant_age", params.domain_params.max_participant_age );
  params.domain_params.v2x_id              = params.planning_params.v2x_id;

  // ---------------------------------------------------------------------------------------------------------
  // -------------------------------------------- In Topics --------------------------------------------------
  // ---------------------------------------------------------------------------------------------------------
  params.in_topics.state                = node.declare_parameter( "topic_vehicle_state_dynamic", params.in_topics.state );
  params.in_topics.route                = node.declare_parameter( "topic_route", params.in_topics.route );
  params.in_topics.safety_corridor      = node.declare_parameter( "topic_safety_corridor", params.in_topics.safety_corridor );
  params.in_topics.suggested_trajectory = node.declare_parameter( "topic_suggested_trajectory", params.in_topics.suggested_trajectory );
  params.in_topics.reference_trajectory = node.declare_parameter( "topic_reference_trajectory", params.in_topics.reference_trajectory );
  params.in_topics.sensor_participants  = node.declare_parameter( "topic_traffic_participants", params.in_topics.sensor_participants );
  params.in_topics.infrastructure_participants     = node.declare_parameter( "topic_infrastructure_participants",
                                                                             params.in_topics.infrastructure_participants );
  params.in_topics.traffic_signals                 = node.declare_parameter( "topic_traffic_signals", params.in_topics.traffic_signals );
  params.in_topics.waypoints                       = node.declare_parameter( "topic_waypoints", params.in_topics.waypoints );
  params.in_topics.suggested_trajectory_acceptance = node.declare_parameter( "topic_suggested_trajectory_accepted",
                                                                             params.in_topics.suggested_trajectory_acceptance );
  params.in_topics.caution_zones                   = node.declare_parameter( "topic_caution_zones", params.in_topics.caution_zones );

  // ---------------------------------------------------------------------------------------------------------
  // -------------------------------------------- Out Topics --------------------------------------------------
  // ---------------------------------------------------------------------------------------------------------
  params.out_topics.trajectory_decision   = node.declare_parameter( "topic_trajectory_decision", params.out_topics.trajectory_decision );
  params.out_topics.trajectory_suggestion = node.declare_parameter( "topic_trajectory_suggestion",
                                                                    params.out_topics.trajectory_suggestion );
  params.out_topics.assistance_request    = node.declare_parameter( "topic_assistance_request", params.out_topics.assistance_request );
  params.out_topics.traffic_participant   = node.declare_parameter( "topic_traffic_participant", params.out_topics.traffic_participant );
  return params;
}
} // namespace adore