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

#pragma once
#include <optional>

#include "dynamics/trajectory.hpp"
#include "adore_map/route.hpp"

#include "adore_ros2_msgs/msg/trajectory.hpp"
#include "adore_ros2_msgs/msg/vehicle_signals.hpp"

#include "dynamics/traffic_participant.hpp"
#include "planning/trajectory_planner.hpp"

#include "adore_dynamics_conversions.hpp"
#include "decision_types.hpp"

#include "driving_conditions.hpp"
#include "planning/trajectory_planner.hpp"
#include "planning/planning_helpers.hpp"

namespace adore
{
namespace behavior
{    
    enum DrivingBehavior
    {
        DrivingMission,
        WaitingForMission,
        RemoteOperations,
        MinimumRisk,
        Emergency,
    };

    struct TrajectoryAndSignals
    {
        adore_ros2_msgs::msg::Trajectory trajectory;
        adore_ros2_msgs::msg::VehicleSignals signals;
    };

    TrajectoryAndSignals driving_mission(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const map::Route& route,
                                const dynamics::TrafficParticipantSet& traffic_participants 
    );

    TrajectoryAndSignals waiting_for_mission(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const dynamics::TrafficParticipantSet& traffic_participants 
    );
    
    TrajectoryAndSignals remote_operations(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const map::Route& route,
                                const dynamics::TrafficParticipantSet& traffic_participants,
                                const bool& approved_to_drive_suggested_remote_operations,
                                const std::optional<dynamics::Trajectory>& suggested_remote_operator_trajectory
    );

    TrajectoryAndSignals minimum_risk(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const map::Route& route,
                                const dynamics::TrafficParticipantSet& traffic_participants 
    );

} // namespace behavior
} // namespace adore