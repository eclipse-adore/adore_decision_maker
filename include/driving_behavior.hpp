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

namespace adore
{
namespace behavior
{
    struct TrajectoryAndSignals
    {
        adore_ros2_msgs::msg::Trajectory trajectory;
        adore_ros2_msgs::msg::VehicleSignals signals;
    };

    TrajectoryAndSignals driving_mission(
                                planner::TrajectoryPlanner& planner,
                                const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic,  
                                const std::optional<map::Route>& route,
                                const dynamics::TrafficParticipantSet& traffic_participants 
                            );

} // namespace behavior
} // namespace adore