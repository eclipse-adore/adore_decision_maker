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
#include <array>
#include <cstdint>
#include "adore_dynamics_conversions.hpp"

namespace adore
{
    namespace conditions
    {
        bool state_ok( const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, const double& time_now);
        bool route_ok( const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, const std::optional<map::Route>& route );
        bool need_remote_operator_assitance( const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, const std::map<std::string, math::Polygon2d>& caution_zones );
    } // namespace conditions
} // namespace adore


