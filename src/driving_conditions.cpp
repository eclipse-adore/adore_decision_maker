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

#include "driving_conditions.hpp"

namespace adore
{
namespace conditions
{
bool state_ok( 
                const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, 
                const double& time_now )
{
    if ( !vehicle_state_dynamic.has_value() )
        return false;

    if ( time_now - vehicle_state_dynamic.value().time > 1 ) // If the message is more than one second old 
        return false;

    // @TODO, add covarianve of estimate

    return true;
}

bool route_ok( 
                const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, 
                const std::optional<map::Route>& route 
            )
{
    if( !route.has_value() || !vehicle_state_dynamic.has_value() )
        return false;

    double remaining = route->get_length() - route->get_s( *vehicle_state_dynamic );
    size_t min_route_length  = 20;  // [m]
    return remaining > min_route_length;
}

bool need_remote_operator_assitance( 
                                        const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, 
                                        const std::map<std::string, math::Polygon2d>& caution_zones )
{
    if ( !vehicle_state_dynamic.has_value() )
        return false;

    // check if in a caution zone
    return std::any_of( caution_zones.begin(), caution_zones.end(),
                        [&]( const auto& zone ) { return zone.second.point_inside( *vehicle_state_dynamic ); } );
}

// bool
// safety_corridor_present( const Domain& d, const ConditionParams& )
// {
//   return d.safety_corridor.has_value();
// }

// bool
// waypoints_available( const Domain& d, const ConditionParams& )
// {
//   return d.waypoints.has_value() && d.waypoints->waypoints.size() > 0;
// }

// bool
// reference_traj_valid( const Domain& d, const ConditionParams& p )
// {
//   if( !d.reference_trajectory )
//     return false;

//   if( d.reference_trajectory->states.size() < p.min_ref_traj_size )
//     return false;

//   double age = d.vehicle_state->time - d.reference_trajectory->states.front().time;
//   return age <= p.max_ref_traj_age;
// }


// bool
// need_assistance( const Domain& d, const ConditionParams& )
// {
//   // check if in a caution zone
//   return std::any_of( d.caution_zones.begin(), d.caution_zones.end(),
//                       [&]( const auto& zone ) { return zone.second.point_inside( *d.vehicle_state ); } );
// }

// bool
// sent_assistance_request( const Domain& d, const ConditionParams& )
// {
//   return d.sent_assistance_request;
// }

// bool
// suggested_trajectory_accepted( const Domain& d, const ConditionParams& )
// {
//   return d.suggested_trajectory_acceptance;
// }

} // namespace conditions
} // namespace adore