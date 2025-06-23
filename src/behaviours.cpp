#include "behaviours.hpp"

#include "planning/planning_helpers.hpp" // your existing helpers

namespace adore::behaviours
{

Decision
execute( DecisionState s, const Domain& domain, DecisionTools& tools )
{
  switch( s )
  {
    case STANDSTILL:
      return standstill( domain, tools );
    case FOLLOW_REFERENCE:
      return follow_reference( domain, tools );
    case FOLLOW_ROUTE:
      return follow_route( domain, tools );
    case REMOTE_OPERATION:
      return remote_operation( domain, tools );
    case EXIT_SAFETY_CORRIDOR:
      return safety_corridor( domain, tools );
    case REQUESTING_ASSISTANCE:
      return request_assistance( domain, tools );
    case MINIMUM_RISK_MANEUVER:
      return minimum_risk( domain, tools );
    case EMERGENCY_STOP:
      return emergency_stop( domain, tools );
    default:
      return emergency_stop( domain, tools );
  }
}

Decision
emergency_stop( const Domain& domain, DecisionTools& )
{
  Decision             out;
  dynamics::Trajectory emergency_stop_trajectory;
  if( domain.latest_vehicle_state )
    emergency_stop_trajectory.states.push_back( domain.latest_vehicle_state.value() );
  emergency_stop_trajectory.label = "Emergency Stop";
  out.trajectory                  = std::move( emergency_stop_trajectory );
  return out;
}

Decision
standstill( const Domain& domain, DecisionTools& )
{
  Decision             out;
  dynamics::Trajectory standstill_trajectory;
  standstill_trajectory.label = "Standstill";
  if( domain.latest_vehicle_state )
    standstill_trajectory.states.push_back( domain.latest_vehicle_state.value() );
  return out;
}

Decision
follow_reference( const Domain& domain, DecisionTools& tools )
{
  Decision out;
  out.trajectory        = tools.planner.optimize_trajectory( *domain.latest_vehicle_state, *domain.latest_reference_trajectory );
  out.trajectory->label = "Follow Reference";

  return out;
}

Decision
follow_route( const Domain& domain, DecisionTools& tools )
{
  Decision out;
  auto     traj  = tools.planner.plan_route_trajectory( *domain.latest_route, *domain.latest_vehicle_state,
                                                        *domain.latest_traffic_participants );
  traj.label     = "Follow Route";
  out.trajectory = std::move( traj );
  return out;
}

Decision
remote_operation( const Domain& domain, DecisionTools& tools )
{
  Decision out;

  dynamics::Trajectory trajectory = planner::waypoints_to_trajectory( *domain.latest_vehicle_state, *domain.latest_waypoints,
                                                                      *domain.latest_traffic_participants, tools.vehicle_model );
  trajectory.label                = "Remote Operation";
  trajectory.adjust_start_time( domain.latest_vehicle_state->time );
  out.trajectory = std::move( trajectory );

  return out;
}

Decision
safety_corridor( const Domain& domain, DecisionTools& tools )
{
  Decision out; // calculate trajectory getting out of safety corridor
  auto right_forward_points = planner::filter_points_in_front( domain.latest_safety_corridor->right_border, *domain.latest_vehicle_state );
  auto safety_waypoints     = planner::shift_points_right( right_forward_points, tools.vehicle_model.params.body_width );
  double target_speed       = planner::is_point_to_right_of_line( *domain.latest_vehicle_state, right_forward_points ) ? 0 : 2.0;

  auto planned_trajectory = planner::waypoints_to_trajectory( *domain.latest_vehicle_state, safety_waypoints,
                                                              *domain.latest_traffic_participants, tools.vehicle_model, target_speed );

  planned_trajectory = tools.planner.optimize_trajectory( *domain.latest_vehicle_state, planned_trajectory );

  planned_trajectory.label = "Safety Corridor";
  out.trajectory           = std::move( planned_trajectory );

  return out;
}

Decision
request_assistance( const Domain& domain, DecisionTools& tools )
{
  Decision out           = minimum_risk( domain, tools );
  out.request_assistance = true;
  return out;
}

Decision
minimum_risk( const Domain& domain, DecisionTools& tools )
{
  Decision out;

  double state_s          = domain.latest_route->get_s( *domain.latest_vehicle_state );
  auto   cut_route        = domain.latest_route->get_shortened_route( state_s, 100.0 );
  auto planned_trajectory = planner::waypoints_to_trajectory( *domain.latest_vehicle_state, cut_route, *domain.latest_traffic_participants,
                                                              tools.vehicle_model, 0.0 /* target_speed */ );

  planned_trajectory = tools.planner.optimize_trajectory( *domain.latest_vehicle_state, planned_trajectory );
  if( planned_trajectory.states.size() < 2 )
  {
    out = standstill( domain, tools );
  }
  planned_trajectory.label = "Minimum Risk Maneuver";
  out.trajectory           = std::move( planned_trajectory );

  return out;
}

} // namespace adore::behaviours
