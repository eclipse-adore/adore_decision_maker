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
    case FOLLOWING_ASSISTANCE:
      return follow_assistance( domain, tools );
    case EXIT_SAFETY_CORRIDOR:
      return safety_corridor( domain, tools );
    case REQUESTING_ASSISTANCE:
      return request_assistance( domain, tools );
    case WAITING_FOR_ASSISTANCE:
      return waiting_for_assistance( domain, tools );
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
  if( domain.vehicle_state )
    emergency_stop_trajectory.states.push_back( domain.vehicle_state.value() );
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
  if( domain.vehicle_state )
    standstill_trajectory.states.push_back( domain.vehicle_state.value() );
  return out;
}

Decision
follow_reference( const Domain& domain, DecisionTools& tools )
{
  Decision out;
  out.trajectory        = tools.planner.optimize_trajectory( *domain.vehicle_state, *domain.reference_trajectory );
  out.trajectory->label = "Follow Reference";

  return out;
}

Decision
follow_route( const Domain& domain, DecisionTools& tools )
{
  Decision out;

  auto route_with_signal = domain.route.value();
  for( auto& p : route_with_signal.center_lane )
  {
    if( std::any_of( domain.traffic_signals.begin(), domain.traffic_signals.end(), [&]( const auto& s ) {
      return adore::math::distance_2d( s.second, p.second ) < 3.0 && s.second.state != adore_ros2_msgs::msg::TrafficSignal::GREEN;
    } ) )
      p.second.max_speed = 0;
  }


  auto traj      = tools.planner.plan_route_trajectory( route_with_signal, *domain.vehicle_state, domain.traffic_participants );
  traj.label     = "Follow Route";
  out.trajectory = std::move( traj );
  return out;
}

Decision
waiting_for_assistance( const Domain& domain, DecisionTools& tools )
{
  // if we have no waypoints, do nothing
  Decision out                 = minimum_risk( domain, tools );
  out.trajectory->label        = "Waiting for Waypoints";
  out.reset_assistance_request = false;

  if( domain.waypoints.has_value() && domain.waypoints->waypoints.size() > 1 )
  {
    // if we have waypoints, create trajectory to send
    dynamics::Trajectory trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, domain.waypoints->waypoints,
                                                                        domain.traffic_participants, tools.vehicle_model );
    trajectory.label                = "Suggested Trajectory";
    trajectory.adjust_start_time( domain.vehicle_state->time );
    out.trajectory_suggestion = std::move( trajectory );
    out.trajectory->label     = "Waiting for Confirmation";
  }

  return out;
}

Decision
follow_assistance( const Domain& domain, DecisionTools& tools )
{

  Decision out;
  out.reset_assistance_request = false;

  dynamics::Trajectory trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, domain.waypoints->waypoints,
                                                                      domain.traffic_participants, tools.vehicle_model );
  trajectory.label                = "Follow Assistance";
  trajectory.adjust_start_time( domain.vehicle_state->time );
  out.trajectory = std::move( trajectory );

  return out;
}

Decision
safety_corridor( const Domain& domain, DecisionTools& tools )
{
  Decision out; // calculate trajectory getting out of safety corridor
  auto     right_forward_points = planner::filter_points_in_front( domain.safety_corridor->right_border, *domain.vehicle_state );
  auto     safety_waypoints     = planner::shift_points_right( right_forward_points, tools.vehicle_model.params.body_width );
  double   target_speed         = planner::is_point_to_right_of_line( *domain.vehicle_state, right_forward_points ) ? 0 : 2.0;

  auto planned_trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, safety_waypoints, domain.traffic_participants,
                                                              tools.vehicle_model, target_speed );

  planned_trajectory = tools.planner.optimize_trajectory( *domain.vehicle_state, planned_trajectory );

  planned_trajectory.label = "Safety Corridor";
  out.trajectory           = std::move( planned_trajectory );

  return out;
}

Decision
request_assistance( const Domain& domain, DecisionTools& tools )
{
  Decision out                 = minimum_risk( domain, tools );
  out.request_assistance       = true;
  out.reset_assistance_request = false;
  return out;
}

Decision
minimum_risk( const Domain& domain, DecisionTools& tools )
{
  Decision out;

  double state_s            = domain.route->get_s( *domain.vehicle_state );
  auto   cut_route          = domain.route->get_shortened_route( state_s, 100.0 );
  auto   planned_trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, cut_route, domain.traffic_participants,
                                                                tools.vehicle_model, 0.0 /* target_speed */ );

  planned_trajectory = tools.planner.optimize_trajectory( *domain.vehicle_state, planned_trajectory );
  if( planned_trajectory.states.size() < 2 )
  {
    out = standstill( domain, tools );
  }
  planned_trajectory.label = "Minimum Risk Maneuver";
  out.trajectory           = std::move( planned_trajectory );

  return out;
}

} // namespace adore::behaviours
