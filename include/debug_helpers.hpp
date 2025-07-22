#pragma once
#include "conditions.hpp"
#include "decision_rules.hpp"
#include "domain.hpp"

namespace adore
{
//---------------------------------------------------------------
// Debug helpers
//---------------------------------------------------------------
inline std::string
condition_to_string( Condition c )
{
  switch( c )
  {
    case STATE:
      return "state_ok";
    case SAFETY_CORRIDOR:
      return "safety_corridor";
    case WAYPOINTS:
      return "waypoints_available";
    case REFERENCE_TRAJECTORY:
      return "reference_traj_valid";
    case ROUTE:
      return "route_ok";
    case IN_ASSISTANCE_ZONE:
      return "in_assistance_zone";
    case SENT_ASSISTANCE_REQUEST:
      return "sent_assistance_request";
    case SUGGESTED_TRAJECTORY_ACCEPTANCE:
      return "suggested_traj_accept";
    default:
      return "unknown_condition";
  }
}

inline std::string
to_string( ConditionMask m )
{
  std::ostringstream oss;
  bool               first = true;
  for( Condition c : { STATE, SAFETY_CORRIDOR, WAYPOINTS, REFERENCE_TRAJECTORY, ROUTE, IN_ASSISTANCE_ZONE, SENT_ASSISTANCE_REQUEST,
                       SUGGESTED_TRAJECTORY_ACCEPTANCE } )
  {
    if( m.has( c ) )
    {
      if( !first )
        oss << '|';
      oss << condition_to_string( c );
      first = false;
    }
  }
  if( first )
    oss << "none";
  return oss.str();
}

} // namespace adore