#pragma once
#include <array>
#include <bitset>
#include <functional>
#include <stdexcept>

#include "conditions.hpp"
#include "domain.hpp"

namespace adore
{
enum DecisionState
{
  EMERGENCY_STOP,
  FOLLOW_REFERENCE,
  FOLLOW_ROUTE,
  STANDSTILL,
  REMOTE_OPERATION,
  EXIT_SAFETY_CORRIDOR,
  REQUESTING_ASSISTANCE,
  MINIMUM_RISK_MANEUVER
};

struct StateRequirement
{
  DecisionState state;
  size_t        required_conditions;
};

namespace requirements
{

inline DecisionState
get_decision_state( const std::vector<StateRequirement>& state_requirements, size_t conditions )
{
  for( const auto& req : state_requirements )
  {
    if( ( conditions & req.required_conditions ) == req.required_conditions )
    {
      return req.state;
    }
  }
  throw std::runtime_error( "No matching state found for the given conditions." );
}

// helper for printing debug output
inline std::string
decision_state_to_string( DecisionState state )
{
  switch( state )
  {
    case EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    case FOLLOW_REFERENCE:
      return "FOLLOW_REFERENCE";
    case FOLLOW_ROUTE:
      return "FOLLOW_ROUTE";
    case STANDSTILL:
      return "STANDSTILL";
    case REMOTE_OPERATION:
      return "REMOTE_OPERATION";
    case EXIT_SAFETY_CORRIDOR:
      return "EXIT_SAFETY_CORRIDOR";
    case REQUESTING_ASSISTANCE:
      return "REQUESTING_ASSISTANCE";
    case MINIMUM_RISK_MANEUVER:
      return "MINIMUM_RISK_MANEUVER";
    default:
      throw std::runtime_error( "Unknown DecisionState" );
  }
};

} // namespace requirements

} // namespace adore