#pragma once
#include <array>

#include "behaviours.hpp" // Decision + behaviour functions
#include "conditions.hpp" // Condition, ConditionMask, flags
#include "domain.hpp"     // Domain

namespace adore
{
//------------------------------------------------------------------
// Behaviour specification
//------------------------------------------------------------------
struct BehaviourSpec
{
  const char* name;                                  // log / debug label
  Decision ( *fn )( const Domain&, DecisionTools& ); // behaviour implementation
  conditions::ConditionMask required;
  conditions::ConditionMask veto{}; // optional negatives
};

using namespace conditions;
using namespace behaviours;

//------------------------------------------------------------------
// Compile-time rule table (priority = order)
//------------------------------------------------------------------
constexpr std::array<BehaviourSpec, 9> k_behaviours{
  { { "Exit Safety Corridor", safety_corridor, //
      STATE | SAFETY_CORRIDOR },
   { "Follow Assistance", follow_assistance, //
      STATE | IN_ASSISTANCE_ZONE | WAYPOINTS | SUGGESTED_TRAJECTORY_ACCEPTANCE },
   { "Wait For Assistance", waiting_for_assistance, //
      STATE | IN_ASSISTANCE_ZONE | SENT_ASSISTANCE_REQUEST },
   { "Request Assistance", request_assistance, //
      STATE | IN_ASSISTANCE_ZONE },
   { "Follow Reference", follow_reference, //
      STATE | REFERENCE_TRAJECTORY },
   { "Follow Route", follow_route, //
      STATE | ROUTE },
   { "Minimum Risk Maneuver", minimum_risk, //
      STATE | ROUTE },
   { "Standstill", standstill, //
      STATE },
   { "Emergency Stop", emergency_stop, //
      ConditionMask{} } }
};

//------------------------------------------------------------------
//  Selector – returns the first matching specification
//------------------------------------------------------------------
[[nodiscard]]
inline const BehaviourSpec&
choose_behaviour( conditions::ConditionMask cm )
{
  for( const auto& row : k_behaviours )
    if( cm.all( row.required ) && cm.none_of( row.veto ) )
      return row;

  return k_behaviours.back(); // never reached (catch-all), but keeps compiler happy
}

} // namespace adore
