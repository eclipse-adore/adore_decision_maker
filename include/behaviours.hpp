#pragma once
#include <optional>

#include "adore_dynamics_conversions.hpp"

#include "conditions.hpp"
#include "domain.hpp"
#include "dynamics/comfort_settings.hpp"
#include "planning/trajectory_planner.hpp"

namespace adore
{

struct Decision
{
  std::optional<dynamics::Trajectory>         trajectory;
  std::optional<dynamics::TrafficParticipant> traffic_participant;
  std::optional<dynamics::Trajectory>         trajectory_suggestion;
  bool                                        request_assistance       = false;
  bool                                        reset_assistance_request = true;
};

struct DecisionTools
{
  planner::TrajectoryPlanner                      planner;
  std::shared_ptr<dynamics::PhysicalVehicleModel> vehicle_model;
  std::shared_ptr<dynamics::ComfortSettings>      comfort_settings;
};

namespace behaviours
{

[[nodiscard]] Decision emergency_stop( const Domain&, DecisionTools& );
[[nodiscard]] Decision standstill( const Domain&, DecisionTools& );
[[nodiscard]] Decision follow_reference( const Domain&, DecisionTools& );
[[nodiscard]] Decision follow_assistance( const Domain&, DecisionTools& );
[[nodiscard]] Decision waiting_for_assistance( const Domain&, DecisionTools& );
[[nodiscard]] Decision follow_route( const Domain&, DecisionTools& );
[[nodiscard]] Decision safety_corridor( const Domain&, DecisionTools& );
[[nodiscard]] Decision request_assistance( const Domain&, DecisionTools& );
[[nodiscard]] Decision minimum_risk( const Domain&, DecisionTools& );

// create map lookup
using BehaviourFnPtr = Decision ( * )( const Domain&, DecisionTools& );

using BehaviourMap = std::unordered_map<std::string, BehaviourFnPtr>;

inline BehaviourMap
make_behaviour_map()
{
  using namespace behaviours;
  return {
    {         "emergency_stop",         &emergency_stop },
    {             "standstill",             &standstill },
    {       "follow_reference",       &follow_reference },
    {      "follow_assistance",      &follow_assistance },
    { "waiting_for_assistance", &waiting_for_assistance },
    {           "follow_route",           &follow_route },
    {        "safety_corridor",        &safety_corridor },
    {     "request_assistance",     &request_assistance },
    {           "minimum_risk",           &minimum_risk },
  };
}

} // namespace behaviours
} // namespace adore
