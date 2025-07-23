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

} // namespace behaviours
} // namespace adore
