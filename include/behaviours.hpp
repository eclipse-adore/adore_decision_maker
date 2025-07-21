#pragma once
#include <optional>

#include "adore_dynamics_conversions.hpp"

#include "conditions.hpp"
#include "domain.hpp"
#include "planning/trajectory_planner.hpp"
#include "requirements.hpp"

namespace adore
{

struct Decision
{
  std::optional<dynamics::Trajectory>         trajectory;
  std::optional<dynamics::TrafficParticipant> traffic_participant;

  std::optional<dynamics::Trajectory> trajectory_suggestion;
  bool                                request_assistance       = false;
  bool                                reset_assistance_request = true;
};

struct DecisionTools
{
  planner::TrajectoryPlanner     planner;
  dynamics::PhysicalVehicleModel vehicle_model;
};

namespace behaviours
{

Decision execute( DecisionState, const Domain&, DecisionTools& );
Decision emergency_stop( const Domain&, DecisionTools& );
Decision standstill( const Domain&, DecisionTools& );
Decision follow_reference( const Domain&, DecisionTools& );
Decision follow_assistance( const Domain&, DecisionTools& );
Decision waiting_for_assistance( const Domain&, DecisionTools& );
Decision follow_route( const Domain&, DecisionTools& );
Decision remote_operation( const Domain&, DecisionTools& );
Decision safety_corridor( const Domain&, DecisionTools& );
Decision request_assistance( const Domain&, DecisionTools& );
Decision minimum_risk( const Domain&, DecisionTools& );

} // namespace behaviours
} // namespace adore
