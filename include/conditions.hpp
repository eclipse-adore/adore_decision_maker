#pragma once
#include <array>
#include <cstdint>

#include "domain.hpp"

namespace adore::conditions
{

// define condition parameters
struct ConditionParams
{
  // reference trajectory
  size_t min_ref_traj_size = 5;
  double max_ref_traj_age  = 1.0; // [s]
  size_t min_route_length  = 20;  // [m]
  double gps_sigma_ok      = 1.0; // [m]s
};

using ConditionFnPtr = bool ( * )( const Domain&, const ConditionParams& );
using ConditionMap   = std::unordered_map<std::string, ConditionFnPtr>;
using ConditionState = std::unordered_map<std::string, bool>;

//---------------------------------------------------------------
// Per‑flag check functions
//---------------------------------------------------------------

bool state_ok( const Domain& domain, const ConditionParams& params );
bool safety_corridor_present( const Domain& domain, const ConditionParams& params );
bool waypoints_available( const Domain& domain, const ConditionParams& params );
bool reference_traj_valid( const Domain& domain, const ConditionParams& params );
bool route_available( const Domain& domain, const ConditionParams& params );
bool need_assistance( const Domain& domain, const ConditionParams& params );
bool sent_assistance_request( const Domain& domain, const ConditionParams& params );
bool suggested_trajectory_accepted( const Domain& domain, const ConditionParams& params );

inline ConditionMap
make_condition_map()
{
  return {
    {                      "state_ok",                      state_ok },
    {       "safety_corridor_present",       safety_corridor_present },
    {           "waypoints_available",           waypoints_available },
    {          "reference_traj_valid",          reference_traj_valid },
    {               "route_available",               route_available },
    {               "need_assistance",               need_assistance },
    {       "sent_assistance_request",       sent_assistance_request },
    { "suggested_trajectory_accepted", suggested_trajectory_accepted },
  };
}

inline ConditionState
evaluate_conditions( const Domain& domain, const ConditionParams& params, const ConditionMap& condition_map )
{
  ConditionState state;
  for( const auto& [name, fn] : condition_map )
    state[name] = fn( domain, params );
  return state;
}


} // namespace adore::conditions
