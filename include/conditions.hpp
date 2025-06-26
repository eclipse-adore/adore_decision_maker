#pragma once
#include <array>
#include <bitset>
#include <functional>
#include <stdexcept>

#include "domain.hpp"

namespace adore::conditions
{

struct ConditionParams
{
  double      gps_sigma_ok      = 0.2;
  std::size_t min_ref_traj_size = 5;
  double      max_ref_traj_age  = 0.5; // s
  double      min_route_length  = 4.0; // m
};

enum Condition
{
  STATE                           = 1 << 1,
  SAFETY_CORRIDOR                 = 1 << 2,
  WAYPOINTS                       = 1 << 3,
  REFERENCE_TRAJECTORY            = 1 << 4,
  ROUTE                           = 1 << 5,
  IN_ASSISTANCE_ZONE              = 1 << 7,
  SENT_ASSISTANCE_REQUEST         = 1 << 8,
  SUGGESTED_TRAJECTORY_ACCEPTANCE = 1 << 9,
};

using CheckFn   = std::function<bool( const Domain&, const ConditionParams& )>;
using CheckList = std::map<Condition, CheckFn>;

size_t check_conditions( const CheckList& all_checks, const Domain& d, const ConditionParams& p );
void   set_check( CheckList& all_checks, Condition c, CheckFn fn );
bool   state_ok( const Domain& d, const ConditionParams& p );
bool   safety_corridor_present( const Domain& d, const ConditionParams& );
bool   waypoints_available( const Domain& d, const ConditionParams& );
bool   reference_traj_valid( const Domain& d, const ConditionParams& p );
bool   route_available( const Domain& d, const ConditionParams& p );
bool   local_map_available( const Domain& d, const ConditionParams& );
bool   need_assistance( const Domain& d, const ConditionParams& );
bool   sent_assistance_request( const Domain& d, const ConditionParams& );
bool   suggested_trajectory_accepted( const Domain& d, const ConditionParams& );

// helper to print the conditions
inline std::string
condition_to_string( size_t conditions )
{
  std::string result;
  if( conditions & STATE )
    result += "State OK, ";
  if( conditions & SAFETY_CORRIDOR )
    result += "Safety Corridor Present, ";
  if( conditions & WAYPOINTS )
    result += "Waypoints Present, ";
  if( conditions & REFERENCE_TRAJECTORY )
    result += "Reference Trajectory Valid, ";
  if( conditions & ROUTE )
    result += "Route OK, ";
  if( conditions & IN_ASSISTANCE_ZONE )
    result += "Need Assistance, ";
  return result;
}

} // namespace adore::conditions
