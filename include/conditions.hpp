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
  double      gps_sigma_ok         = 0.2;
  std::size_t min_ref_traj_size    = 5;
  double      max_ref_traj_age     = 0.5; // s
  double      min_route_length     = 4.0; // m
  bool        only_follow_ref_traj = false;
};

enum Condition
{
  STATE                = 1 << 1,
  SAFETY_CORRIDOR      = 1 << 2,
  WAYPOINTS            = 1 << 3,
  REFERENCE_TRAJECTORY = 1 << 4,
  ROUTE                = 1 << 5,
  LOCAL_MAP            = 1 << 6,
  NEED_ASSISTANCE      = 1 << 7
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

// helper to print the conditions
inline std::string
condition_to_string( size_t conditions )
{
  std::string result;
  if( conditions & STATE )
    result += "STATE ";
  if( conditions & SAFETY_CORRIDOR )
    result += "SAFETY_CORRIDOR ";
  if( conditions & WAYPOINTS )
    result += "WAYPOINTS ";
  if( conditions & REFERENCE_TRAJECTORY )
    result += "REFERENCE_TRAJECTORY ";
  if( conditions & ROUTE )
    result += "ROUTE ";
  if( conditions & LOCAL_MAP )
    result += "LOCAL_MAP ";
  if( conditions & NEED_ASSISTANCE )
    result += "NEED_ASSISTANCE ";
  return result;
}

/* ------------------------------------------------------------------ */
} // namespace adore::conditions
