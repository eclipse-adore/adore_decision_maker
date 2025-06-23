#include "conditions.hpp"

namespace adore::conditions
{

size_t
check_conditions( const CheckList& all_checks, const Domain& d, const ConditionParams& p )
{
  size_t condition = 0;

  for( const auto& [c, fn] : all_checks )
  {
    if( fn( d, p ) )
    {
      condition |= static_cast<size_t>( c );
    }
  }

  return condition;
}

void
set_check( CheckList& all_checks, Condition c, CheckFn fn )
{

  all_checks[c] = std::move( fn );
}

bool
state_ok( const Domain& d, const ConditionParams& p )
{
  return d.latest_vehicle_state && d.latest_vehicle_info && d.latest_vehicle_info->localization_error < p.gps_sigma_ok;
}

bool
safety_corridor_present( const Domain& d, const ConditionParams& )
{
  return static_cast<bool>( d.latest_safety_corridor );
}

bool
waypoints_available( const Domain& d, const ConditionParams& )
{
  return d.latest_waypoints && d.latest_waypoints->size() > 1;
}

bool
reference_traj_valid( const Domain& d, const ConditionParams& p )
{
  if( !d.latest_reference_trajectory || !d.latest_vehicle_state )
    return false;

  if( d.latest_reference_trajectory->states.size() < p.min_ref_traj_size )
    return false;

  double age = d.latest_vehicle_state->time - d.latest_reference_trajectory->states.front().time;
  return age <= p.max_ref_traj_age;
}

bool
route_available( const Domain& d, const ConditionParams& p )
{
  if( !d.latest_route || !d.latest_vehicle_state )
    return false;

  double remaining = d.latest_route->get_length() - d.latest_route->get_s( *d.latest_vehicle_state );
  return remaining > p.min_route_length;
}

bool
local_map_available( const Domain& d, const ConditionParams& )
{
  return static_cast<bool>( d.latest_local_map );
}

bool
need_assistance( const Domain& d, const ConditionParams& )
{
  return d.need_assistance;
}
} // namespace adore::conditions