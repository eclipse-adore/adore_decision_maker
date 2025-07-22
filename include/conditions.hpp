#pragma once
#include <array>
#include <cstdint>

#include "domain.hpp"

namespace adore::conditions
{
//---------------------------------------------------------------
// Threshold parameters
//---------------------------------------------------------------
struct ConditionParams
{
  double gps_sigma_ok      = 0.2;
  size_t min_ref_traj_size = 5;
  double max_ref_traj_age  = 0.5; // [s]
  double min_route_length  = 4.0; // [m]
};

//---------------------------------------------------------------
// Atomic flag bits
//---------------------------------------------------------------
enum Condition : std::uint32_t
{
  STATE                           = 1u << 1,
  SAFETY_CORRIDOR                 = 1u << 2,
  WAYPOINTS                       = 1u << 3,
  REFERENCE_TRAJECTORY            = 1u << 4,
  ROUTE                           = 1u << 5,
  IN_ASSISTANCE_ZONE              = 1u << 7,
  SENT_ASSISTANCE_REQUEST         = 1u << 8,
  SUGGESTED_TRAJECTORY_ACCEPTANCE = 1u << 9,
};

//---------------------------------------------------------------
// Per‑flag check functions
//---------------------------------------------------------------
bool state_ok( const Domain& d, const ConditionParams& );
bool safety_corridor_present( const Domain& d, const ConditionParams& );
bool waypoints_available( const Domain& d, const ConditionParams& );
bool reference_traj_valid( const Domain& d, const ConditionParams& p );
bool route_available( const Domain& d, const ConditionParams& p );
bool need_assistance( const Domain& d, const ConditionParams& );
bool sent_assistance_request( const Domain& d, const ConditionParams& );
bool suggested_trajectory_accepted( const Domain& d, const ConditionParams& );

//---------------------------------------------------------------
// Descriptor + master constexpr table
//---------------------------------------------------------------
struct ConditionDesc
{
  Condition flag;
  bool ( *eval )( const Domain&, const ConditionParams& );
};

constexpr std::array<ConditionDesc, 8> k_conditions = {
  { { STATE, &state_ok },
   { SAFETY_CORRIDOR, &safety_corridor_present },
   { WAYPOINTS, &waypoints_available },
   { REFERENCE_TRAJECTORY, &reference_traj_valid },
   { ROUTE, &route_available },
   { IN_ASSISTANCE_ZONE, &need_assistance },
   { SENT_ASSISTANCE_REQUEST, &sent_assistance_request },
   { SUGGESTED_TRAJECTORY_ACCEPTANCE, &suggested_trajectory_accepted } }
};

//---------------------------------------------------------------
// Type‑safe bit mask wrapper
//---------------------------------------------------------------
struct ConditionMask
{
  constexpr ConditionMask( Condition c ) :
    bits{ static_cast<std::uint32_t>( c ) }
  {}

  constexpr ConditionMask() = default;
  std::uint32_t bits{ 0u };

  constexpr ConditionMask&
  set( Condition c )
  {
    bits |= static_cast<std::uint32_t>( c );
    return *this;
  }

  [[nodiscard]] constexpr bool
  has( Condition c ) const
  {
    return ( bits & static_cast<std::uint32_t>( c ) ) != 0u;
  }

  [[nodiscard]] constexpr bool
  all( ConditionMask m ) const
  {
    return ( bits & m.bits ) == m.bits;
  }

  [[nodiscard]] constexpr bool
  none_of( ConditionMask m ) const
  {
    return ( bits & m.bits ) == 0u;
  }
};

constexpr ConditionMask
operator|( Condition a, Condition b )
{
  return ConditionMask{}.set( a ).set( b );
}

constexpr ConditionMask
operator|( ConditionMask a, Condition b )
{
  return a.set( b );
}

//---------------------------------------------------------------
//  Run‑time evaluation
//---------------------------------------------------------------
[[nodiscard]]
inline ConditionMask
evaluate_conditions( const Domain& d, const ConditionParams& p )
{
  ConditionMask mask;
  for( const auto& c : k_conditions )
    if( c.eval( d, p ) )
      mask.set( c.flag );
  return mask;
}

} // namespace adore::conditions
