
#include "rules.hpp"

#include <fstream>
#include <limits>

#include <yaml-cpp/yaml.h>

namespace adore::rules
{

std::optional<std::string>
choose_behaviour( const conditions::ConditionState& state, const Rules& rules )
{
  const auto is_true = [&]( const std::string& n ) {
    if( auto it = state.find( n ); it != state.end() )
      return it->second;
    return false;
  };

  std::optional<Rule> best;

  for( const auto& [name, rule] : rules )
  {
    if( rule.behaviour.empty() )
      continue;

    const bool require_ok = std::all_of( rule.require.begin(), rule.require.end(), is_true );
    if( !require_ok )
      continue;

    const bool forbid_ok = std::none_of( rule.forbid.begin(), rule.forbid.end(), is_true );
    if( !forbid_ok )
      continue;

    const bool better = !best || ( rule.priority > best->priority );
    if( better )
    {
      best = rule;
    }
  }

  return best ? std::optional<std::string>( best->behaviour ) : std::nullopt;
}

Rules
load_rules_yaml( const std::string& yaml_path )
{
  YAML::Node       doc        = YAML::LoadFile( yaml_path );
  const YAML::Node rules_node = doc["rules"];
  if( !rules_node || !rules_node.IsSequence() )
  {
    throw std::runtime_error( "rules: expected a YAML sequence at key 'rules'" );
  }

  Rules out;

  for( const auto& n : rules_node )
  {
    Rule r;

    if( !n["name"] )
    {
      throw std::runtime_error( "rule: missing required field 'name'" );
    }
    r.name = n["name"].as<std::string>();

    if( n["behaviour"] )
      r.behaviour = n["behaviour"].as<std::string>();

    // If none present, r.behaviour stays empty; choose_behaviour() can ignore empty behaviours.
    r.priority = n["priority"] ? n["priority"].as<int>( 0 ) : 0;

    if( n["require"] )
    {
      const auto& req = n["require"];
      if( !req.IsSequence() )
        throw std::runtime_error( "rule '" + r.name + "': 'require' must be a sequence" );
      r.require.reserve( req.size() );
      for( const auto& x : req )
        r.require.push_back( x.as<std::string>() );
    }

    if( n["forbid"] )
    {
      const auto& forb = n["forbid"];
      if( !forb.IsSequence() )
        throw std::runtime_error( "rule '" + r.name + "': 'forbid' must be a sequence" );
      r.forbid.reserve( forb.size() );
      for( const auto& x : forb )
        r.forbid.push_back( x.as<std::string>() );
    }

    auto [it, inserted] = out.emplace( r.name, std::move( r ) );
    if( !inserted )
    {
      throw std::runtime_error( "duplicate rule name: '" + it->first + "'" );
    }
  }

  return out;
}

} // namespace adore