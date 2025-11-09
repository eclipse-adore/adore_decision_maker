#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviours.hpp"
#include "conditions.hpp"
#include "decision_types.hpp"
#include "domain.hpp"
#include "rules.hpp"

using namespace adore;

// Small helper: load the same rules file used by DecisionMaker::setup()
static rules::Rules
load_default_rules()
{
  const std::string           pkg        = ament_index_cpp::get_package_share_directory( "decision_maker" );
  const std::filesystem::path rules_path = std::filesystem::path( pkg ) / "config" / "rules.yaml";
  return rules::load_rules_yaml( rules_path.string() );
}

TEST( DecisionMakerRules, EmergencyStopFallback )
{
  Domain domain{}; // all default-constructed, i.e. empty

  auto condition_map = conditions::make_condition_map();
  auto behaviour_map = behaviours::make_behaviour_map();
  auto rules         = load_default_rules();

  ConditionParams condition_params{};

  auto cond_state = conditions::evaluate_conditions( domain, condition_params, condition_map );
  auto chosen     = rules::choose_behaviour( cond_state, rules );

  // cerr expected vs actual chosen behaviour
  std::cerr << "Chosen behaviour: " << ( chosen.has_value() ? chosen.value() : "none" ) << std::endl;

  ASSERT_TRUE( chosen.has_value() );
  EXPECT_EQ( chosen.value(), "emergency_stop" );
}
