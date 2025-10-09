#pragma once
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "behaviours.hpp"
#include "conditions.hpp"
#include "domain.hpp"

namespace adore::rules
{

struct Rule
{
  std::string              name;         // human label (for logs)
  std::string              behaviour;    // behaviour name to run if rule matches
  std::vector<std::string> require;      // all must be true
  std::vector<std::string> forbid;       // all must be false
  int                      priority = 0; // higher wins when multiple rules match
};

using Rules = std::unordered_map<std::string, Rule>;

Rules load_rules_yaml( const std::string& yaml_path );

std::optional<std::string> choose_behaviour( const conditions::ConditionState& state, const Rules& rules );

} // namespace adore