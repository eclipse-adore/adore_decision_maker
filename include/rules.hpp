/********************************************************************************
 * Copyright (C) 2024-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 ********************************************************************************/
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