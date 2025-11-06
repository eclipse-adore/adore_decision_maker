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
#include "adore_dynamics_adapters.hpp"

#include "behaviours.hpp"
#include "conditions.hpp"
#include "decision_publisher.hpp"
#include "decision_types.hpp"
#include "domain.hpp"
#include "rules.hpp"
#include <rclcpp/rclcpp.hpp>

namespace adore
{

class DecisionMaker : public rclcpp::Node
{
public:

  explicit DecisionMaker( const rclcpp::NodeOptions& opts );

private:

  conditions::ConditionMap condition_map = conditions::make_condition_map();
  behaviours::BehaviourMap behaviour_map = behaviours::make_behaviour_map();
  rules::Rules             rules;

  // subscribers and latest sensor snapshots are stored in domain
  Domain domain;
  // publishers
  DecisionPublisher publisher;


  DecisionParams               params;
  rclcpp::TimerBase::SharedPtr timer;

  void setup();
  void run(); // main loop
};

} // namespace adore
