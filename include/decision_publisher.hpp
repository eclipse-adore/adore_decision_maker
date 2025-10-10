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
#include "domain.hpp"
#include "rules.hpp"
#include <rclcpp/rclcpp.hpp>

namespace adore
{

struct DecisionPublisher
{
  rclcpp::Publisher<TrajectoryAdapter>::SharedPtr                       trajectory_publisher;
  rclcpp::Publisher<TrajectoryAdapter>::SharedPtr                       trajectory_suggestion_publisher;
  rclcpp::Publisher<adore_ros2_msgs::msg::AssistanceRequest>::SharedPtr assistance_publisher;
  rclcpp::Publisher<ParticipantAdapter>::SharedPtr                      traffic_participant_publisher;
  void                                                                  setup( rclcpp::Node& node, const OutTopics& topics );
  void                                                                  publish( const Decision& decision );
};
} // namespace adore