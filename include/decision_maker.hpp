/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once
#include "adore_dynamics_adapters.hpp"
#include "adore_ros2_msgs/msg/driving_behavior.hpp"

#include "behaviours.hpp"
#include "conditions.hpp"
#include "decision_publisher.hpp"
#include "decision_types.hpp"
#include "domain.hpp"
#include "rules.hpp"
#include <rclcpp/rclcpp.hpp>

#include "driving_behavior.hpp"
#include "dynamics/comfort_settings.hpp"
#include "planning/trajectory_planner.hpp"

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

  Domain domain;

  rclcpp::Subscription<adore_ros2_msgs::msg::DrivingBehavior>::SharedPtr subscriber_driving_behavior;

  rclcpp::Subscription<adore_ros2_msgs::msg::VehicleStateDynamic>::SharedPtr subscriber_vehicle_state_dynamic;
  rclcpp::Subscription<adore_ros2_msgs::msg::Route>::SharedPtr subscriber_route;
  // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr       subscriber_automation_toggle;

  // publishers
  // DecisionPublisher publisher;

  rclcpp::Publisher<adore_ros2_msgs::msg::Trajectory>::SharedPtr publisher_trajectory_decision;
  

  // Planning
  planner::TrajectoryPlanner planner; 
  dynamics::PhysicalVehicleParameters physical_vehicle_parameters;
  std::shared_ptr<dynamics::ComfortSettings> comfort_settings;

  // Domain
  adore_ros2_msgs::msg::DrivingBehavior latest_driving_behavior; // Always start in WAITING_FOR_MISSION
  std::optional<dynamics::VehicleStateDynamic> latest_vehicle_state_dynamic;
  std::optional<map::Route> latest_route;
  dynamics::TrafficParticipantSet traffic_participants;

  DecisionParams               params;
  rclcpp::TimerBase::SharedPtr timer;


  void load_parameters();
  void setup_subscribers();
  void setup_publishers();
  void timer_callback(); // main loop

  behavior::TrajectoryAndSignals plan_based_on_driving_behavior();
};

} // namespace adore
