#pragma once
#include "adore_dynamics_adapters.hpp"

#include "behaviours.hpp"
#include "conditions.hpp"
#include "domain.hpp"
#include <rclcpp/rclcpp.hpp>

namespace adore
{

struct DecisionPublisher
{
  rclcpp::Publisher<TrajectoryAdapter>::SharedPtr                       trajectory_publisher;
  rclcpp::Publisher<TrajectoryAdapter>::SharedPtr                       trajectory_suggestion_publisher;
  rclcpp::Publisher<adore_ros2_msgs::msg::AssistanceRequest>::SharedPtr assistance_publisher;
  rclcpp::Publisher<ParticipantAdapter>::SharedPtr                      traffic_participant_publisher;
  void                                                                  init( rclcpp::Node& node );
  void                                                                  publish( const Decision& decision, Domain& domain );
};

class DecisionMaker : public rclcpp::Node
{
public:

  explicit DecisionMaker( const rclcpp::NodeOptions& opts );

private:

  Domain                       domain;
  conditions::ConditionParams  params;
  rclcpp::TimerBase::SharedPtr timer;
  DecisionPublisher            publisher;
  DecisionTools                tools;

  void                         load_parameters();
  void                         run();              // main loop
  dynamics::TrafficParticipant make_participant(); // helper to make participant

  double run_delta_time = 0.1; // seconds, how often to run the decision maker
  bool   debug          = false;
};

} // namespace adore
