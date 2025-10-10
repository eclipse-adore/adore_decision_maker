
#include "decision_publisher.hpp"

namespace adore
{

void
DecisionPublisher::setup( rclcpp::Node& node, const OutTopics& topics )
{
  trajectory_publisher            = node.create_publisher<TrajectoryAdapter>( topics.trajectory_decision, 1 );
  trajectory_suggestion_publisher = node.create_publisher<TrajectoryAdapter>( topics.trajectory_suggestion, 1 );
  assistance_publisher            = node.create_publisher<adore_ros2_msgs::msg::AssistanceRequest>( topics.assistance_request, 1 );
  traffic_participant_publisher   = node.create_publisher<ParticipantAdapter>( topics.traffic_participant, 1 );
}

void
DecisionPublisher::publish( const Decision& decision )
{
  if( decision.trajectory )
    trajectory_publisher->publish( *decision.trajectory );

  if( decision.trajectory_suggestion )
    trajectory_suggestion_publisher->publish( *decision.trajectory_suggestion );

  if( decision.request_assistance )
  {
    // TODO
  }


  // decision.traffic_participant = participant;
  traffic_participant_publisher->publish( *decision.traffic_participant );
}


} // namespace adore