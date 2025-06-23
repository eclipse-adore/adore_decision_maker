#pragma once
#include "behaviours.hpp"
#include "conditions.hpp"
#include "domain.hpp"
#include <rclcpp/rclcpp.hpp>

namespace adore
{

struct DecisionPublisher
{
  rclcpp::Publisher<adore_ros2_msgs::msg::Trajectory>::SharedPtr         trajectory_publisher;
  rclcpp::Publisher<adore_ros2_msgs::msg::Trajectory>::SharedPtr         trajectory_suggestion_publisher;
  rclcpp::Publisher<adore_ros2_msgs::msg::AssistanceRequest>::SharedPtr  assistance_publisher;
  rclcpp::Publisher<adore_ros2_msgs::msg::CautionZone>::SharedPtr        caution_zone_publisher;
  rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipant>::SharedPtr traffic_participant_publisher;

  void
  init( rclcpp::Node& node )
  {
    trajectory_publisher            = node.create_publisher<adore_ros2_msgs::msg::Trajectory>( "trajectory_decision", rclcpp::QoS( 1 ) );
    trajectory_suggestion_publisher = node.create_publisher<adore_ros2_msgs::msg::Trajectory>( "trajectory_suggestion", rclcpp::QoS( 1 ) );
    assistance_publisher   = node.create_publisher<adore_ros2_msgs::msg::AssistanceRequest>( "assistance_request", rclcpp::QoS( 1 ) );
    caution_zone_publisher = node.create_publisher<adore_ros2_msgs::msg::CautionZone>( "caution_zones", rclcpp::QoS( 1 ) );
    traffic_participant_publisher = node.create_publisher<adore_ros2_msgs::msg::TrafficParticipant>( "traffic_participant",
                                                                                                     rclcpp::QoS( 1 ) );
  }

  void
  publish( const Decision& o )
  {
    if( o.trajectory )
      trajectory_publisher->publish( dynamics::conversions::to_ros_msg( *o.trajectory ) );
    // if( o.request_assistance )
    //   assistance_publisher->publish( o.request_assistance );
    if( o.trajectory_suggestion )
      trajectory_suggestion_publisher->publish( dynamics::conversions::to_ros_msg( *o.trajectory_suggestion ) );
    // if( d.caution_zones.size() > 0 )
    // {
    //   for( const auto& [label, polygon] : d.caution_zones )
    //   {
    //     adore_ros2_msgs::msg::CautionZone caution_zone_msg;
    //     caution_zone_msg.label           = label;
    //     caution_zone_msg.polygon         = math::conversions::to_ros_msg( polygon );
    //     caution_zone_msg.header.frame_id = "world";
    //     publisher_caution_zones->publish( caution_zone_msg );
    //   }
    // }
    if( o.publisher_traffic_participant )
    {
      auto msg = dynamics::conversions::to_ros_msg( *o.publisher_traffic_participant );
      traffic_participant_publisher->publish( msg );
    }
  }
};

/* ------------------------------------------------------------------ */
/*  DecisionMaker Node                                                */
/* ------------------------------------------------------------------ */
class DecisionMaker : public rclcpp::Node

{
public:

  explicit DecisionMaker( const rclcpp::NodeOptions& opts );

private:

  Domain                      domain;
  conditions::ConditionParams params;
  conditions::CheckList       check_list;

  rclcpp::TimerBase::SharedPtr  timer;
  DecisionPublisher             publisher_hub;
  std::vector<StateRequirement> state_requirements;

  DecisionTools tools;

  void load_parameters();
  void run(); // main loop
  void init_state_requirements();

  double run_delta_time = 0.1; // seconds, how often to run the decision maker
  bool   debug          = true;
};

} // namespace adore
