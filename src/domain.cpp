#include "domain.hpp"

namespace adore
{

void
Domain::setup( rclcpp::Node& n )
{

  latest_vehicle_state.setup( n, "vehicle_state/dynamic", []( const adore_ros2_msgs::msg::VehicleStateDynamic& m )
  {
    return dynamics::conversions::to_cpp_type( m );
  } );

  latest_route.setup( n, "route", [=]( const adore_ros2_msgs::msg::Route& m )
  {
    auto route = map::conversions::to_cpp_type( m );
    if( latest_local_map.has_value() )
    {
      route.map = std::make_shared<map::Map>( *latest_local_map );
    }
    return route;
  } );

  latest_local_map.setup( n, "local_map", []( const adore_ros2_msgs::msg::Map& m )
  {
    return map::conversions::to_cpp_type( m );
  } );

  latest_traffic_participants.setup( n, "traffic_participants", []( const adore_ros2_msgs::msg::TrafficParticipantSet& m )
  {
    return dynamics::conversions::to_cpp_type( m );
  } );

  latest_traffic_signals.setup( n, "traffic_signals", []( const adore_ros2_msgs::msg::TrafficSignals& m )
  {
    std::vector<map::TrafficLight> stopping_points;
    // for( const auto& signal : m.signals )
    // {
    //   map::TrafficLight tl;
    //   tl.id = signal.id;
    //   for( const auto& cp : signal.control_points )
    //     tl.control_points.emplace_back( cp.x, cp.y );
    //   tl.state = static_cast<map::TrafficLight::TrafficLightState>( signal.state );
    //   stopping_points.push_back( tl );
    // }
    return stopping_points;
  } );

  latest_suggested_trajectory.setup( n, "trajectory_suggestion", []( const adore_ros2_msgs::msg::Trajectory& m )
  {
    return dynamics::conversions::to_cpp_type( m );
  } );

  latest_reference_trajectory.setup( n, "reference_trajectory", []( const adore_ros2_msgs::msg::Trajectory& m )
  {
    return dynamics::conversions::to_cpp_type( m );
  } );

  latest_safety_corridor.setup( n, "safety_corridor", []( const adore_ros2_msgs::msg::SafetyCorridor& m )
  {
    return m;
  } );

  latest_vehicle_info.setup( n, "vehicle_info", []( const adore_ros2_msgs::msg::VehicleInfo& m )
  {
    return m;
  } );

  latest_waypoints.setup( n, "remote_operation_waypoints", []( const adore_ros2_msgs::msg::Waypoints& msg )
  {
    std::deque<adore::math::Point2d> out;
    // out.reserve( msg.waypoints.size() );
    // for( const auto& wp : msg.waypoints )
    //   out.emplace_back( wp.x, wp.y );
    return out;
  } );

  latest_goal.setup( n, "mission/goal_position", []( const adore_ros2_msgs::msg::GoalPoint& g )
  {
    return adore::math::Point2d{ g.x_position, g.y_position };
  } );

  latest_suggested_trajectory_acceptance.setup( n, "suggested_trajectory_accepted", []( const std_msgs::msg::Bool& b )
  {
    return b.data;
  } );
}

}; // namespace adore
