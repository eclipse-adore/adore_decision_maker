#include "domain.hpp"

#include "adore_dynamics_conversions.hpp"
#include "adore_map_conversions.hpp"
#include "adore_math_conversions.hpp"

namespace adore
{

void
Domain::setup( rclcpp::Node& n )
{
  using namespace adore_ros2_msgs;

  std::string state_topic                           = "vehicle_state/dynamic";
  std::string route_topic                           = "route";
  std::string safety_corridor_topic                 = "safety_corridor";
  std::string suggested_trajectory_topic            = "suggested_trajectory";
  std::string reference_trajectory_topic            = "reference_trajectory";
  std::string traffic_participants_topic            = "infrastructure_traffic_participants";
  std::string traffic_signals_topic                 = "traffic_signals";
  std::string waypoints_topic                       = "remote_operation_waypoints";
  std::string suggested_trajectory_acceptance_topic = "suggested_trajectory_accepted";
  std::string caution_zones_topic                   = "caution_zones";

  add_subscription<StateAdapter>( n, state_topic, [&]( const dynamics::VehicleStateDynamic& msg ) { vehicle_state = msg; } );

  add_subscription<RouteAdapter>( n, route_topic, [&]( const map::Route& msg ) { route = msg; } );

  add_subscription<msg::SafetyCorridor>( n, safety_corridor_topic,
                                         [&]( const adore_ros2_msgs::msg::SafetyCorridor& msg ) { safety_corridor = msg; } );

  add_subscription<TrajectoryAdapter>( n, suggested_trajectory_topic,
                                       [&]( const dynamics::Trajectory& msg ) { suggested_trajectory = msg; } );

  add_subscription<TrajectoryAdapter>( n, reference_trajectory_topic,
                                       [&]( const dynamics::Trajectory& msg ) { reference_trajectory = msg; } );

  add_subscription<std_msgs::msg::Bool>( n, suggested_trajectory_acceptance_topic,
                                         [&]( const std_msgs::msg::Bool& msg ) { suggested_trajectory_acceptance = msg.data; } );

  add_subscription<ParticipantSetAdapter>( n, traffic_participants_topic, [&]( const dynamics::TrafficParticipantSet& participants ) {
    for( const auto& [id, participant] : participants.participants )
    {
      if( participant.id != v2x_id )
        traffic_participants.update_traffic_participants( participant );
      if( participant.trajectory && participant.id == v2x_id )
        reference_trajectory = participant.trajectory;

      traffic_participants.remove_old_participants( max_participant_age, n.now().seconds() );
    }
  } );

  add_subscription<msg::CautionZone>( n, caution_zones_topic, [&]( const msg::CautionZone& msg ) {
    caution_zones[msg.label] = math::conversions::to_cpp_type( msg.polygon );
  } );

  add_subscription<msg::Waypoints>( n, waypoints_topic, [&]( const msg::Waypoints& msg ) { waypoints = msg; } );

  add_subscription<msg::TrafficSignal>( n, traffic_signals_topic,
                                        [&]( const msg::TrafficSignal& msg ) { traffic_signals[msg.signal_group_id] = msg; } );
}
} // namespace adore
