#include "driving_behavior.hpp"

namespace adore
{
namespace behavior
{

    TrajectoryAndSignals driving_mission(
                                planner::TrajectoryPlanner& planner,
                                const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic,  
                                const std::optional<map::Route>& route,
                                const dynamics::TrafficParticipantSet& traffic_participants 
                            )
    {
        if ( // This scenario should in theory never happen, but is still here for safety
            !vehicle_state_dynamic.has_value() ||
            !route.has_value()
        )
        {
            return TrajectoryAndSignals {};
        }


        // Go through route, and update speed at points based on traffic signal positions
        auto route_with_signal = route.value();
        for( auto& p : route_with_signal.reference_line )
        {
            // if( std::any_of( domain.traffic_signals.begin(), domain.traffic_signals.end(), [&]( const auto& s ) {
            //     return adore::math::distance_2d( s.second, p.second ) < 3.0 && s.second.state != adore_ros2_msgs::msg::TrafficSignal::GREEN;
            //     } ) )
            // p.second.max_speed = 0;
        }

        dynamics::Trajectory trajectory = planner.plan_route_trajectory( route_with_signal, vehicle_state_dynamic.value(), traffic_participants );
        // auto traj = planning_tools.planner.plan_route_trajectory( route_with_signal, vehicle_state_dynamic, traffic_participants );
        trajectory.adjust_start_time( vehicle_state_dynamic.value().time );
        trajectory.label              = "Follow Route";
        // out.traffic_participant = make_default_participant( domain, planning_tools );

        TrajectoryAndSignals trajectory_and_signal;
        trajectory_and_signal.trajectory = dynamics::conversions::to_ros_msg( trajectory );

        return trajectory_and_signal;
    }
}
}

