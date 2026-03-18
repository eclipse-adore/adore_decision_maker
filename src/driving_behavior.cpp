#include "driving_behavior.hpp"

namespace adore
{
namespace behavior
{
    TrajectoryAndSignals driving_mission(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const map::Route& route,
                                const dynamics::TrafficParticipantSet& traffic_participants 
                            )
    {
        // Go through route, and update speed at points based on traffic signal positions
        auto route_with_signal = route;
        for( auto& p : route_with_signal.reference_line )
        {
            // if( std::any_of( domain.traffic_signals.begin(), domain.traffic_signals.end(), [&]( const auto& s ) {
            //     return adore::math::distance_2d( s.second, p.second ) < 3.0 && s.second.state != adore_ros2_msgs::msg::TrafficSignal::GREEN;
            //     } ) )
            // p.second.max_speed = 0;
        }

        dynamics::Trajectory trajectory = planner.plan_route_trajectory( route_with_signal, vehicle_state_dynamic, traffic_participants );
        // auto traj = planning_tools.planner.plan_route_trajectory( route_with_signal, vehicle_state_dynamic, traffic_participants );
        trajectory.adjust_start_time( vehicle_state_dynamic.time );
        trajectory.label              = "Follow Route";
        // out.traffic_participant = make_default_participant( domain, planning_tools );

        TrajectoryAndSignals trajectory_and_signal;
        trajectory_and_signal.trajectory = dynamics::conversions::to_ros_msg( trajectory );

        return trajectory_and_signal;
    }

    TrajectoryAndSignals waiting_for_mission(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const dynamics::TrafficParticipantSet& traffic_participants 
                            )
    {
        dynamics::Trajectory trajectory;
        trajectory.label = "Standstill";

        trajectory.states.push_back( vehicle_state_dynamic );

        TrajectoryAndSignals trajectory_and_signal;
        trajectory_and_signal.trajectory = dynamics::conversions::to_ros_msg( trajectory );

        return trajectory_and_signal;
    }

    TrajectoryAndSignals remote_operations(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const map::Route& route,
                                const dynamics::TrafficParticipantSet& traffic_participants,
                                const bool& approved_to_drive_suggested_remote_operations,
                                const std::optional<dynamics::Trajectory>& suggested_remote_operator_trajectory
    )
    {
        TrajectoryAndSignals trajectory_and_signals;

        if ( suggested_remote_operator_trajectory.has_value() && approved_to_drive_suggested_remote_operations )
        {
            dynamics::Trajectory trajectory = suggested_remote_operator_trajectory.value();
            trajectory.label              = "remote operations (driving remote operator instructions)";

            trajectory_and_signals.trajectory = dynamics::conversions::to_ros_msg(trajectory);
            return trajectory_and_signals;
        }

        trajectory_and_signals = minimum_risk(
            planner,
            vehicle_state_dynamic,
            route,
            traffic_participants
        );

        trajectory_and_signals.trajectory.label = "remote operations (waiting for remote operator instructions)";
        return trajectory_and_signals;
    }

    TrajectoryAndSignals minimum_risk(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const map::Route& route,
                                const dynamics::TrafficParticipantSet& traffic_participants 
    )
    {

        double state_s = route.get_s( vehicle_state_dynamic );
        auto   cut_route          = route.get_shortened_route( state_s, 100.0 );
        auto   planned_trajectory = planner::waypoints_to_trajectory( vehicle_state_dynamic, cut_route, traffic_participants,
                                                                        dynamics::PhysicalVehicleModel(planner.get_physical_vehicle_parameters()), 0.0 /* target_speed */ );

        // planned_trajectory = planning_tools.planner.optimize_trajectory( *domain.vehicle_state, planned_trajectory );
        if( planned_trajectory.states.size() < 2 )
        {
            planned_trajectory.states.push_back( vehicle_state_dynamic );
        }
        planned_trajectory.label = "Minimum Risk Maneuver";

        TrajectoryAndSignals trajectory_and_signals;
        trajectory_and_signals.trajectory = dynamics::conversions::to_ros_msg(planned_trajectory);

        return trajectory_and_signals;
    }
}
}

