#include "decision_maker.hpp"

#include "debug_helpers.hpp"

namespace adore
{

DecisionMaker::DecisionMaker( const rclcpp::NodeOptions& opts ) :
  rclcpp::Node{ "decision_maker", opts }
{
  domain.setup( *this );
  publisher.init( *this );
  load_parameters();
  timer = create_wall_timer( std::chrono::milliseconds( static_cast<int>( run_delta_time * 1000 ) ),
                             std::bind( &DecisionMaker::run, this ) );
}

void
DecisionMaker::run()
{
  auto        cond_mask        = conditions::evaluate_conditions( domain, params );
  const auto& behaviour        = adore::choose_behaviour( cond_mask );
  Decision    decision         = behaviour.fn( domain, tools );
  decision.traffic_participant = make_participant();
  publisher.publish( decision, domain );
}

dynamics::TrafficParticipant
DecisionMaker::make_participant()
{
  dynamics::TrafficParticipant participant;
  if( domain.vehicle_state )
    participant.state = domain.vehicle_state.value();
  if( domain.route )
  {
    participant.route      = domain.route.value();
    participant.goal_point = domain.route.value().destination;
  }
  participant.id                  = domain.v2x_id;
  participant.classification      = dynamics::CAR;
  participant.physical_parameters = tools.vehicle_model->params;
  return participant;
}

void
DecisionMaker::load_parameters()
{
  params.gps_sigma_ok      = declare_parameter( "gps_sigma_ok", 1.0 );
  params.min_ref_traj_size = declare_parameter( "min_ref_traj_size", 5 );
  params.max_ref_traj_age  = declare_parameter( "max_ref_traj_age", 0.5 );
  params.min_route_length  = declare_parameter( "min_route_length", 4.0 );
  domain.v2x_id            = declare_parameter( "v2x_id", 0 );

  std::vector<std::string> keys   = declare_parameter( "planner_settings_keys", std::vector<std::string>() );
  std::vector<double>      values = declare_parameter( "planner_settings_values", std::vector<double>() );

  std::map<std::string, double> planner_params;
  if( keys.size() == values.size() )
  {
    for( size_t i = 0; i < keys.size(); i++ )
      planner_params[keys[i]] = values[i];
  }
  tools.planner.set_parameters( planner_params );
  std::string vehicle_model_file = declare_parameter( "vehicle_model_file", "" );
  tools.vehicle_model            = std::make_shared<dynamics::PhysicalVehicleModel>( vehicle_model_file, false );
  tools.comfort_settings         = std::make_shared<dynamics::ComfortSettings>(); // default value comfort settings
  tools.planner.set_vehicle_parameters( tools.vehicle_model->params );
  tools.planner.set_comfort_settings( tools.comfort_settings );
}

// -----------------------------------------------------------------------------
// ----- PUBLISHING ------------------------------------------------------------
// -----------------------------------------------------------------------------
void
DecisionPublisher::init( rclcpp::Node& node )
{
  const std::string topic_trajectory_decision = node.declare_parameter<std::string>( "topic_trajectory_decision", "trajectory_decision" );

  const std::string topic_trajectory_suggestion = node.declare_parameter<std::string>( "topic_trajectory_suggestion",
                                                                                       "trajectory_suggestion" );

  const std::string topic_assistance_request = node.declare_parameter<std::string>( "topic_assistance_request", "assistance_request" );

  const std::string topic_traffic_participant = node.declare_parameter<std::string>( "topic_traffic_participant", "traffic_participant" );

  trajectory_publisher            = node.create_publisher<TrajectoryAdapter>( topic_trajectory_decision, 1 );
  trajectory_suggestion_publisher = node.create_publisher<TrajectoryAdapter>( topic_trajectory_suggestion, 1 );
  assistance_publisher            = node.create_publisher<adore_ros2_msgs::msg::AssistanceRequest>( topic_assistance_request, 1 );
  traffic_participant_publisher   = node.create_publisher<ParticipantAdapter>( topic_traffic_participant, 1 );
}

void
DecisionPublisher::publish( const Decision& decision, Domain& domain )
{
  if( decision.trajectory )
    trajectory_publisher->publish( *decision.trajectory );
  if( decision.request_assistance )
  {
    // TODO
  }
  if( decision.trajectory_suggestion )
    trajectory_suggestion_publisher->publish( *decision.trajectory_suggestion );
  if( decision.traffic_participant )
  {
    traffic_participant_publisher->publish( *decision.traffic_participant );
  }
  if( decision.reset_assistance_request )
  {
    domain.sent_assistance_request         = false;
    domain.suggested_trajectory_acceptance = false;
    domain.suggested_trajectory.reset();
    domain.waypoints.reset();
  }
}


} // namespace adore

/* Register as component --------------------------------------------- */
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::DecisionMaker )
