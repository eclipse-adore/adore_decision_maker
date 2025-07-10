#include "decision_maker.hpp"

namespace adore
{

DecisionMaker::DecisionMaker( const rclcpp::NodeOptions& opts ) :
  rclcpp::Node{ "decision_maker", opts }
{
  domain.setup( *this );
  decision_publisher.init( *this );
  load_parameters();
  init_state_requirements();
  timer = create_wall_timer( std::chrono::milliseconds( static_cast<int>( run_delta_time * 1000 ) ),
                             std::bind( &DecisionMaker::run, this ) );
}

void
DecisionMaker::run()
{

  auto conditions      = conditions::check_conditions( check_list, domain, params );
  auto behaviour_state = requirements::get_decision_state( state_requirements, conditions );
  auto behaviour       = behaviours::execute( behaviour_state, domain, tools );

  // always publish participant TODO tidy up
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
  participant.physical_parameters = tools.vehicle_model.params;

  behaviour.traffic_participant = participant;

  decision_publisher.publish( behaviour, domain );
  print_debug_info( conditions, behaviour_state );
}

void
DecisionMaker::init_state_requirements()
{
  using namespace conditions;

  state_requirements = {
    {   EXIT_SAFETY_CORRIDOR,                                                  STATE | SAFETY_CORRIDOR },
    {   FOLLOWING_ASSISTANCE, STATE | IN_ASSISTANCE_ZONE | WAYPOINTS | SUGGESTED_TRAJECTORY_ACCEPTANCE },
    { WAITING_FOR_ASSISTANCE,                     STATE | IN_ASSISTANCE_ZONE | SENT_ASSISTANCE_REQUEST },
    {  REQUESTING_ASSISTANCE,                                               STATE | IN_ASSISTANCE_ZONE },
    {       FOLLOW_REFERENCE,                                             STATE | REFERENCE_TRAJECTORY },
    {           FOLLOW_ROUTE,                                                            STATE | ROUTE },
    {  MINIMUM_RISK_MANEUVER,                                                            STATE | ROUTE },
    {             STANDSTILL,                                                                    STATE },
    {         EMERGENCY_STOP,                                                                        0 },
  };

  set_check( check_list, STATE, state_ok );
  set_check( check_list, ROUTE, route_available );
  set_check( check_list, REFERENCE_TRAJECTORY, reference_traj_valid );
  set_check( check_list, SAFETY_CORRIDOR, safety_corridor_present );
  set_check( check_list, WAYPOINTS, waypoints_available );
  set_check( check_list, IN_ASSISTANCE_ZONE, need_assistance );
  set_check( check_list, SUGGESTED_TRAJECTORY_ACCEPTANCE, suggested_trajectory_accepted );
  set_check( check_list, SENT_ASSISTANCE_REQUEST, sent_assistance_request );
}

void
DecisionMaker::load_parameters()
{
  declare_parameter( "gps_sigma_ok", 1.0 );
  declare_parameter( "min_ref_traj_size", 5 );
  declare_parameter( "max_ref_traj_age", 0.5 );
  declare_parameter( "min_route_length", 4.0 );

  get_parameter( "gps_sigma_ok", params.gps_sigma_ok );
  get_parameter( "min_ref_traj_size", params.min_ref_traj_size );
  get_parameter( "max_ref_traj_age", params.max_ref_traj_age );
  get_parameter( "min_route_length", params.min_route_length );

  declare_parameter( "v2x_id", 0 );
  get_parameter( "v2x_id", domain.v2x_id );

  std::string vehicle_model_file;
  declare_parameter( "vehicle_model_file", "" );
  get_parameter( "vehicle_model_file", vehicle_model_file );
  tools.vehicle_model                = dynamics::PhysicalVehicleModel( vehicle_model_file, false );
  tools.speed_profile.vehicle_params = tools.vehicle_model.params;
  tools.planner.speed_profile        = tools.speed_profile;
}

void
DecisionMaker::print_debug_info( size_t conditions, DecisionState behaviour_state )
{
  if( debug )
  {
    std::cerr << "Conditions: " << conditions::condition_to_string( conditions ) << std::endl;
    std::cerr << "Decision State: " << requirements::decision_state_to_string( behaviour_state ) << std::endl;
  }
}

void
DecisionPublisher::init( rclcpp::Node& node )
{
  trajectory_publisher            = node.create_publisher<TrajectoryAdapter>( "trajectory_decision", 1 );
  trajectory_suggestion_publisher = node.create_publisher<TrajectoryAdapter>( "trajectory_suggestion", 1 );
  assistance_publisher            = node.create_publisher<adore_ros2_msgs::msg::AssistanceRequest>( "assistance_request", 1 );
  traffic_participant_publisher   = node.create_publisher<ParticipantAdapter>( "traffic_participant", 1 );
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
