#include "decision_maker.hpp"

namespace adore
{

DecisionMaker::DecisionMaker( const rclcpp::NodeOptions& opts ) :
  rclcpp::Node{ "decision_maker", opts }
{
  domain.setup( *this );
  publisher_hub.init( *this );
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
  publisher_hub.publish( behaviour );

  if( debug )
  {
    std::cerr << "Conditions: " << conditions::condition_to_string( conditions ) << std::endl;
    std::cerr << "Decision State: " << requirements::decision_state_to_string( behaviour_state ) << std::endl;
  }
}

void
DecisionMaker::init_state_requirements()
{
  using namespace conditions;

  state_requirements = {
    {  EXIT_SAFETY_CORRIDOR,      STATE | SAFETY_CORRIDOR },
    {      REMOTE_OPERATION,            STATE | WAYPOINTS },
    { REQUESTING_ASSISTANCE,      STATE | NEED_ASSISTANCE },
    {      FOLLOW_REFERENCE, STATE | REFERENCE_TRAJECTORY },
    {          FOLLOW_ROUTE,    STATE | ROUTE | LOCAL_MAP },
    { MINIMUM_RISK_MANEUVER,    STATE | ROUTE | LOCAL_MAP },
    {            STANDSTILL,                        STATE },
    {        EMERGENCY_STOP,                            0 },
  };

  set_check( check_list, STATE, state_ok );
  set_check( check_list, SAFETY_CORRIDOR, safety_corridor_present );
  set_check( check_list, WAYPOINTS, waypoints_available );
  set_check( check_list, REFERENCE_TRAJECTORY, reference_traj_valid );
  set_check( check_list, ROUTE, route_available );
  set_check( check_list, LOCAL_MAP, local_map_available );
  set_check( check_list, NEED_ASSISTANCE, need_assistance );
}

void
DecisionMaker::load_parameters()
{
  declare_parameter( "gps_sigma_ok", 0.2 );
  declare_parameter( "min_ref_traj_size", 5 );
  declare_parameter( "max_ref_traj_age", 0.5 );
  declare_parameter( "min_route_length", 4.0 );

  get_parameter( "gps_sigma_ok", params.gps_sigma_ok );
  get_parameter( "min_ref_traj_size", params.min_ref_traj_size );
  get_parameter( "max_ref_traj_age", params.max_ref_traj_age );
  get_parameter( "min_route_length", params.min_route_length );

  std::string vehicle_model_file;
  declare_parameter( "vehicle_model_file", "" );
  get_parameter( "vehicle_model_file", vehicle_model_file );
  tools.vehicle_model                = dynamics::PhysicalVehicleModel( vehicle_model_file, false );
  tools.speed_profile.vehicle_params = tools.vehicle_model.params;
  tools.planner.speed_profile        = tools.speed_profile;
}


} // namespace adore

/* Register as component --------------------------------------------- */
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::DecisionMaker )
