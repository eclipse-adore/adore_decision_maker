#include "decision_maker.hpp"

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace adore
{

DecisionMaker::DecisionMaker( const rclcpp::NodeOptions& opts ) :
  rclcpp::Node{ "decision_maker", opts },
  logger( *this )

{
  setup();
  domain.setup( *this, params.domain_params, params.in_topics );
  publisher.setup( *this, params.out_topics );
}

void
DecisionMaker::run()
{
  auto     condition_state = conditions::evaluate_conditions( domain, params.condition_params, condition_map );
  auto     behaviour       = rules::choose_behaviour( condition_state, rules );
  Decision decision        = behaviour_map[behaviour.value()]( domain, params.planning_params );
  publisher.publish( decision );
}

void
DecisionMaker::setup()
{
  params = load_params( *this );

  timer = create_wall_timer( std::chrono::milliseconds( static_cast<int>( params.run_delta_time * 1000 ) ),
                             std::bind( &DecisionMaker::run, this ) );

  const std::string           pkg        = ament_index_cpp::get_package_share_directory( "decision_maker" );
  const std::filesystem::path rules_path = std::filesystem::path( pkg ) / "config" / "rules.yaml";

  std::string rules_file = declare_parameter( "rules_file", rules_path.string() );
  rules                  = rules::load_rules_yaml( rules_file );
}

} // namespace adore

/* Register as component --------------------------------------------- */
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::DecisionMaker )
