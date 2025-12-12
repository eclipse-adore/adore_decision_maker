/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "decision_maker.hpp"

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace adore
{

DecisionMaker::DecisionMaker( const rclcpp::NodeOptions& opts ) :
  rclcpp::Node{ "decision_maker", opts }

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
  publisher.publish( *this, decision );
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
