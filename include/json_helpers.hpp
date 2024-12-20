/********************************************************************************
 * Copyright (C) 2024-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 *    Mikkel Skov Maarssø
 *    Sanath Himasekhar Konthala
 ********************************************************************************/

#pragma once
#include <cmath>

#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

#include "adore_dynamics_conversions.hpp"
#include "adore_map/lat_long_conversions.hpp"
#include "adore_math/point.h"

#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>

using json = nlohmann::json;

namespace adore
{

static std_msgs::msg::String
serialize_trajectory_to_json( const dynamics::Trajectory& trajectory_suggestion )
{
  // Helper lambda for UTM to lat/long conversion
  auto convert_to_latlong = []( double x, double y ) {
    auto lat_long = adore::map::convert_utm_to_lat_lon( x, y, 32, "U" );
    return std::vector<int>{ int( lat_long[0] * 1e7 ), int( lat_long[1] * 1e7 ), 0 };
  };

  // Helper function for heading conversion
  auto convert_heading = []( double psi ) { return static_cast<int>( ( psi * 360 / ( 2 * M_PI ) - 90 ) * 100 ); };

  // Populate each attribute in a structured way
  json j;
  j["path_point"]["position"]               = {};
  j["path_point"]["heading"]                = {};
  j["path_point"]["speed"]                  = {};
  j["path_point"]["acceleration"]           = {};
  j["path_point"]["symmetric_area_offset"]  = {};
  j["path_point"]["asymmetric_area_offset"] = {};
  j["path_point"]["delta_time"]             = {};

  double dt = 0.1;
  if( trajectory_suggestion.states.size() > 1 )
    dt = trajectory_suggestion.states[0].time - trajectory_suggestion.states[1].time;

  for( const auto& point : trajectory_suggestion.states )
  {
    j["path_point"]["position"].push_back( convert_to_latlong( point.x, point.y ) );
    j["path_point"]["heading"].push_back( convert_heading( point.yaw_angle ) );
    j["path_point"]["speed"].push_back( static_cast<int>( point.vx ) );
    j["path_point"]["acceleration"].push_back( 0 );
    j["path_point"]["symmetric_area_offset"].push_back( 0 );
    j["path_point"]["asymmetric_area_offset"].push_back( 0 );
    j["path_point"]["delta_time"].push_back( static_cast<int>( point.time ) );
  }

  // Fill in other fixed values
  j["timestamp"]                  = 1727420908399; // TODO: Replace with actual timestamp
  j["related_waypoint_timestamp"] = 1727420908399; // TODO: Replace with actual related timestamp

  // Convert JSON object to a string and assign to the message data field
  std_msgs::msg::String msg;
  msg.data = j.dump();
  return msg;
}

// Function to deserialize JSON waypoints
static std::deque<adore::math::Point2d>
deserialize_waypoints_from_json( const std::string& json_string )
{
  std::deque<adore::math::Point2d> waypoints;
  // Parse JSON string
  json j;
  try
  {
    j = json::parse( json_string );
  }
  catch( const json::parse_error& e )
  {
    std::cerr << "JSON parsing error: " << e.what() << '\n';
    return waypoints;
  }

  // Validate required fields
  if( !j.contains( "waypoints" ) || !j["waypoints"].contains( "position" ) )
  {
    std::cerr << "The waypoints message is missing required fields.\n";
    return waypoints;
  }

  // Deserialize position data
  const auto& position_vector = j["waypoints"]["position"].get<std::vector<std::vector<int>>>();
  if( position_vector.empty() )
  {
    std::cerr << "The position field of the waypoints message is empty.\n";
    return waypoints;
  }

  // Helper lambda for UTM conversion
  auto convert_position_to_utm = []( const std::vector<int>& pos ) -> std::pair<double, double> {
    auto utm_coords = adore::map::convert_lat_lon_to_utm( pos[0] * 1e-7, pos[1] * 1e-7 );
    return { utm_coords[0], utm_coords[1] };
  };

  // Deserialize each position into a Point

  for( size_t i = 0; i < position_vector.size(); ++i )
  {
    const auto& pos = position_vector[i];
    if( pos.size() < 2 )
    {
      std::cerr << "Position entry has insufficient coordinates.\n";
      return waypoints;
    }

    auto [utm_x, utm_y] = convert_position_to_utm( pos );
    waypoints.push_back( adore::math::Point2d{ utm_x, utm_y } );
  }

  return waypoints;
}

} // namespace adore
