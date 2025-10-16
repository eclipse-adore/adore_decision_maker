#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Logger
{
public:


  Logger( rclcpp::Node& node )

  {
    topic_name = compute_default_topic( node );
    ros_pub    = node.create_publisher<std_msgs::msg::String>( topic_name, 1 );
  }

  // ---------------- Core API ----------------
  template<typename T>
  void
  add_info( const std::string& key, const T& content )

  {
    nlohmann::json jval = nlohmann::json( content );
    {
      std::lock_guard<std::mutex> lk( mu );
      snapshot[key] = jval;
    }
  }

  template<typename T>
  std::optional<T>
  get_info( const std::string& key ) const
  {
    std::lock_guard<std::mutex> lk( mu );
    auto                        it = snapshot.find( key );
    if( it == snapshot.end() )
      return std::nullopt;
    try
    {
      return it.value().get<T>();
    }
    catch( ... )
    {
      return std::nullopt;
    }
  }

  // Returns the full snapshot as a stringified JSON object
  std::string
  get_as_string( int indent = -1 ) const
  {
    std::lock_guard<std::mutex> lk( mu );
    return indent >= 0 ? snapshot.dump( indent ) : snapshot.dump();
  }

  // Replaces the snapshot with a parsed JSON object (object expected)
  bool
  from_string( const std::string& s )
  {
    try
    {
      auto j = nlohmann::json::parse( s );
      if( !j.is_object() )
        return false;
      std::lock_guard<std::mutex> lk( mu );
      snapshot = std::move( j );
      return true;
    }
    catch( ... )
    {
      return false;
    }
  }

  // Optional: publish the *entire snapshot* once (compact JSON)
  void
  publish_snapshot_once( rclcpp::Node& node )
  {
    std::string node_fqn;
    int64_t     sec  = 0;
    uint32_t    nsec = 0;
    {
      std::lock_guard<std::mutex> lk( mu );
      if( !ros_pub )
        return;
      node_fqn       = node.get_fully_qualified_name();
      const auto now = node.get_clock()->now();
      sec            = static_cast<int64_t>( now.seconds() );
      nsec           = static_cast<uint32_t>( now.nanoseconds() % 1000000000LL );
    }
    nlohmann::json root = {
      {     "node",                             node_fqn },
      {    "stamp", { { "sec", sec }, { "nsec", nsec } } },
      { "snapshot",        parse_json( get_as_string() ) }
    };
    std_msgs::msg::String msg;
    msg.data = root.dump();
    if( ros_pub )
    {
      try
      {
        ros_pub->publish( msg );
      }
      catch( ... )
      {}
    }
    std::cerr << root << std::endl;
  }

  // ---------------- Static helpers ----------------
  // One event (key/value) → NDJSON line
  static std::string
  stringify_entry( const std::string& key, int64_t sec, uint32_t nsec, const nlohmann::json& value, const std::string& node_fqn )
  {
    nlohmann::json line = {
      {  "node",                             node_fqn },
      {   "key",                                  key },
      { "stamp", { { "sec", sec }, { "nsec", nsec } } },
      { "value",                                value }
    };
    return line.dump(); // one-line JSON
  }

  // Try to parse an NDJSON line back to fields
  static bool
  unstringify_entry( const std::string& s, std::string& node_fqn_out, std::string& key_out, int64_t& sec_out, uint32_t& nsec_out,
                     nlohmann::json& value_out )
  {
    try
    {
      auto j = nlohmann::json::parse( s );
      if( !j.contains( "key" ) || !j.contains( "stamp" ) )
        return false;
      node_fqn_out   = j.value( "node", "" );
      key_out        = j.value( "key", "" );
      const auto& st = j.at( "stamp" );
      sec_out        = st.value( "sec", int64_t{ 0 } );
      nsec_out       = st.value( "nsec", uint32_t{ 0 } );
      value_out      = j.contains( "value" ) ? j.at( "value" ) : nlohmann::json();
      return true;
    }
    catch( ... )
    {
      return false;
    }
  }

  // ROS message wrappers (string payload flexibility)
  static std_msgs::msg::String
  to_ros_message( const std::string& payload )
  {
    std_msgs::msg::String msg;
    msg.data = payload;
    return msg;
  }

  static std::string
  from_ros_message( const std_msgs::msg::String& msg )
  {
    return msg.data;
  }

  // Utility: parse JSON with safety
  static nlohmann::json
  parse_json( const std::string& s )
  {
    try
    {
      return nlohmann::json::parse( s );
    }
    catch( ... )
    {
      return nlohmann::json();
    }
  }

  std::string
  get_topic_name() const
  {
    std::lock_guard<std::mutex> lk( mu );
    return topic_name;
  }

private:

  // Compute "<ns>/<node_name>/logging" with clean slashes
  std::string
  compute_default_topic( const rclcpp::Node& node ) const
  {
    std::string ns   = node.get_effective_namespace(); // e.g. "/" or "/robot"
    std::string name = node.get_name();                // e.g. "planner"
    if( ns.empty() )
      ns = "/";
    // ensure single leading slash and no double slashes when joining
    if( ns.back() == '/' )
      ns.pop_back();
    if( !ns.empty() && ns.front() != '/' )
      ns = "/" + ns;
    return ns + "/" + name + "/logging"; // e.g. "/robot/planner/logging"
  }

private:

  mutable std::mutex mu;
  nlohmann::json     snapshot = nlohmann::json::object();

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> ros_pub;
  std::string                                               topic_name;
}

;
