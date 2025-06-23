#pragma once
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace adore
{
template<class T, class RosMsg>
class Latest
{
public:

  using optional_type = std::optional<T>;
  using Converter     = std::function<T( const RosMsg& )>;

  template<class Callable>
  void
  setup( rclcpp::Node& node, std::string topic, Callable&& convert, std::size_t queue = 1 )
  {
    static_assert( std::is_invocable_r_v<T, Callable, const RosMsg&>, "Converter must be callable as T(const RosMsg&)" );
    convert_ = std::forward<Callable>( convert ); // store converter
    sub_     = node.create_subscription<RosMsg>( std::move( topic ), queue, [this]( const typename RosMsg::SharedPtr msg )
        {
      std::lock_guard lock( mtx_ );
      value_.emplace( convert_( *msg ) );
    } );
  }

  Latest() = default;

  /* optional-like API */
  bool
  has_value() const noexcept
  {
    std::lock_guard lock( mtx_ );
    return value_.has_value();
  }

  const T&
  value() const
  {
    std::lock_guard lock( mtx_ );
    return value_.value();
  }

  optional_type
  get_optional() const
  {
    std::lock_guard lock( mtx_ );
    return value_;
  }

  const T&
  operator*() const
  {
    return value();
  } // const ref

  T&
  operator*()
  { // non-const ref
    std::lock_guard lock( mtx_ );
    return value_.value();
  }

  const T*
  operator->() const
  {
    return &value();
  }

  T*
  operator->()
  {
    std::lock_guard lock( mtx_ );
    return &value_.value();
  }

  explicit
  operator bool() const noexcept
  {
    return has_value();
  }

private:

  Converter                               convert_;
  mutable std::mutex                      mtx_;
  optional_type                           value_;
  rclcpp::Subscription<RosMsg>::SharedPtr sub_;
};
} // namespace adore
