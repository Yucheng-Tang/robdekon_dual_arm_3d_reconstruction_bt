#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace nbv_param {

// Read param `name` into `out`; if missing, use `def`.
// Does NOT declare the parameter (safe with automatically_declare_parameters_from_overrides).
template <typename T>
inline void get_or_default(const rclcpp::Node::SharedPtr& node,
                           const std::string& name,
                           T& out, const T& def)
{
  if (!node->get_parameter(name, out)) {
    out = def;
  }
}

// Optional: declare only if absent (useful if you are NOT using automatically_declare...).
template <typename T>
inline void declare_if_absent(rclcpp::Node& node,
                              const std::string& name,
                              const T& def)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, def);
  }
}

} // namespace nbv_param