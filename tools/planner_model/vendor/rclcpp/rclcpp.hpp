#pragma once
// Standalone build: remove logging only; NavFn arithmetic remains unmodified.
#define RCLCPP_DEBUG(...) ((void)0)
#define RCLCPP_WARN(...) ((void)0)
