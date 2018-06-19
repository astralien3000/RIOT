#ifndef ROS2_LOG_H
#define ROS2_LOG_H

typedef enum {

#define MACRO(a) a,
#include "ros2_log_defs.h"

} ros2_log_event_t;

void ros2_log_add_event(ros2_log_event_t evt);

void ros2_log_print(void);

void ros2_log_reset(void);

#endif//ROS2_LOG_H
