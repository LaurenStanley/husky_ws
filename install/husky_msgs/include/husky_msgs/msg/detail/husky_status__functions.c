// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husky_msgs:msg/HuskyStatus.idl
// generated code does not contain a copyright notice
#include "husky_msgs/msg/detail/husky_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
husky_msgs__msg__HuskyStatus__init(husky_msgs__msg__HuskyStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    husky_msgs__msg__HuskyStatus__fini(msg);
    return false;
  }
  // uptime
  // ros_control_loop_freq
  // mcu_and_user_port_current
  // left_driver_current
  // right_driver_current
  // battery_voltage
  // left_driver_voltage
  // right_driver_voltage
  // left_driver_temp
  // right_driver_temp
  // left_motor_temp
  // right_motor_temp
  // capacity_estimate
  // charge_estimate
  // timeout
  // lockout
  // e_stop
  // ros_pause
  // no_battery
  // current_limit
  return true;
}

void
husky_msgs__msg__HuskyStatus__fini(husky_msgs__msg__HuskyStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // uptime
  // ros_control_loop_freq
  // mcu_and_user_port_current
  // left_driver_current
  // right_driver_current
  // battery_voltage
  // left_driver_voltage
  // right_driver_voltage
  // left_driver_temp
  // right_driver_temp
  // left_motor_temp
  // right_motor_temp
  // capacity_estimate
  // charge_estimate
  // timeout
  // lockout
  // e_stop
  // ros_pause
  // no_battery
  // current_limit
}

husky_msgs__msg__HuskyStatus *
husky_msgs__msg__HuskyStatus__create()
{
  husky_msgs__msg__HuskyStatus * msg = (husky_msgs__msg__HuskyStatus *)malloc(sizeof(husky_msgs__msg__HuskyStatus));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husky_msgs__msg__HuskyStatus));
  bool success = husky_msgs__msg__HuskyStatus__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
husky_msgs__msg__HuskyStatus__destroy(husky_msgs__msg__HuskyStatus * msg)
{
  if (msg) {
    husky_msgs__msg__HuskyStatus__fini(msg);
  }
  free(msg);
}


bool
husky_msgs__msg__HuskyStatus__Sequence__init(husky_msgs__msg__HuskyStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  husky_msgs__msg__HuskyStatus * data = NULL;
  if (size) {
    data = (husky_msgs__msg__HuskyStatus *)calloc(size, sizeof(husky_msgs__msg__HuskyStatus));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husky_msgs__msg__HuskyStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husky_msgs__msg__HuskyStatus__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
husky_msgs__msg__HuskyStatus__Sequence__fini(husky_msgs__msg__HuskyStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      husky_msgs__msg__HuskyStatus__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

husky_msgs__msg__HuskyStatus__Sequence *
husky_msgs__msg__HuskyStatus__Sequence__create(size_t size)
{
  husky_msgs__msg__HuskyStatus__Sequence * array = (husky_msgs__msg__HuskyStatus__Sequence *)malloc(sizeof(husky_msgs__msg__HuskyStatus__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = husky_msgs__msg__HuskyStatus__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
husky_msgs__msg__HuskyStatus__Sequence__destroy(husky_msgs__msg__HuskyStatus__Sequence * array)
{
  if (array) {
    husky_msgs__msg__HuskyStatus__Sequence__fini(array);
  }
  free(array);
}
