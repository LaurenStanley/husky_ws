// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from husky_msgs:msg/HuskyStatus.idl
// generated code does not contain a copyright notice

#ifndef HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__FUNCTIONS_H_
#define HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "husky_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "husky_msgs/msg/detail/husky_status__struct.h"

/// Initialize msg/HuskyStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * husky_msgs__msg__HuskyStatus
 * )) before or use
 * husky_msgs__msg__HuskyStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_husky_msgs
bool
husky_msgs__msg__HuskyStatus__init(husky_msgs__msg__HuskyStatus * msg);

/// Finalize msg/HuskyStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_husky_msgs
void
husky_msgs__msg__HuskyStatus__fini(husky_msgs__msg__HuskyStatus * msg);

/// Create msg/HuskyStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * husky_msgs__msg__HuskyStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_husky_msgs
husky_msgs__msg__HuskyStatus *
husky_msgs__msg__HuskyStatus__create();

/// Destroy msg/HuskyStatus message.
/**
 * It calls
 * husky_msgs__msg__HuskyStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_husky_msgs
void
husky_msgs__msg__HuskyStatus__destroy(husky_msgs__msg__HuskyStatus * msg);


/// Initialize array of msg/HuskyStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * husky_msgs__msg__HuskyStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_husky_msgs
bool
husky_msgs__msg__HuskyStatus__Sequence__init(husky_msgs__msg__HuskyStatus__Sequence * array, size_t size);

/// Finalize array of msg/HuskyStatus messages.
/**
 * It calls
 * husky_msgs__msg__HuskyStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_husky_msgs
void
husky_msgs__msg__HuskyStatus__Sequence__fini(husky_msgs__msg__HuskyStatus__Sequence * array);

/// Create array of msg/HuskyStatus messages.
/**
 * It allocates the memory for the array and calls
 * husky_msgs__msg__HuskyStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_husky_msgs
husky_msgs__msg__HuskyStatus__Sequence *
husky_msgs__msg__HuskyStatus__Sequence__create(size_t size);

/// Destroy array of msg/HuskyStatus messages.
/**
 * It calls
 * husky_msgs__msg__HuskyStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_husky_msgs
void
husky_msgs__msg__HuskyStatus__Sequence__destroy(husky_msgs__msg__HuskyStatus__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // HUSKY_MSGS__MSG__DETAIL__HUSKY_STATUS__FUNCTIONS_H_
