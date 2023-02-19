/**************************************************\
* The MIT License (MIT)
* Copyright (c) 2019 Kevin Walchko
* see LICENSE for full details
\**************************************************/

#pragma once

#include <cstdint>

// error messages
constexpr uint8_t MSG_NO_ERROR = 0;
constexpr uint8_t MSG_ERROR = 1;

// message types
constexpr uint8_t MSG_RAW_IMU = 0x41;
constexpr uint8_t MSG_IMU = 0x42;
constexpr uint8_t MSG_TWIST = 0x43;
constexpr uint8_t MSG_WRENCH = 0x44;
constexpr uint8_t MSG_POSE = 0x45;
constexpr uint8_t MSG_ATMOSPHERIC = 0x46;

/*
struct initialization:
https://en.cppreference.com/w/c/language/struct_initialization

c
----------------------
long   8B x86
----------------------
struct timeval {time_t tv_sec, suseconds_t tv_usec}
gettimeofday(timeval&, timezone& or NULL);

struct timespec {time_t tv_sec, long tv_nsec}
clock_gettime(CLOCK_MONOTONIC, timespec&)

time_t not defined by C

- `CLOCK_REALTIME` reports the actual wall clock time.
- `CLOCK_MONOTONIC` is for measuring relative real time. It advances at the same
rate as the actual flow of time but it's not subject to discontinuities from
manual or automatic (NTP) adjustments to the system clock.
- `CLOCK_PROCESS_CPUTIME_ID` is for measuring the amount of CPU time consumed by
the process.
- `CLOCK_THREAD_CPUTIME_ID` is for measuring the amount of CPU time consumed by
the thread. It's supported by modern kernels and glibc since 2.6.12, but on
older linux kernels glibc emulates it badly by simply returning the amount of
CPU time consumed by the process since the moment the thread was created.

arduino
----------------------
int    4B (SAMD)
https://www.arduino.cc/reference/en/language/variables/data-types/int/ long   4B
https://www.arduino.cc/reference/en/language/variables/data-types/long/ float 4B
https://www.arduino.cc/reference/en/language/variables/data-types/float/ double
8B https://www.arduino.cc/reference/en/language/variables/data-types/double/
----------------------
unsigned long (uint32_t) = millis()
unsigned long (uint32_t) = micros()
----------------------
*/

struct __attribute__((packed)) header_t {
  // uint8_t type;   // 1
  // uint8_t error;  // 1 or frame? or remove?
  uint64_t timestamp; // 8
                      // uint32_t timestamp; // 8
};                    // 10 bytes

struct __attribute__((packed)) vec_t {
  float x, y, z; // 4*3 = 12
};

struct __attribute__((packed)) quaternion_t {
  float w, x, y, z; // 4*4 = 16
};

struct __attribute__((packed)) imu_raw_t : header_t {
  // header_t header;
  vec_t linear_acceration; // 12
  vec_t angular_velocity;  // 12
  vec_t magnetic_field;    // 12
  float temperature;       // 4
};                         // 36 + 4 + 10 = 50

struct __attribute__((packed)) imu_ros_t : header_t {
  vec_t linear_acceration;  // 12
  vec_t angular_velocity;   // 12
  vec_t magnetic_field;     // 12
  quaternion_t orientation; // 16
  float temperature;        // 4
};

struct __attribute__((packed)) twist_t : header_t {
  vec_t linear;  // 12
  vec_t angular; // 12
};

struct __attribute__((packed)) wrench_t : header_t {
  vec_t force;  // 12
  vec_t torque; // 12
};

struct __attribute__((packed)) pose_t : header_t {
  vec_t position;           // 12
  quaternion_t orientation; // 16
};

struct __attribute__((packed)) atmospheric_t : header_t {
  float pressure;    // 4
  float temperature; // 4
};

struct __attribute__((packed)) imu_full_t : header_t {
  vec_t accelleration;      // 12
  vec_t gyroscope;          // 12
  vec_t magnetometer;       // 12
  quaternion_t orientation; // 16
  float pressure;           // 4
  float temperature;        // 4
};                          // 36 + 16 + 8 + 10 = 70

// struct illuminance_t: header_t {}
// struct image_t: header_t {}
// struct joy_t: header_t {}
// struct range_t: header_t {}
// struct range_array_t: header_t {} // array of range sensors
// struct laser_scan_t: header_t {}
