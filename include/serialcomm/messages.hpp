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
constexpr uint8_t MSG_RAW_IMU     = 0x41;
constexpr uint8_t MSG_IMU         = 0x42;
constexpr uint8_t MSG_TWIST       = 0x43;
constexpr uint8_t MSG_WRENCH      = 0x44;
constexpr uint8_t MSG_POSE        = 0x45;
constexpr uint8_t MSG_ATMOSPHERIC = 0x46;


typedef struct {
    uint8_t type;   // 1
    uint8_t error;  // 1
    long timestamp; // 8
} header_t;

typedef struct {
    float x,y,z; // 4*3 = 12
} vec_t;

typedef struct {
    float w,x,y,z; // 4*4 = 16
} quaternion_t;

struct imu_raw_t: header_t {
    vec_t linear_acceration;  // 12
    vec_t angular_velocity;   // 12
    vec_t magnetic_field;     // 12
    float temperature;        // 4
};

struct imu_t: header_t {
    vec_t linear_acceration;  // 12
    vec_t angular_velocity;   // 12
    vec_t magnetic_field;     // 12
    quaternion_t orientation; // 4
    float temperature;        // 4
};

struct twist_t: header_t {
    vec_t linear;  // 12
    vec_t angular; // 12
};

struct wrench_t: header_t {
    vec_t force;  // 12
    vec_t torque; // 12
};

struct pose_t: header_t {
    vec_t position;           // 12
    quaternion_t orientation; // 16
};

struct atmospheric_t: header_t {
    float pressure;    // 4
    float temperature; // 4
};

// struct illuminance_t: header_t {}
// struct image_t: header_t {}
// struct joy_t: header_t {}
// struct range_t: header_t {}
// struct range_array_t: header_t {} // array of range sensors
// struct laser_scan_t: header_t {}
