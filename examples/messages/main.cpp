#include <cstdio>
#include <cstdint>
#include <serialcomm/serial.hpp>
#include <string>
#include <unistd.h>
#include <iostream>
#include <serialcomm/messages.hpp>
#include <ctime>

using namespace std;

int main(){
    // uint32_t now = (uint32_t) time(NULL);
    uint64_t now = time(NULL);
    atmospheric_t a;
    a.pressure = 1;
    a.temperature = 2;
    // a.error = MSG_NO_ERROR;
    a.timestamp = now;
    cout << "a: " << sizeof(a) << endl;

    // atmospheric_t b{1,2,3,4,5};
    // cout << "b: " << b.pressure << " " << b.timestamp << endl;

    vec_t f = {1.,2.,3.};
    vec_t d;
    d.x = 12.;
    vec_t e{1.,2.,3.};

    // imu_raw_t c{MSG_RAW_IMU,MSG_NO_ERROR,now,{1,2,3},{4,5,6},{7,8,9},10};
    // // cout << "c: " << c.linear_acceration.x << " " << c.temperature << " " << c.timestamp << endl;
    // printf("Raw IMU (%lu):\n", sizeof(c));
    // printf(" header: %u %llu %d:\n", c.type, c.timestamp, c.error);
    // // printf(" a: %f %f %f\n", c.linear_acceration.x, c.linear_acceration.y, c.linear_acceration.z);
    // // printf(" g: %f %f %f\n", c.angular_velocity.x, c.angular_velocity.y, c.angular_velocity.z);
    // // printf(" m: %f %f %f\n", c.magnetic_field.x, c.magnetic_field.y, c.magnetic_field.z);
    // printf(" t: %f\n", c.temperature);

    // cout << "sizof(imu_raw_t): " << sizeof(c) << endl;
    cout << "int:" << sizeof(int) << " " << sizeof(unsigned int) << " " << sizeof(uint32_t) << endl;
    cout << "ulong: " << sizeof(long) << " " << sizeof(unsigned long) << " " << sizeof(int64_t) << endl;
    cout << "vec_t 12: " << sizeof(vec_t) << endl;
    cout << "header_t 10: " << sizeof(header_t) << endl;
    cout << "imu_raw_t 50: " << sizeof(imu_raw_t) << endl;
    cout << "imu_full_t 70: " << sizeof(imu_full_t) << endl;
    cout << "quaternion_t 16: " << sizeof(quaternion_t) << endl;

    return 0;
}
