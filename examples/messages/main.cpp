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
    atmospheric_t a;
    a.pressure = 1;
    a.temperature = 2;
    a.error = MSG_NO_ERROR;
    a.timestamp = time(NULL);
    cout << "a: " << sizeof(a) << endl;

    atmospheric_t b{1,2,3,4,5};
    cout << "b: " << b.pressure << " " << b.timestamp << endl;

    imu_raw_t c{MSG_RAW_IMU,MSG_NO_ERROR,time(NULL),{1,2,3},{4,5,6},{7,8,9},10};
    // cout << "c: " << c.linear_acceration.x << " " << c.temperature << " " << c.timestamp << endl;
    printf("Raw IMU (%lu):\n", sizeof(c));
    printf(" header: %c %ld %d:\n", c.type, c.timestamp, c.error);
    printf(" a: %f %f %f\n", c.linear_acceration.x, c.linear_acceration.y, c.linear_acceration.z);
    printf(" g: %f %f %f\n", c.angular_velocity.x, c.angular_velocity.y, c.angular_velocity.z);
    printf(" m: %f %f %f\n", c.magnetic_field.x, c.magnetic_field.y, c.magnetic_field.z);
    printf(" t: %f\n", c.temperature);

    return 0;
}
