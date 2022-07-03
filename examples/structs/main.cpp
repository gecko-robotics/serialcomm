#include <cstdio>
#include <cstdint>
#include <serialcomm/serial.hpp>
#include <string>
#include <unistd.h>
#include <iostream>

using namespace std;

typedef struct {
    float x,y,z;
    int error;
} msg_t;

int main(){
    // char buff[7];
    string port{"/dev/cu.usbmodem14601"};
    string msg{"g"};

    Serial ser;
    try {
        ser.open(port, B1000000);
    }
    catch (const SerialError& e){
        cout << e.what() << endl;
        exit(1);
    }

    // send "get data" command
    ser.write(msg.c_str(), msg.size());
    sleep(1);

    // read data
    msg_t data;
    int num = ser.read((uint8_t*)&data, sizeof(data));
    // string ans((char*)&bb, num);
    // string ans = ser.read();
    printf("got: %f %f %f %d\n", data.x, data.y, data.z, data.error);

    // uint8_t b[64];
    // memcpy(b, ans.c_str(), ans.size());
    // for (int i=0; i<ans.size(); i++){
    //     cout << b[i] << ",";
    // }
    // cout << endl;

    return 0;
}
