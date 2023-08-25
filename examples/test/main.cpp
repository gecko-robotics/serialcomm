#include <cstdint>
#include <cstdio>
#include <iostream>
#include <serialcomm/serialcomm.hpp>
#include <string>
#include <unistd.h>

using namespace std;

typedef struct {
  char a, b, c, d, e, f, g;
} msg_t;

int main() {
  char buff[7];
  string port{"/dev/cu.usbmodem14601"};
  string msg{"hello"};

  SerialPort ser;
  try {
    ser.open(port, B2000000);
  } catch (const SerialError &e) {
    cout << e.what() << endl;
    exit(1);
  }

  ser.write(msg.c_str(), msg.size());
  sleep(1);

  msg_t bb;
  int num = ser.readBytes((uint8_t *)&bb, sizeof(buff));
  string ans((char *)&bb, num);
  // string ans = ser.read();
  printf("got: %s\n", ans.c_str());

  uint8_t b[64];
  memcpy(b, ans.c_str(), ans.size());
  for (int i = 0; i < ans.size(); i++) {
    cout << b[i] << ",";
  }
  cout << endl;

  return 0;
}
