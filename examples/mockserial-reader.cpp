#include <cstdint>
#include <cstdio>
#include <ctype.h>
#include <iostream>
#include <serialcomm/serialcomm.hpp>
#include <string>
#include <unistd.h>

using namespace std;

int main(int argc, char *argv[]) {
  if (argc < 2) {
    printf("ERROR: ./reader [serial_port]\n  Please supply port\n\n");
    return 1;
  }

  cout << ">> Opening: " << argv[1] << endl;

  SerialPort ser;
  ser.open(argv[1], B9600);
  if (!ser) {
    printf("*** ERROR Opening Port %s ***", argv[1]);
    return 1;
  }

  while (true) {
    char c = ser.read();
    if (c == -1) return 0;

    char C = toupper(c);
    printf(">> %c -> %c [%d]\n", c, C, int(c));
    ser.write(C);
    sleep(1);
  }
  return 0;
}