#include <gtest/gtest.h>
#include <iostream>
#include <serialcomm/serialcomm.hpp>
#include <string>

using namespace std;

struct msg_t {
  float x, y, z;
  int error;
};

// Demonstrate some basic assertions.
TEST(serialcomm, port) {
  string port = "/dev/cu.usbmodem1234";

  SerialPort ser;
  bool ok = ser.open(port, B2000000);
  EXPECT_FALSE(ok);
  EXPECT_FALSE(ser);
  EXPECT_FALSE(ser.is_open());
  ser.close();
}

TEST(serialcomm, read_write_byte) {
  SerialPipe sp; // create virtual serial port for testing
  sp.init();

  SerialPort s;
  bool ok = s.open(sp.masterfd);
  EXPECT_TRUE(ok);

  SerialPort p;
  ok = p.open(sp.slavefd);
  EXPECT_TRUE(ok);

  // read-write byte ---------------------
  int num = s.write(12);
  EXPECT_EQ(num, 1);

  num = p.available();
  EXPECT_EQ(num, 1);

  num = p.read();
  EXPECT_EQ(num, 12);

  // done --------------------------------
  s.close();
  p.close();
}

TEST(serialcomm, read_write_string) {
  SerialPipe sp; // create virtual serial port for testing
  sp.init();

  SerialPort s;
  bool ok = s.open(sp.masterfd);
  EXPECT_TRUE(ok);

  SerialPort p;
  ok = p.open(sp.slavefd);
  EXPECT_TRUE(ok);

  // read-write string -------------------
  string msg = "hello";
  int num    = s.write(msg);
  EXPECT_EQ(num, 5);

  num = p.available();
  EXPECT_EQ(num, 5);
  // cout << "available: " << num << endl;

  string str = p.readString();
  // for (const char& chr: "hello") {
  //   char c = p.read();
  //   EXPECT_EQ(c, chr);
  // }
  EXPECT_STREQ(str.c_str(), msg.c_str());
  EXPECT_EQ(str.size(), msg.size());

  // done --------------------------------
  s.close();
  p.close();
}

TEST(serialcomm, read_write_buffer) {
  SerialPipe sp; // create virtual serial port for testing
  sp.init();

  SerialPort s;
  bool ok = s.open(sp.masterfd);
  EXPECT_TRUE(ok);

  SerialPort p;
  ok = p.open(sp.slavefd);
  EXPECT_TRUE(ok);

  // read-write buffer -------------------
  msg_t m{1, 2, 3, 4};
  int num = s.write((void *)&m, sizeof(msg_t));
  EXPECT_EQ(num, 16);

  num = p.available();
  EXPECT_EQ(num, 16);

  msg_t n;
  num = p.readBytes((uint8_t *)&n, sizeof(n));
  EXPECT_EQ(num, 16);
  EXPECT_EQ(m.x, n.x);
  EXPECT_EQ(m.y, n.y);
  EXPECT_EQ(m.z, n.z);
  EXPECT_EQ(m.error, n.error);

  // done --------------------------------
  s.close();
  p.close();
}