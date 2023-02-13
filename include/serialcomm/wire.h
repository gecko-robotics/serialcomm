/**************************************************\
* The MIT License (MIT)
* Copyright (c) 2019 The Guild of Calamitous Intent
* see LICENSE for full details
\**************************************************/
// https://github.com/esp8266/Arduino/blob/master/libraries/Wire/Wire.h

// not useful
// https://github.com/WiringPi/WiringPi/blob/master/wiringPi/wiringPiI2C.c

// https://docs.kernel.org/driver-api/i2c.html

// https://elinux.org/images/1/1e/I2C-SPI-ELC-2020.pdf

/*
https://www.kernel.org/doc/Documentation/i2c/dev-interface

ioctl(file, I2C_SMBUS, struct i2c_smbus_ioctl_data *args)
  If possible, use the provided i2c_smbus_* methods described below instead
  of issuing direct ioctls.

You can do plain i2c transactions by using read(2) and write(2) calls.
You do not need to pass the address byte; instead, set it through
ioctl I2C_SLAVE before you try to access the device.

You can do SMBus level transactions (see documentation file smbus-protocol
for details) through the following functions:
  __s32 i2c_smbus_write_quick(int file, __u8 value);
  __s32 i2c_smbus_read_byte(int file);
  __s32 i2c_smbus_write_byte(int file, __u8 value);
  __s32 i2c_smbus_read_byte_data(int file, __u8 command);
  __s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value);
  __s32 i2c_smbus_read_word_data(int file, __u8 command);
  __s32 i2c_smbus_write_word_data(int file, __u8 command, __u16 value);
  __s32 i2c_smbus_process_call(int file, __u8 command, __u16 value);
  __s32 i2c_smbus_read_block_data(int file, __u8 command, __u8 *values);
  __s32 i2c_smbus_write_block_data(int file, __u8 command, __u8 length,
                                   __u8 *values);
All these transactions return -1 on failure; you can read errno to see
what happened. The 'write' transactions return 0 on success; the
'read' transactions return the read value, except for read_block, which
returns the number of values read. The block buffers need not be longer
than 32 bytes.

The above functions are made available by linking against the libi2c library,
which is provided by the i2c-tools project.  See:
https://git.kernel.org/pub/scm/utils/i2c-tools/i2c-tools.git/.

*/

/**
set i2c speed
/boot/config.txt
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
*/
#pragma once

#ifndef TwoWire_h
#define TwoWire_h
#endif

#if defined(linux)
// sudo apt-get install libi2c-dev
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}
#endif

// #include <stddef.h>
#include <stdint.h>     // int types
#include <unistd.h>			// file read/write
#include <fcntl.h>		  // file open
#include <sys/ioctl.h>	// Needed for I2C port
#include <cstdio>       // printf / perror
#include <cstring>      // memset

constexpr uint8_t I2C_MAX_BUFFER_SIZE = 32;

class TwoWire {
  public:
  TwoWire();
  ~TwoWire();

  void    begin(int sda=0, int scl=0) {}
  void    begin(int sda, int scl, uint8_t address) {}
  void    set(uint8_t address);
  void    setClock(uint32_t) {}
  void    setClockStretchLimit(uint32_t) {}
  void    beginTransmission(uint8_t) {}
  void    beginTransmission(int) {}
  uint8_t endTransmission(void) {return 0;}
  uint8_t endTransmission(uint8_t) {return 0;}
  size_t  requestFrom(uint8_t address, size_t size, bool sendStop) {return 0;}
  // uint8_t status();

  uint8_t requestFrom(uint8_t, uint8_t) {return 0;}
  uint8_t requestFrom(uint8_t, uint8_t, uint8_t) {return 0;}
  uint8_t requestFrom(int, int) {return 0;}
  uint8_t requestFrom(int, int, int) {return 0;}

  size_t write(uint8_t) {return 0;}
  bool write(const uint8_t reg, const uint8_t data);
  uint8_t read(void) {return 0;}
  bool read(const uint8_t reg, const uint8_t count, uint8_t *const data);

  protected:
  int fd;
  uint8_t outbuf[2];
  #if defined(linux)
  struct i2c_msg msgs[2];
  struct i2c_rdwr_ioctl_data i2c_data;
  #endif
  uint8_t addr;
};

// extern TwoWire Wire;
