/*******************************************************************************\
MIT License

Copyright (c) 2019 Kevin J. Walchko

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
\********************************************************************************/
#pragma once

#include <cstdint> // uint8_t
// #include <exception>
#include <string>
// #include <errno.h>     // errno
#include <fcntl.h>     // open
#include <string.h>    // memset
#include <sys/ioctl.h> // ioctl, dtr or rts
#include <termios.h>   // serial
#include <unistd.h>    // write(), read(), close()


// These datarates should already be defined
// B0
// B50
// B75
// B110
// B134
// B150
// B200
// B300
// B600
// B1200
// B1800
// B2400
// B4800
// B9600
// B19200
// B38400
// B57600
// B115200
// B230400
//
// macos is broken!!!
// define some higher, missing data rates
#ifndef B1000000
  #define B1000000 0010010
  #define B2000000 0010013
  #define B2500000 0010014
  #define B3000000 0010015
  #define B3500000 0010016
  #define B4000000 0010017
#endif

// O_RDWR     - Read/Write access to serial port
// O_NOCTTY   - No terminal will control the process
// O_SYNC     - data is written synchronously
// O_NONBLOCK - will always return immediately and will not wait
// O_NDELAY   - tells UNIX to ignore state of DCD signals, same as O_NONBLOCK
constexpr int O_FLAGS_NONBLOCKING = O_RDWR | O_NOCTTY | O_NONBLOCK;
constexpr int O_FLAGS_BLOCKING = O_RDWR | O_NOCTTY;

// pass a struct into open() to give more options
// maybe this is a good idea or not ... not sure
// struct port_info_t {
//   std::string port; // /dev/tty.usbmodemXXXX
//   int baud{115200}; // 9600, 115200, 1000000, etc
//   int o_flags{O_FLAGS_NONBLOCKING}; // flags for open

//   // IF using blocking flags, you can put a timeout or char read
//   // to override, otherwise these are ignored
//   int timeout{0}; // 1 = 0.1 sec (100 msec), not a fine resolution
//   // int min_chars{0}; // number of chars to receive before return, don't use
// };

/**
 * Serial port
 */
class SerialPort {
public:
  SerialPort(): fd(0) {}
  SerialPort(const SerialPort &)  = delete; // copy
  SerialPort(const SerialPort &&) = delete; // move
  ~SerialPort() { this->close(); }

  // void begin(int i) {} // does nothing

  bool open(const std::string &port, int speed, int flags=O_FLAGS_NONBLOCKING, int timeout=0) {

    // O_RDWR means that the port is opened for both reading and writing.
    // O_NOCTTY means that no terminal will control the process opening the
    // serial port.
    fd = ::open(port.c_str(), flags);
    if (fd < 0) {
      fd = 0;
      return false;
    }

    int baud{0};

    // honestly I can't image any thing slower than GPS at 9600
    switch(speed) {
      case 2400:
        baud = B2400;
        break;
      case 4800:
        baud = B4800;
        break;
      case 9600:
        baud = B9600;
        break;
      case 19200:
        baud = B19200;
        break;
      case 38400:
        baud = B38400;
        break;
      case 57600:
        baud = B57600;
        break;
      case 115200:
        baud = B115200;
        break;
      case 230400:
        baud = B230400;
        break;
      case 1000000:
        baud = B1000000;
        break;
      case 2000000:
        baud = B2000000;
        break;
      default:
        return false;
    }

    struct termios t;
    memset(&t, 0, sizeof(t)); // clear struct for new port settings

    t.c_cflag = baud | CS8 | CLOCAL | CREAD;
    t.c_iflag = IGNPAR;
    t.c_oflag = 0; // already cleared by memset
    t.c_lflag = 0;
    t.c_cc[VTIME] = 0;
    t.c_cc[VMIN] = 0;
    // if blocking, we can set a timeout
    if (!(flags & O_NONBLOCK)) {
      // VTIME: 10th of second, 1 = 0.1 sec, time to block
      // before return
      t.c_cc[VTIME] = timeout;
      // VMIN: min number of characters before return
      t.c_cc[VMIN] = 0;
    }

    // activate the settings for the port
    if (tcsetattr(fd, TCSANOW, &t) < 0) return false;

    return true;
  }

  // bool open(int d) {
  //   fd = d;
  //   return true;
  // }

  void close() {
    if (fd > 0) ::close(fd);
  }

  int write(const uint8_t byte) {
    return ::write(fd, (void *)&byte, 1);
  }
  int write(const std::string &s) {
    return ::write(fd, (void *)s.c_str(), s.size());
  }
  int write(const void *buffer, int size) {
    return ::write(fd, buffer, size);
  }

  // inline void print(const std::string &s) { write(s); }
  // inline void println(const std::string &s) { write(s + "\n"); }

  // int read() {
  //   // std::string ret;
  //   int c;
  //   int num = ::read(fd, (void *)&c, 1);
  //   if (num <= 0) c = -1; // timeout if non-blocking
  //   return c;
  // }

  // std::string readString() {
  //   std::string ret;
  //   char c;
  //   int err;
  //   int num = available();
  //   for (int i = 0; i < num; ++i) {
  //     err = ::read(fd, (void *)&c, 1);
  //     if (err < 0) break;
  //     ret.push_back(c);
  //   }
  //   return ret;
  // }

  inline
  int read(uint8_t *buf, int size) { return readBytes(buf, size); }

  int readBytes(uint8_t *buf, int size) {
    int num = 0;
    memset(buf, 0, size);
    int err;

    while (num < size) {
      err = ::read(fd, (void *)&buf[num], size - num);
      if (err < 0) return err; // why isn't this num?
      num += err;
    }

    return num;
  }

  bool flush_input() {
    if (tcflush(fd, TCIFLUSH) < 0) return false;
    return true;
  }

  bool flush_output() {
    if (tcflush(fd, TCOFLUSH) < 0) return false;
    return true;
  }

  // flush both input/output
  bool flush() {
    if (tcflush(fd, TCIOFLUSH) < 0) return false;
    return true;
  }

  bool set_dtr(bool enabled) {
    int pin   = TIOCM_DTR;
    int value = enabled ? TIOCMBIS : TIOCMBIC;
    if (ioctl(fd, value, &pin) < 0) return false;
    return true;
  }

  bool set_rts(bool enabled) {
    int pin   = TIOCM_RTS;
    int value = enabled ? TIOCMBIS : TIOCMBIC;
    if (ioctl(fd, value, &pin) < 0) return false;
    return true;
  }

  // bool setTimeout(int time) { /* FIXME */
  //   return false;
  // }

  // FIXME: does this work on both macOS and Linux?
  int available() {
    int bytes;
    // guard(ioctl(fd, FIONREAD, &bytes), "available(): ");
    if (ioctl(fd, FIONREAD, &bytes) < 0) return -1;
    return bytes;
  }

  // inline int peek() { return -1; } // FIXME: can I do thiis?
  // int peek() {
  //   int c = fgetc(fd);
  //   ungetc(c, fd);
  //   return c;
  // }

  inline explicit operator bool() const noexcept { return bool(fd); }

  inline const bool is_open() const { return (fd > 0) ? true : false; }

protected:
  int fd{0};
  // std::string error_string; // is something like this better than throw/catch
  // error?
};

// class SerialPort : public Stream {};
