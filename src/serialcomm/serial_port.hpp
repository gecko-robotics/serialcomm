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
#include <exception>
#include <string>
// #include <errno.h>     // errno
#include <fcntl.h>     // open
#include <string.h>    // memset
#include <sys/ioctl.h> // ioctl, dtr or rts
#include <termios.h>   // serial
#include <unistd.h>    // write(), read(), close()

// class SerialError : public std::exception {
// public:
//   SerialError(const std::string &s) : msg(s) {}
//   SerialError() : msg("Serial Error") {}
//   const char *what() const throw() { return msg.c_str(); }

// protected:
//   std::string msg;
// };

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

// typedef struct {
//     uint8_t id;
//     uint8_t error;
//     std::vector<uint8_t> params;
// } status_t;

// enum class SerialError: int {
//   OK = 0,
//   ERROR_OPEN,
//   ERROR_FLUSH,
//   ERROR_IN_FLUSH,
//   ERROR_OUT_FLUSH,
//   ERROR_SET,
//   ERROR_SET_DTR,
//   ERROR_SET_RTS,
//   ERROR_AVAILABLE,
//   ERROR_TIMEOUT,
//   ERROR_PEAK
// };

// inline
// SerialError guard(int err, SerialError val) {
//   if (err < 0) return val;
//     // throw SerialError(msg + getError());
//   return SerialError::OK;
// }

/**
 * Serial port
 */
class Stream {
public:
  Stream() : fd(0) {}
  ~Stream() { this->close(); }

  void begin(int i) {} // does nothing

  bool open(const std::string &port, int speed) {

    // O_RDWR means that the port is opened for both reading and writing.
    // O_NOCTTY means that no terminal will control the process opening the
    // serial port.
    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      fd = 0;
      return false;
    }

    struct termios t;
    memset(&t, 0, sizeof(t)); // clear struct for new port settings

    t.c_cflag = speed | CS8 | CLOCAL | CREAD;
    t.c_iflag = IGNPAR;
    t.c_oflag = 0;
    t.c_lflag = 0;

    // VTIME: 10th of second, 1 = 0.1 sec, time to block
    // before return
    t.c_cc[VTIME] = 0;
    // VMIN: min number of characters before return
    t.c_cc[VMIN] = 0;

    // activate the settings for the port
    if (tcsetattr(fd, TCSANOW, &t) < 0) return false;

    return true;
  }

  bool open(int d) {
    fd = d;
    return true;
  }

  void close() {
    if (fd > 0) ::close(fd);
  }

  int write(const uint8_t byte) {
    // int ret = guard(::write(fd, (void *)&byte, 1), "write(byte): ");
    return ::write(fd, (void *)&byte, 1);
  }
  int write(const std::string &s) {
    // int ret =
    //     guard(::write(fd, (void *)s.c_str(), s.size()), "write(string): ");
    return ::write(fd, (void *)s.c_str(), s.size());
  }
  int write(const void *buffer, int size) {
    // int ret = guard(::write(fd, buffer, size), "write(buffer): ");
    return ::write(fd, buffer, size);
  }

  inline void print(const std::string &s) { write(s); }
  inline void println(const std::string &s) { write(s + "\n"); }

  int read() {
    // std::string ret;
    int c;
    int num = ::read(fd, (void *)&c, 1);
    // int num = guard(::read(fd, (void *)&c, 1), "read(): ");
    if (num <= 0) c = -1; // timeout if non-blocking
    return c;
  }

  std::string readString() {
    std::string ret;
    char c;
    int num = available();
    for (int i = 0; i < num; ++i) {
      int num =
          ::read(fd, (void *)&c,
                 1); // guard(::read(fd, (void *)&c, 1), "readString(): ");
      if (num < 0) break;
      ret.push_back(c);
    }
    return ret;
  }

  int readBytes(uint8_t *buf, int size) {
    int num = 0;
    memset(buf, 0, size);

    while (num < size) {
      int err = ::read(fd, (void *)&buf[num],
                       size - num); // guard(::read(fd, (void *)&buf[num], size
                                    // - num), "readBytes(): ");
      if (err < 0) return err;
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

  bool setTimeout(int time) { /* FIXME */
    return false;
  }

  int available() {
    int bytes;
    // guard(ioctl(fd, FIONREAD, &bytes), "available(): ");
    if (ioctl(fd, FIONREAD, &bytes) < 0) return -1;
    return bytes;
  }

  inline int peek() { return -1; } // FIXME: can I do thiis?
  // int peek() {
  //   int c = fgetc(fd);
  //   ungetc(c, fd);
  //   return c;
  // }

  inline explicit operator bool() const noexcept { return bool(fd); }

  inline const bool is_open() const { return (fd > 0) ? true : false; }

protected:
  int fd;
  // std::string error_string; // is something like this better than throw/catch
  // error?
};

class SerialPort : public Stream {};
