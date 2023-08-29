/**************************************************\
* The MIT License (MIT)
* Copyright (c) 2019 Kevin Walchko
* see LICENSE for full details
\**************************************************/
#pragma once

#include <cstdint> // uint8_t
#include <exception>
#include <string>
// #include <errno.h>     // errno
#include "helper.hpp"  // guard, get_error
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
    guard(fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK),
          "open(): ");

    this->flush();

    struct termios t;
    memset(&t, 0, sizeof(t)); // clear struct for new port settings

    t.c_cflag = speed | CS8 | CLOCAL | CREAD;
    t.c_iflag = IGNPAR;
    t.c_oflag = 0;
    t.c_lflag = 0;
    t.c_cc[VTIME] =
        0; // 10th of second, 1 = 0.1 sec, time to block before return
    t.c_cc[VMIN] = 0; // min number of characters before return

    // clean the buffer and activate the settings for the port
    guard(tcflush(fd, TCIFLUSH), "open(): ");
    guard(tcsetattr(fd, TCSANOW, &t), "open(): ");

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
    int ret = guard(::write(fd, (void *)&byte, 1), "write(byte): ");
    return ret;
  }
  int write(const std::string &s) {
    int ret =
        guard(::write(fd, (void *)s.c_str(), s.size()), "write(string): ");
    return ret;
  }
  int write(const void *buffer, int size) {
    int ret = guard(::write(fd, buffer, size), "write(buffer): ");
    return ret;
  }

  inline void print(const std::string &s) { write(s); }
  inline void println(const std::string &s) { write(s + "\n"); }

  int read() {
    // std::string ret;
    int c;
    int num = guard(::read(fd, (void *)&c, 1), "read(): ");
    if (num <= 0) c = -1;
    return c;
  }

  std::string readString() {
    std::string ret;
    char c;
    while (true) {
      int num = guard(::read(fd, (void *)&c, 1), "readString(): ");
      if (num == 1) ret.push_back(c);
      else break;
    }
    return ret;
  }
  int readBytes(uint8_t *buf, int size) {
    int num = 0;
    memset(buf, 0, size);

    while (num < size) {
      num += guard(::read(fd, (void *)&buf[num], size - num), "readBytes(): ");
    }

    return num;
  }

  void flush_input() { guard(tcflush(fd, TCIFLUSH), "flush_input(): "); }
  void flush_output() { guard(tcflush(fd, TCOFLUSH), "flush_output(): "); }
  void flush() { guard(tcflush(fd, TCIOFLUSH), "flush(): "); }

  void set_dtr(bool enabled) {
    int pin   = TIOCM_DTR;
    int value = enabled ? TIOCMBIS : TIOCMBIC;
    guard(ioctl(fd, value, &pin), "set_dtr(): ");
  }
  void set_rts(bool enabled) {
    int pin   = TIOCM_RTS;
    int value = enabled ? TIOCMBIS : TIOCMBIC;
    guard(ioctl(fd, value, &pin), "set_rts(): ");
  }

  void setTimeout(int time) { /* FIXME */
  }

  int available() {
    int bytes;
    guard(ioctl(fd, FIONREAD, &bytes), "available(): ");
    return bytes;
  }

  inline int peek() { return -1; } // FIXME: can I do thiis?
  // int peek() {
  //   int c = fgetc(fd);
  //   ungetc(c, fd);
  //   return c;
  // }

  inline explicit operator bool() const noexcept { return bool(fd); }

protected:
  int fd;
  // std::string error_string; // is something like this better than throw/catch
  // error?
};

class SerialPort : public Stream {};
