/**************************************************\
* The MIT License (MIT)
* Copyright (c) 2019 Kevin Walchko
* see LICENSE for full details
\**************************************************/
#pragma once

#include <errno.h> // errno
#include <exception>
#include <string>

class SerialError : public std::exception {
public:
  SerialError(const std::string &s) : msg(s) {}
  SerialError() : msg("Serial Error") {}
  const char *what() const throw() { return msg.c_str(); }

protected:
  std::string msg;
};

std::string getError() {
  extern int errno;
  return std::string(strerror(int(errno)));
}

int guard(int err, std::string msg) {
  if (err < 0) {
    throw SerialError(msg + getError());
  }
  return err;
}