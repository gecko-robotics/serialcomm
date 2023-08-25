/**************************************************\
* The MIT License (MIT)
* Copyright (c) 2019 Kevin Walchko
* see LICENSE for full details
\**************************************************/
#pragma once


#include "serial_port.hpp"
#include "mock_serial.hpp"

#ifndef TwoWire_h
  #define TwoWire_h
#endif

#if defined(linux)
  #include "wire_linux.hpp"
#elif defined(__APPLE__)
  #include "wire_apple.hpp"
#endif