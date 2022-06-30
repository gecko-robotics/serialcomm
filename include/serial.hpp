/******************************************************************************
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
******************************************************************************/
#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <exception>

class SerialError : public std::exception {
public:
    SerialError(const std::string& s): msg(s) {}
    SerialError(): msg("Serial Error") {}
    const char* what () const throw () {return msg.c_str();}
protected:
    std::string msg;
};

// macos is broken!!!
#ifndef B1000000
#define B1000000 0010010
#endif

typedef struct {
    uint8_t id;
    uint8_t error;
    std::vector<uint8_t> params;
} status_t;

class Serial {
public:
    Serial();
    ~Serial();

    bool open(const std::string& port, int speed=B1000000);
    void close();
    int available();
    int write(const void* buffer, int size);
    int read();
    int read(void* buf, int size);
    void flush_input();
    void flush_output();
    void flush();
    void set_dtr(bool enabled);
    void set_rts(bool enabled);

protected:
    int fd;
    uint8_t buffer[512];
};
