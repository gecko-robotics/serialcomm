/**************************************************\
* The MIT License (MIT)
* Copyright (c) 2019 Kevin Walchko
* see LICENSE for full details
\**************************************************/
#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <exception>
#include <termios.h>

class SerialError : public std::exception {
public:
    SerialError(const std::string& s): msg(s) {}
    SerialError(): msg("Serial Error") {}
    const char* what () const throw () {return msg.c_str();}
protected:
    std::string msg;
};

// macos is broken!!!
// define some higher, missing data rates
#ifndef  B1000000
#define  B1000000 0010010
#define  B2000000 0010013
#define  B2500000 0010014
#define  B3000000 0010015
#define  B3500000 0010016
#define  B4000000 0010017
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
    Stream();
    ~Stream();

    bool open(const std::string& port, int speed);
    bool open(int d){fd = d; return true;}
    void close();

    int write(const uint8_t byte);
    int write(const std::string& s);
    int write(const void* buffer, int size);

    int read();
    std::string readString();
    int readBytes(uint8_t* buf, int size);

    void flush_input();
    void flush_output();
    void flush();

    void set_dtr(bool enabled);
    void set_rts(bool enabled);

    void setTimeout(int time);

    int available(); // in_waiting()

protected:
    int fd;
    // std::string error_string; // is something like this better than throw/catch error?
};

class SerialPort: public Stream {

};
