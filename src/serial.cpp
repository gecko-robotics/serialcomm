/**************************************************\
* The MIT License (MIT)
* Copyright (c) 2019 Kevin Walchko
* see LICENSE for full details
\**************************************************/
#include "serialcomm/serial.hpp"
#include <sys/ioctl.h> // ioctl, dtr or rts
#include <fcntl.h>     // open
#include <termios.h>   // serial
#include <string.h>    // memset
#include <unistd.h>    // write(), read(), close()
#include <errno.h>     // errno
// #include <iostream>
// #include <sys/file.h> // flock

using namespace std;


string getError(){
    extern int errno;
    return string(strerror( int(errno) ));
}

int guard(int err, std::string msg){
    if (err < 0) { throw SerialError(msg + getError()); }
    return err;
}

Serial::Serial(): fd(0) {}
Serial::~Serial(){ this->close(); }

/**
 * Get number of bytes waiting to be read.
 */
int Serial::available(){
    int bytes;
    guard(ioctl(fd, FIONREAD, &bytes), "available(): ");
    return bytes;
}

// Opens the serial port
// @port: string containing the file path
// @return: boolean for was the port opened? True success, false failed
bool Serial::open(const std::string& port, int speed){
    // struct termios t;

    // O_RDWR means that the port is opened for both reading and writing.
    // O_NOCTTY means that no terminal will control the process opening the serial port.
    // fd = ::open(port.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK);
    // guard(fd = ::open(port.c_str(), O_RDWR|O_NOCTTY), "open(): ");
    guard(fd = ::open(port.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK), "open(): ");

    this->flush();

    struct termios t;
    memset(&t, 0, sizeof(t)); // clear struct for new port settings

    // struct termios
    // {
    // tcflag_t c_iflag; /* input mode flags   */
    // tcflag_t c_oflag; /* output mode flags  */
    // tcflag_t c_cflag; /* control mode flags */
    // tcflag_t c_lflag; /* local mode flags   */
    // };
    // *** Note: memset clears a lot of flags, all we need to do is
    //           set the ones we care about.
    // CS8 - (1), 8bits set
    // PARENB - (0), parity cleared
    // CSTOPB - (0), stop bits set to 1 by clearing it
    // CREAD - (1), receiver turned on
    // CRTSCTS - (0), HW flow control off by clearing it
    // IXON | IXOFF | IXANY - (0), SW flow control off by clearing it
    // CLOCAL - (1), ignore modem control lines
    // IGNPAR - (1), ignore framing and parity errors
    // [VMIN] - (0), minimum number of characters read before read() reaturns
    // [VTIME] - (0), minimum time to wait before returning, 0 - immediately
    t.c_cflag = speed | CS8 | CLOCAL | CREAD;
    // t.c_cflag = B1000000 | CS8 | CLOCAL | CREAD;
    // t.c_cflag = B57600 | CS8 | CLOCAL | CREAD;
    // t.c_cflag = B19200 | CS8 | CLOCAL | CREAD;
    t.c_iflag = IGNPAR;
    t.c_oflag      = 0;
    t.c_lflag      = 0;
    t.c_cc[VTIME]  = 0; // 10th of second, 1 = 0.1 sec, time to block before return
    t.c_cc[VMIN]   = 0; // min number of characters before return



    // t.c_cflag |= (tcflag_t) (CLOCAL | CREAD);
    // t.c_cflag &= B1000000;
    // t.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
    //                                    ISIG | IEXTEN); //|ECHOPRT
    // t.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);
    // // t.c_oflag      = 0;
    // t.c_oflag &= (tcflag_t) ~(OPOST);
    // // t.c_lflag      = 0;
    // t.c_cc[VTIME]  = 0; // 10th of second, 1 = 0.1 sec
    // t.c_cc[VMIN]   = 0;
    //
    // // 8N1
    // t.c_cflag &= ~PARENB;
    // t.c_cflag &= ~CSTOPB;
    // t.c_cflag &= ~CSIZE;
    // t.c_cflag |= CS8;

    // disable HW flow control
    // t.c_cflag &= ~CNEW_RTSCTS;

    // disable SW flow control
    // t.c_iflag &= ~(IXON | IXOFF | IXANY);

    // clean the buffer and activate the settings for the port
    // if (tcflush(fd, TCIFLUSH) < 0) perror("*** Couldn't flush input buffer");
    // if (tcsetattr(fd, TCSANOW, &t) < 0) perror("*** Couldn't set port attribute");
    guard(tcflush(fd, TCIFLUSH), "open(): ");
    guard(tcsetattr(fd, TCSANOW, &t), "open(): ");

    // if (tcgetattr(fd, &t) < 0) perror("Couldn't get port attribute");
    // printf(">> %u %u \n", cfgetispeed(&t), cfgetospeed(&t));

    // flush();

    // if(flock(fd, LOCK_EX | LOCK_NB) == -1) {
    //     throw std::runtime_error("Serial port with file descriptor " +
    //         std::to_string(fd) + " is already locked by another process.");
    // }

    return true;
}

// Closes the serial port file descriptor
void Serial::close(){
    if (fd > 0) ::close(fd);
}

// Writes packets to the output buffer for transmit
// @return: bytes written
int Serial::write(const void* buf, int size){
    int ret = guard(::write(fd, buf, size), "write(buffer): ");
    return ret;
}

int Serial::write(const string& s){
    int ret = guard(::write(fd, (void*)s.c_str(), s.size()), "write(string): ");
    return ret;
}

int Serial::write(const uint8_t byte){
    int ret = guard(::write(fd, (void*)&byte, 1), "write(byte): ");
    return ret;
}

// Reads the input serial buffer
// @returns: number of bytes read
std::string Serial::readString(){
    string ret;
    char c;
    while (true) {
        int num = guard(::read(fd, (void*)&c, 1), "readString(): ");
        if (num == 1) ret.push_back(c);
        else break;
    }
    return ret;
}

// Reads the input serial buffer
// @returns: number of bytes read
int Serial::read(){
    string ret;
    int c;
    int num = guard(::read(fd, (void*)&c, 1), "read(): ");
    if (num <=0) c = -1;
    return c;
}

// Reads the input serial buffer
// @returns: number of bytes read
int Serial::readBytes(uint8_t* buf, int size){
    int num = 0;
    memset(buf, 0, size);

    while (num < size) {
        num += guard(::read(fd, (void*)&buf[num], size-num), "readBytes(): ");
    }

    return num;
}

// Flush input buffer
void Serial::flush_input(){
    guard(tcflush(fd, TCIFLUSH), "flush_input(): ");
}

// Flush output buffer
void Serial::flush_output(){
    guard(tcflush(fd, TCOFLUSH), "flush_output(): ");
}

// Flush both input and output buffers
void Serial::flush(){
    guard(tcflush(fd, TCIOFLUSH), "flush(): ");
}

void Serial::set_dtr(bool enabled){
    int pin = TIOCM_DTR;
    int value = enabled ? TIOCMBIS : TIOCMBIC;
    guard(ioctl(fd, value, &pin), "set_dtr(): ");
}

void Serial::set_rts(bool enabled){
    int pin = TIOCM_RTS;
    int value = enabled ? TIOCMBIS : TIOCMBIC;
    guard(ioctl(fd, value, &pin), "set_rts(): ");
}

void Serial::setTimeout(int time) {
  // FIXME: fix
}
