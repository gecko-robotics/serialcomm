#include "serial.hpp"
#include <sys/ioctl.h> // ioctl, dtr or rts
#include <fcntl.h>     // open
#include <termios.h>   // serial
#include <string.h>    // memset
#include <unistd.h>    // write(), read(), close()
#include <errno.h>     // errno
#include <iostream>
// #include <sys/file.h> // flock

using namespace std;


string getError(){
    extern int errno;
    return string(strerror( int(errno) ));
}

Serial::Serial(): fd(0) /*, dir_pin(TIOCM_DTR)*/ {}
Serial::~Serial(){close();}

int Serial::available(){
    int bytes;
    ioctl(fd, FIONREAD, &bytes);
    return bytes;
}

// Opens the serial port
// @port: string containing the file path
// @return: boolean for was the port opened? True success, false failed
bool Serial::open(const std::string& port, int speed){
    struct termios t;

    // O_RDWR means that the port is opened for both reading and writing.
    // O_NOCTTY means that no terminal will control the process opening the serial port.
    // fd = ::open(port.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK);
    fd = ::open(port.c_str(), O_RDWR|O_NOCTTY);
    if (fd < 0) throw SerialError("open(): " + getError());

    memset(&t, 0, sizeof(t)); // clear struct for new port settings

    // struct termios
    // {
    // tcflag_t c_iflag; /* input mode flags   */
    // tcflag_t c_oflag; /* output mode flags  */
    // tcflag_t c_cflag; /* control mode flags */
    // tcflag_t c_lflag; /* local mode flags   */
    // };
    // Dynamixel SDK method
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
    if (tcflush(fd, TCIFLUSH) < 0) perror("*** Couldn't flush input buffer");
    if (tcsetattr(fd, TCSANOW, &t) < 0) perror("*** Couldn't set port attribute");

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
    int ret;
    if ((ret = ::write(fd, buf, size)) < 0){
        throw SerialError("write(): " + getError());
    }
    return ret;
}

// Reads the input serial buffer
// @returns: number of bytes read
int Serial::read(){
    int num = 0;
    int size = sizeof(buffer);
    memset(buffer, 0, size);
    num = 0;
    if ((num = ::read(fd, (void*)buffer, size)) < 0){
        throw SerialError("read(): " + getError());
    }
    return num;
}

// Reads the input serial buffer
// @returns: number of bytes read
int Serial::read(void* buf, int size){
    int num = 0;

    // num = ::read(fd, buffer.data(), buffer.size());
    uint8_t b[128];
    uint8_t *p = b;
    num = 0;
    // num = ::read(fd, b, 10);
    // for (int i=0; i <10; ++i) {
    //     num += ::read(fd, p, 1);
    //     if (*p < 0) printf("oops %d", *p);
    //     p++;
    // }
    if ((num = ::read(fd, buf, size)) < 0){
        throw SerialError("read(): " + getError());
    }
    return num;
}

// Flush input buffer
void Serial::flush_input(){
    if (tcflush(fd, TCIFLUSH) < 0) throw SerialError("flush(): " + getError());
}

// Flush output buffer
void Serial::flush_output(){
    if (tcflush(fd, TCOFLUSH) < 0) throw SerialError("flush(): " + getError());
}

// Flush both input and output buffers
void Serial::flush(){
    if (tcflush(fd, TCIOFLUSH) < 0) throw SerialError("flush(): " + getError());
}
