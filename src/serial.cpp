// #include <mv/SerialPort.hpp>
#include <sys/ioctl.h>  // dtr or rts
#include <fcntl.h>      // open
#include <termios.h>    // serial
#include <string.h>     // memset
#include <iostream>
// #include <sys/file.h> // flock
// #include <termcolor/termcolor.hpp>


constexpr bool DD_WRITE = false;  // false
constexpr bool DD_READ = !DD_WRITE;

using namespace std;

/*
   Instr Pkt                      Status Pkt
--////////////-------------------///////////------
              Return Delay Time

Return Delay Time: set by Reg 5 (AX-12), default to 250 (0.5msec)
Packet Times: time = 8*num_bytes/DataRate

shortest status packet = 6 bytes
Packet Times (msec)
57600    0.833
115200   0.417
1000000  0.048
*/

Serial::Serial(): fd(0), dir_pin(TIOCM_DTR) {}
Serial::~Serial(){close();}

// Opens the serial port
// @port: string containing the file path
// @return: boolean for was the port opened? True success, false failed
bool Serial::open(const std::string& port, int speed){
    struct termios t;

    // O_RDWR means that the port is opened for both reading and writing.
    // O_NOCTTY means that no terminal will control the process opening the serial port.
    // fd = ::open(port.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK);
    fd = ::open(port.c_str(), O_RDWR|O_NOCTTY);
    if(fd < 0){
        perror("Error opening serial port");
        return false;
    }

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

    set_dir(DD_READ);
    msleep(100);
    set_dir(DD_WRITE);
    // set_dir(DD_READ); // ?

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
int Serial::write(const packet& pkt){
    set_dir(DD_WRITE);
    // int num = 0;
    // if ((num = available()) > 0) printf("** Data in input: %d\n", num);
    flush_input();
    int ret = ::write(fd, pkt.data(), pkt.size());
    // for (const auto& p: pkt) ::write(fd, (void*)&p, 1);
    // int ret = pkt.size();
    // flush_output();

    // KEEP this delay ... not sure what the minimum should be
    int min_time = 100;
    int delay = int(1e6*double(pkt.size())/1e6);
    delay = delay > min_time ? delay : min_time;
    // printf(">> delay: %d\n", delay);
    usleep(delay);
    set_dir(DD_READ);
    return ret;
}

// Reads the input serial buffer
// @returns: number of bytes read
int Serial::read(){
    int num = 0;
    // set_dir(DD_READ);
    // smallest packet is 6 bytes
    // int num = available();
    // printf(">> available: %d\n", num);
    // if (num < 6) return 0;

    // num = (num > 0) ? num : 10;
    //
    // buffer.fill(0);
    // size_t remain = num;
    // while(remain > 0){
    //     remain -= ::read(fd, buffer.data() + remain, remain);
    // }

    // num = ::read(fd, buffer.data(), buffer.size());
    uint8_t b[128];
    uint8_t *p = b;
    num = 0;
    // num = ::read(fd, b, 10);
    for (int i=0; i <10; ++i) {
        num += ::read(fd, p, 1);
        if (*p < 0) printf("oops %d", *p);
        p++;
    }
    // num = ::read(fd, buffer.data(), num);

    // set_dir(DD_WRITE);
    // TODO: check error code and return it if error
    return num;
}

/*
do something like python:
status: {'id': 1, 'error str': 'None', 'error num': 0, 'params': [253, 1], 'raw': [255, 255, 1, 4, 0, 253, 1, 252]}
*/
packet Serial::buffer2packet(int num, int offset){
    for (int i=0; i < 10; ++i) cout << int(buffer[i]) << ",";
    cout << endl;
    auto pkt = packet(num);
    std::copy(buffer.begin(), buffer.begin() + offset, pkt.begin());
    return pkt;
}

status_t Serial::decode(){
    status_t ret;
    // resp: [s,s,id,len,err, ... ,chksum]
    for (int i = 0; i < buffer.size() - 2; ++i) {
        if (buffer[i] == 0xff && buffer[i+1] == 0xff) {
            ret.id = buffer[i+2];
            ret.error = buffer[i+4];
            for (int j=0; j < buffer[i+3]-2; ++j) ret.params.push_back(buffer[i+5+j]);
            break;
        }
    }

    return ret;
}

// Flush input buffer
void Serial::flush_input(){
    tcflush(fd, TCIFLUSH);
}

// Flush output buffer
void Serial::flush_output(){
    tcflush(fd, TCOFLUSH);
}

// Flush both input and output buffers
void Serial::flush(){
    tcflush(fd, TCIOFLUSH);
}

// See how many bytes are available in the input buffer to read
int Serial::available(){
    int bytes_available;
    ioctl(fd, FIONREAD, &bytes_available);
    return bytes_available;
}

// Toggles both the RTS and DTR pins to signal TX or RX
// TODO: change to RTS or DTR, not both, but configurable
void Serial::set_dir(bool enabled){
    // int pin = 0;
    dir_pin = TIOCM_DTR;
    int value = enabled ? TIOCMBIS : TIOCMBIC;
    ioctl(fd, value, &dir_pin);

    // int pin = 0;
    // if (enabled){
    //     // pin = TIOCM_RTS;
    //     // ioctl(fd, TIOCMBIS, &pin);
    //     pin = TIOCM_DTR;
    //     ioctl(fd, TIOCMBIS, &pin);
    // }
    // else {
    //     // pin = TIOCM_RTS;
    //     // ioctl(fd, TIOCMBIC, &pin);
    //     pin = TIOCM_DTR;
    //     ioctl(fd, TIOCMBIC, &pin);
    // }
}
