# Serial Communications

- `::open`:
    - **O_NOCTTY:** If pathname refers to a terminal device — it will not become the process's controlling terminal even if the process does not have one. O_NOFOLLOW : If pathname is a symbolic link, then the open fails.
    - **O_NONBLOCK:** When possible, the file is opened in non-blocking mode. Neither the open() nor any subsequent operations on the file descriptor which is returned will cause the calling process to wait. This option is equivalent to O_NODELAY option.
    - **O_SYNC:** The file is opened for synchronous I/O. Any write on the resulting file descriptor will block the calling process until the data has been physically written to the underlying hardware.

```c
#include <stdio.h>
#include <fcntl.h>     // File Control Definitions
#include <termios.h>   // POSIX Terminal Control Definitions
#include <unistd.h>    // UNIX Standard Definitions
#include <errno.h>     // ERROR Number Definitions
#include <sys/ioctl.h> // define both TIOCMBIS and TIOCMBIC

/** References
https://xanthium.in/Serial-Port-Programming-on-Linux
http://xanthium.in/Controlling-RTS-and-DTR-pins-SerialPort-in-Linux
*/

int init(int fd, int speed){
  int err = 0;
  struct termios options;
  memset (&tty, 0, sizeof tty);  // clear it

  // grab current settings
  err = tcgetattr(fd, &options);
  if (err != 0){
    printf("error reading port options");
    return 1;
  }

  // set input/output speeds
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);

  // no blocking
  options.c_cc[VMIN]  = 0;    // no minimum number of chars to read
  options.c_cc[VTIME] = 5;    // 0.5 seconds read timeout

  options.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  // 8N1
  options.c_cflag &= ~PARENB
  options.c_cflag &= ~CSTOPB
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  options.c_iflag &= ~IGNBRK;  // disable break processing
  options.c_lflag = 0;         // no signaling chars, no echo,
                               // no canonical processing
  options.c_oflag = 0;         // no remapping, no delays

  // set HW flow control
  // options.c_cflag |= CNEW_RTSCTS;

  // clear HW flow control
  options.c_cflag &= ~CNEW_RTSCTS;

  // set port options
  err = tcsetattr(fd, TCSANOW, &options);
  if (err != 0){
    printf("error setting up port");
    return 1;
  }

  return 0;
}

// can do the same for DTR: TIOCM_DTR
void setRTS(int fd){
  int RTS_flag = TIOCM_RTS;       // defined in termios.h
  ioctl(fd, TIOCMBIS, &RTS_flag); // Set RTS pin
}

void clearRTS(int fd){
  int RTS_flag = TIOCM_RTS;       // defined in termios.h
  ioctl(fd, TIOCMBIC, &RTS_flag); // Clear RTS pin
}

int main(){
  int fd;

  // O_RDWR   - Read/Write access to serial port
  // O_NOCTTY - No terminal will control the process
  // O_SYNC - data is written synchronously
  // O_NDELAY - tells UNIX to ignore state of DCD signals
  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);

  if (fd == -1){
    printf("\n  Error! in Opening ttyUSB0\n\n");
    return 1;
  }
  else
    printf("\n\n  ttyUSB0 Opened Successfully\n\n");

  int err = init(fd, B115200);
  if (err != 0) return 1;

  // do stuff ------------------------
  write (fd, "hello!\n", 7);
  usleep ((7 + 25) * 100);
  char buf [100];
  int n = read (fd, buf, sizeof buf);

  // All done -------------------------
  close(fd);
  return 0;
}
```



## Reference

- [Understanding Linux open() system call)](https://www.ibm.com/developerworks/community/blogs/58e72888-6340-46ac-b488-d31aa4058e9c/entry/understanding_linux_open_system_call?lang=en)
- [ref](https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c)
- [ref](https://github.com/rm5248/CSerial/blob/master/examples/example_full.c)
- [ref](https://github.com/rm5248/CSerial/blob/master/c_serial.h)
- [ref](https://github.com/rm5248/CSerial/blob/master/c_serial.c)
- [Serial Programming Guide for POSIX Operating Systems](https://www.cmrr.umn.edu/~strupp/serial.html#3_1)