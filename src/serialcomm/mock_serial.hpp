/**************************************************\
* The MIT License (MIT)
* Copyright (c) 2019 The Guild of Calamitous Intent
* see LICENSE for full details
\**************************************************/
#pragma once

#include <fcntl.h> // open
#include <stdlib.h>
#include <unistd.h> // close
#if defined(__APPLE__)
  #include <util.h>
#elif defined(__linux__)
  #include <pty.h>
  #include <termios.h> // tcgetattr, struct termios
#endif

/*
(py)  kevin@Logan ~ % stty -f /dev/ttys006
speed 9600 baud;
lflags: echoe echoke echoctl
oflags: -oxtabs
cflags: cs8 -parenb

(py)  kevin@Logan ~ % stty -f /dev/ttys006
speed 9600 baud;
lflags: -icanon -isig -iexten -echo echoke echoctl
iflags: -icrnl -ixon -imaxbel ignbrk -brkint
oflags: -opost -oxtabs
cflags: cs8 -parenb
*/

struct SerialPipe {
  SerialPipe() : masterfd(0), slavefd(0) {}

  ~SerialPipe() {
    close(masterfd);
    close(slavefd);
  }

  bool init(bool info = false) {
    masterfd = ::open("/dev/ptmx", O_RDWR | O_NOCTTY);
    if (masterfd == -1) {
      perror("Couldn't open /dev/ptmx");
      return false;
    }

    grantpt(masterfd);
    unlockpt(masterfd);
    char *slavename = ptsname(masterfd);
    if (slavename == NULL) {
      perror("Couldn't get slave pseudoterminal");
      return false;
    }

    slavefd = ::open(slavename, O_RDWR | O_NOCTTY);
    if (slavefd == -1) {
      perror("Couldn't open slave pseudoterminal");
      return false;
    }

    tty = std::string(slavename);

    // fixes slave write, master read issue -----
    // https://stackoverflow.com/questions/23459520/how-to-read-write-to-linux-pseudoterminals-using-separate-processes-but-without
    struct termios ts;
    if (tcgetattr(slavefd, &ts)) {
      perror("tcgetattr");
      return false;
    }
    cfmakeraw(&ts);
    tcsetattr(slavefd, TCSANOW, &ts);
    // tty = ttyname(slavefd); // use ptsname instead

    if (info) {
      printf("-----------------------------------------------\n");
      printf("| Master fd: %d\n", masterfd);
      printf("| Slave fd: %d  tty: %s\n", slavefd, tty.c_str());
      printf("-----------------------------------------------\n");
    }

    return true;
  }

  int masterfd;
  int slavefd;
  std::string tty;
};