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
#elif defined(linux)
#include <pty.h>
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
  // public:
  SerialPipe() {
    masterfd = ::open("/dev/ptmx", O_RDWR | O_NOCTTY);
    // masterfd = ::open("/dev/ptmx", O_RDWR | O_NOCTTY | O_NONBLOCK);

    grantpt(masterfd);
    unlockpt(masterfd);
    char *slavename = ptsname(masterfd);

    slavefd = ::open(slavename, O_RDWR | O_NOCTTY);
    // slavefd = ::open(slavename, O_RDWR | O_NOCTTY | O_NONBLOCK);

    // fixes slave write, master read issue -----
    // https://stackoverflow.com/questions/23459520/how-to-read-write-
    //   to-linux-pseudoterminals-using-separate-processes-but-without
    struct termios ts;
    if (tcgetattr(slavefd, &ts)) {
      perror("tcgetattr");
      exit(1);
    }
    cfmakeraw(&ts);
    tcsetattr(slavefd, TCSANOW, &ts);

    // cout << "master " << masterfd << endl;
    // cout << "slave " << ttyname(slavefd) << " " << slavefd << endl;
  }
  ~SerialPipe() {
    close(masterfd);
    close(slavefd);
  }

  int masterfd;
  int slavefd;
};