#include <cstdint>
#include <cstdio>
#include <iostream>
#include <serialcomm/serialcomm.hpp>
#include <string>
#include <unistd.h>

using namespace std;

// % stty -f /dev/ttys005
// speed 9600 baud;
// lflags: -icanon -isig -iexten -echo echoke echoctl
// iflags: -icrnl -ixon -imaxbel ignbrk -brkint
// oflags: -opost -oxtabs
// cflags: cs8 -parenb

int main() {
  sleep(2);

  SerialPipe s;
  if (!s.init()) return 1;

  cout << ">> master " << s.masterfd << endl;
  cout << ">> slave " << s.tty << " " << s.slavefd << endl;

  char bufin;

  string buffer{"deadcownow"};

  for (const char &c : buffer) {
    write(s.masterfd, &c, 1);
    int num = read(s.masterfd, &bufin, 1);
    printf("read %i bytes: %c\n", num, bufin);
  }

  return 0;
}
