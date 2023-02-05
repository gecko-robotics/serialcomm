#include "serialcomm/serial.hpp"
#include "serialcomm/mock.hpp"
#include <iostream>
#include <string>

using namespace std;

int main() {
  SerialPipe sp;

  Serial sa, sb;
  sa.open(sp.masterfd);
  sb.open(sp.slavefd);

  char bufa[128];
  char bufb[128];
  memcpy((void*)bufa, (void*)"hi how are you", 14);
  sa.write((void*)bufa, 14);
  sb.readBytes((uint8_t*)bufb, 14);
  cout << bufb << endl;

  int wr = sa.write(100);
  uint8_t b = sb.read();
  cout << wr << " " << (int)b << endl;

  wr = sb.write(200);
  b = sa.read();
  cout << wr << " "  << (int)b << endl;

  return 0;
}


// int main()  {
//   char buf[128];
//   char msg[] = "hello there";
//   char *slavename;
//   int masterfd;
//   int slavefd;
//   int ss;
//   int sz;

//   // masterfd = getpt(); // not defined macos
//   masterfd = ::open("/dev/ptmx", O_RDWR | O_NOCTTY);
//   // masterfd = ::open("/dev/ptmx", O_RDWR | O_NOCTTY | O_NONBLOCK);
//   // openpty(&masterfd, &slavefd, NULL, NULL, NULL);

//   grantpt(masterfd);
//   unlockpt(masterfd);
//   slavename = ptsname(masterfd);

//   slavefd = ::open(slavename, O_RDWR | O_NOCTTY);
//   // slavefd = ::open(slavename, O_RDWR | O_NOCTTY | O_NONBLOCK);

//   // fixes slave write, master read issue -----
//   struct termios ts;

//   if(tcgetattr(slavefd, &ts))
//   {
//     perror("tcgetattr");
//     exit(1);
//   }

//   cfmakeraw(&ts);
//   tcsetattr (slavefd, TCSANOW, &ts);
//   //---------------------

//   ss = slavefd;

//   // cout << "master " << ttyname(masterfd) << endl; // segfault
//   cout << "master " << masterfd << endl;
//   cout << "slave " << ttyname(slavefd) << endl;
//   cout << slavename << " " << slavefd << endl;

//   cout << "slave write: " << ::write(ss, msg, 10) << " " << msg << endl;
//   sz = ::read(masterfd, (void*)buf, 10);
//   cout << "master read " << sz << " " << buf << endl;

//   memset(buf,0,10);

//   cout << "master write: " << ::write(masterfd, msg, 10) << " " << msg << endl;
//   sz = ::read(slavefd, (void*)buf, 10);
//   cout << "slave read " << sz << " " << buf << endl;

//   // sleep(5);

//   close(slavefd);
//   close(masterfd);
//   return 0;
// }



