# `pty` — Pseudo-terminal

## Python

```python
>>> import pty
>>> import os
>>> from serial import Serial
>>> master, slave = pty.openpty()
>>> port = os.ttyname(slave)
>>> port
'/dev/ttys004'
>>> serial = Serial(port)
>>> n = serial.write(b"hi")
>>> print(">> wrote:", n)
>> wrote: 2
>>> os.read(master,2)
b'hi'
>>> os.write(master,"back")
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
TypeError: a bytes-like object is required, not 'str'
>>> os.write(master,b"back")
4
>>> serial.read(4)
b'back'
>>> serial.close()
```

## C++

```c++
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
```