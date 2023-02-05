![](pics/header.jpg)

![GitHub](https://img.shields.io/github/license/gecko-robotics/serialcomm)

# Serial Communications

A simple serial communications library, that mimic most of the Arduino serial
syntax. The goal is to use drivers between Arduino and C++ on linux easier with
little to no rewrite.

## Example

```c++
#include <serialcomm/serial.hpp>
#include <string>

int main(){
    Serial ser;
    ser.open("/dev/serial", B9600);

    try {
        uint8_t buffer[3] = {1,2,3,4};
        ser.write(buffer, sizeof(buffer));
        std::string msg("hello");
        ser.write(msg);

    }
    catch (const SerialError& e) {
        std::cout << e.what() << std::endl;
    }

    return 0
}
```

## Todo

- [ ] namespace ... gecko?
- [ ] fix the timeout and blocking
- [ ] determine either: return errors w/error string OR stay with catch/throw on error
- [x] toggle DTR/RTS pins
- [x] change version number to date? ex: 2022.6.25 ... version numbers mean nothing but this gives you an idea of what has been done when

## Reference

- [mbedded.ninja linux serial ports](https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/)

# MIT License

**Copyright (c) 2019 Kevin J. Walchko**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
