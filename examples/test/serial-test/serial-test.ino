#include <string>
using namespace std;

void setup() {
  Serial.begin(1000000);
  delay(100);
}

int cnt = 0;

void loop() {
  if (Serial.available() > 0) {
    char end = '\n';
    char buff[64];
    size_t len = sizeof(buff);
    size_t howmany = Serial.readBytesUntil(end, buff, len);

    string rep(buff, howmany);
    rep += to_string(cnt++);

    Serial.write(rep.c_str(), rep.size());
  }

}
