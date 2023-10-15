#include <string>
using namespace std;

void setup() {
  Serial.begin(1000000);
  delay(100);
}

int cnt = 0;
typedef struct {
    float x,y,z;
    int error;
} msg_t;

void loop() {

  if (Serial.available() > 0) {
    if (Serial.read() == 'g') {
      msg_t ans{1.23,2.34,3.45,cnt++};
      Serial.write(reinterpret_cast<char*>(&ans), sizeof(ans));
    }
  }

}
