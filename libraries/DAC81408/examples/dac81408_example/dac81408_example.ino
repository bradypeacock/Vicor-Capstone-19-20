#include <DAC81408.h>

#define slavePin 10

DAC81408 DAC;

void setup() {
  Serial.begin(9600);
  Serial.println("DAC81408 example");

  // Initialize DAC and set 2.5V on channel 1
  DAC.init(slavePin);
  byte data[3] = {0x14, 0x80, 0x00};
  DAC.spi_transfer(data, 3);
}

void loop() {
  // put your main code here, to run repeatedly:

}
