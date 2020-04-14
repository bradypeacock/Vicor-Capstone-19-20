#include "DAC81408.h"

#include <math.h>

#define DAC_CONV (pow(2, 16) - 1) / 5.0

DAC81408::DAC81408() {
}

void DAC81408::init(int slavePin) {
  this->slavePin = slavePin;
  pinMode(slavePin, OUTPUT);
  digitalWrite(slavePin, HIGH);

  byte data[3] = {0x00, 0x00, 0x00};
  // initialize config registers 0x03 - 0x0F
  for (byte i = 0x03; i <= 0x0F; i++) {
    // 0x0A and 0x0D are reserved registers
    if (i != 0x0A && i != 0x0D) {
      data[0] = i;
      data[1] = 0x00;
      data[2] = 0x00;
      spi_transfer(data, 3);
    }
  }
}

void DAC81408::set(int channel, double voltage) {
  int volt = voltage * DAC_CONV;
  byte data[3] = {DAC_CHANNELS[channel], volt>>8, volt & 0x00FF};
  spi_transfer(data, 3);
}

void DAC81408::spi_transfer(byte *data, int len) {
  SPI.begin();
  // 20KHz clock speed, MSB first, [CPOL|CPHA]=0b01
  SPI.beginTransaction(SPISettings(20000, MSBFIRST, SPI_MODE1));
  // Set slave select low to send data
  digitalWrite(slavePin, LOW);

  // This will put the data received back into "data" array after the transfer call
  SPI.transfer(data, len);

  // Set slave select back high once you are done sending data
  digitalWrite(slavePin, HIGH);
  SPI.endTransaction();
  SPI.end();
}