#ifndef __DAC81408_H__
#define __DAC81408_H__

#include <Arduino.h>
#include <SPI.h>

class DAC81408 {
  public:
    DAC81408();
    void init(int slavePin);
    void set(int channel, double voltage);
  private:
  	void spi_transfer(byte *data, int len);
  	int slavePin;
  	int DAC_CHANNELS[8] = {0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B};
};

#endif