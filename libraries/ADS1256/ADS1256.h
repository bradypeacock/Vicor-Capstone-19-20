#ifndef __ADS1256_H__
#define __ADS1256_H__

#include <Arduino.h>
#include <SPI.h>

class ADS1256 {
  public:
    ADS1256();
    bool init(int cs, int rst, int pdwn, int drdy);
    void configure(int gain, int drate);
    void reset();
    double read(byte channel);
    void readall(double *channels);
  private:
    bool init_adc();
  	void spi_transfer(byte *data, int len);
  	byte ReadChipID();
  	void SetChannel(byte channel);
  	double ReadData();
  	void WaitDRDY();
  	int cs;
  	int rst;
  	int pdwn;
  	int drdy;
};

#endif