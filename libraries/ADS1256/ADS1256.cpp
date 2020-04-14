#include "ADS1256.h"

// constants
#define ADS1256_CONVERSION 5.0 / 8388607.0 // VREF / (2^23 - 1)
#define ADS1256_GAIN_1 0 // gain channel
#define ADS1256_30000SPS 0xF0 // data rate
// register definitions
#define REG_STATUS 0 // x1H
#define REG_MUX 1 // 01H
// command definitions
#define CMD_WAKEUP 0x00 // Completes SYNC and Exits Standby Mode 0000 0000 (00h)
#define CMD_RDATA 0x01 // Read Data 0000 0001 (01h)
#define CMD_RREG 0x10 // Read from REG rrr 0001 rrrr (1xh)
#define CMD_WREG 0x50 // Write to REG rrr 0101 rrrr (5xh)
#define CMD_SYNC 0xFC // Synchronize the A/D Conversion 1111 1100 (FCh)

ADS1256::ADS1256() {
}

bool ADS1256::init(int cs, int rst, int pdwn, int drdy) {
  this->cs = cs;
  this->rst = rst;
  this->pdwn = pdwn;
  this->drdy = drdy;
  pinMode(cs, OUTPUT);
  pinMode(rst, OUTPUT);
  pinMode(pdwn, OUTPUT);
  pinMode(drdy, INPUT_PULLUP);
  // Output pins are active low, so initialize them to high
  digitalWrite(cs, HIGH);
  digitalWrite(rst, HIGH);
  digitalWrite(pdwn, HIGH);

  bool check_adc = init_adc();
  if (!check_adc) {
    Serial.println("Failed to initialize ADC");
  }
  return check_adc;
}

bool ADS1256::init_adc() {
  reset();
  if (ReadChipID() == 3) {
    Serial.println("ADC Chip ID Read success");
  } else {
    Serial.println("ADC Chip ID Read failed");
    return false;
  }
  configure(ADS1256_GAIN_1, ADS1256_30000SPS);
  return true;
}

void ADS1256::spi_transfer(byte *data, int len) {
  SPI.begin();
  SPI.beginTransaction(SPISettings(20000, MSBFIRST, SPI_MODE1));
  digitalWrite(cs, LOW);
  SPI.transfer(data, len);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  SPI.end();
}

void ADS1256::reset() {
  digitalWrite(rst, HIGH);
  delay(200);
  digitalWrite(rst, LOW);
  delay(200);
  digitalWrite(rst, HIGH);
}

byte ADS1256::ReadChipID() {
  WaitDRDY();
  byte data[4] = {CMD_RREG | REG_STATUS, 0x00, 0x00, 0x00};
  spi_transfer(data, 4);
  return data[3] >> 4;
}

// The configadsuration parameters of ADC, gain and data rate
void ADS1256::configure(int gain, int drate) {
  WaitDRDY();
  byte buf[10] = {CMD_WREG | 0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  buf[2] = 4;
  buf[3] = 0x08;
  buf[4] = (0 << 5) | (0 << 3) | (gain << 0);
  buf[5] = drate;
  
  spi_transfer(buf, 10);
  delay(1);
}

double ADS1256::read(byte channel) {
  SetChannel(channel);
  byte data[1] = {CMD_SYNC};
  spi_transfer(data, 1);
  data[0] = CMD_WAKEUP;
  spi_transfer(data, 1);
  return ReadData() * ADS1256_CONVERSION;
}

void ADS1256::readall(double *channels) {
  for (int i = 0; i < 8; i++) {
    channels[i] = read(i);
  }
}

void ADS1256::SetChannel(byte channel) {
  byte reg_data = (channel << 4) | (1 << 3);
  byte data[3] = {CMD_WREG | REG_MUX, 0x00, reg_data};
  spi_transfer(data, 3);
}

double ADS1256::ReadData() {
  WaitDRDY();
  byte data[5] = {CMD_RDATA, 0x00, 0x00, 0x00, 0x00};
  spi_transfer(data, 5);
  int val = (data[2] << 16) & 0xff0000;
  val |= (data[3] << 8) & 0xff00;
  val |= (data[4]) & 0xff;
  if (val & 0x800000) {
    val &= 0xF000000;
  }
  return val;
}

void ADS1256::WaitDRDY() {
  int i;
  for (i = 0; i < 400000; i++) {
    if (digitalRead(drdy) == LOW) {
      break;
    }
  }
  if (i >= 399999) {
    Serial.println("DRDY Time Out occurred ...");
  }
}