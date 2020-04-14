#include <ADS1256.h>

#define CONVERSION 5 / 8388608

#define CS 10
#define DRDY 11
#define RST 12
#define PDWN 13

double voltages[8] = {0,0,0,0,0,0,0,0};
ADS1256 ADC1256(CS, RST, PDWN, DRDY);

void setup() {
  Serial.begin(9600);
  Serial.println("ADS1256 example");

  ADC1256.init();
}

void loop() {
  ADC1256.readall(voltages);

  for (int i = 0; i <= 8; i++) {
    Serial.print("Channel ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(voltages[i] * CONVERSION);
  }
}
