#include "API.h"

#include <cmath>
#include <algorithm>

/****** I2C ADDRESS OF DUT ******/
#define I2C_ADDR 0x4c
/****** DAC SPI CHIP SELECT ******/
#define DAC_CS 4
/****** ADC SPI CHIP SELECT/GPIO PINS ******/
#define ADC_CS 10
#define DRDY 11
#define RST 12
#define PDWN 13
/****** 48-to-1 FORCE MUX, FSU CHANNEL 0 ******/
#define FRS_DMUX_23_FSU0 22
#define FRS_DMUX_22_FSU0 24
#define FRS_DMUX_21_FSU0 26
#define FRS_DMUX_20_FSU0 28
#define FRS_SEL0_FSU0 30
#define FRS_SEL1_FSU0 32
/****** 48-to-1 FORCE MUX, FSU CHANNEL 0 ******/
#define FRS_DMUX_23_FSU1 23
#define FRS_DMUX_22_FSU1 25
#define FRS_DMUX_21_FSU1 27
#define FRS_DMUX_20_FSU1 29
#define FRS_SEL0_FSU1 31
#define FRS_SEL1_FSU1 33
/****** SENSE RESISTOR MUX, FSU CHANNEL 0 ******/
#define ENA_FSU0 14
#define ENB_FSU0 15
#define SEL1_FSU0 16
#define SEL0_FSU0 17
/****** SENSE RESISTOR MUX, FSU CHANNEL 1 ******/
#define ENA_FSU1 5
#define ENB_FSU1 6
#define SEL1_FSU1 7
#define SEL0_FSU1 8

/**
 * This function determines if values a and b are approximately equal, within an
 * error bound set by epsilon. Useful for confirmation of DAC/ADC values
 */
bool approximatelyEqual(double a, double b, double epsilon) {
    return (std::abs(a - b) <= (std::max(std::abs(a), std::abs(b)) * epsilon));
}

/**
 * Initialize success bool and selected resistor to 5ohms on each FSU
 */
API::API() {
	initSuccess = false;
	Rsel_FSU0 = Rvals[3];
	Rsel_FSU1 = Rvals[3];
}

bool API::init() {
	Wire.begin();

	// setup GPIO pins
	// These loops assume pins are assigned together and not spread out
	// 48-to-1 FORCE MUX, FSU CHANNEL 0
	for (int i = FRS_DMUX_23_FSU0; i <= FRS_SEL1_FSU0; i++) {
		pinMode(i, OUTPUT);
		digitalWrite(i, LOW);
	}
	// 48-to-1 FORCE MUX, FSU CHANNEL 1
	for (int i = FRS_DMUX_23_FSU1; i <= FRS_SEL1_FSU1; i++) {
		pinMode(i, OUTPUT);
		digitalWrite(i, LOW);
	}
	// SENSE RESISTOR MUX, FSU CHANNEL 0
	// Set all pins as "high" to enable MUX with 5ohm resistor
	for (int i = ENA_FSU0; i <= SEL0_FSU0; i++) {
		pinMode(i, OUTPUT);
		digitalWrite(i, HIGH);
	}
	// SENSE RESISTOR MUX, FSU CHANNEL 1
	// Set all pins as "high" to enable MUX with 5ohm resistor
	for (int i = ENA_FSU1; i <= SEL0_FSU1; i++) {
		pinMode(i, OUTPUT);
		digitalWrite(i, HIGH);
	}

	// unkill
	byte unkill_data[2] = {0x8e, 0xa3};
	Wire.beginTransmission(I2C_ADDR);
	Wire.write(0xe8);
	Wire.write(unkill_data, 2);
	Wire.endTransmission();

	// zombie
	Wire.beginTransmission(I2C_ADDR);
	Wire.write(0xeb);
	Wire.write(0x01);
	Wire.endTransmission();

	DAC.init(DAC_CS);
	initSuccess = ADC1256.init(ADC_CS, RST, PDWN, DRDY);
	return initSuccess;
}

/**
 * Sets de-mux for MUX1-A through MUX6-B
 */
void API::set_dmux(int channel, int pin) {
	if (channel == 0) {
		digitalWrite(FRS_DMUX_20_FSU0, pin>>3 & 0x1);
		digitalWrite(FRS_DMUX_21_FSU0, pin>>2 & 0x1);
		digitalWrite(FRS_DMUX_22_FSU0, pin>>1 & 0x1);
		digitalWrite(FRS_DMUX_23_FSU0, pin>>0 & 0x1);
	} else if (channel == 1) {
		digitalWrite(FRS_DMUX_20_FSU1, pin>>3 & 0x1);
		digitalWrite(FRS_DMUX_21_FSU1, pin>>2 & 0x1);
		digitalWrite(FRS_DMUX_22_FSU1, pin>>1 & 0x1);
		digitalWrite(FRS_DMUX_23_FSU1, pin>>0 & 0x1);
	}
}

void API::select_pin(int channel, int pin, bool force) {
	int sel_val = (pin - 1) % 4; // subtract 1 because pin starts from 1 instead of 0
	int select1 = sel_val>>1 & 0x1; // high sel bit
	int select0 = sel_val>>0 & 0x1; // low sel bit

	/* FIXME: once sense mux board is made, must add sense selection as well */

	// Reset all mux pins to low
	for (int i = FRS_DMUX_23_FSU0; i <= FRS_SEL0_FSU0; i++) {
		digitalWrite(i, LOW);
	}
	for (int i = FRS_DMUX_23_FSU1; i <= FRS_SEL0_FSU1; i++) {
		digitalWrite(i, LOW);
	}
	if (force) {
		int mux_ena_pin = (pin - 1) / 4; // subtract 1 because pin starts from 1 instead of 0
		set_dmux(channel, mux_ena_pin);

		int FRS_SEL1 = (channel == 0 ? FRS_SEL1_FSU0 : FRS_SEL1_FSU1);
		int FRS_SEL0 = (channel == 0 ? FRS_SEL0_FSU0 : FRS_SEL0_FSU1);
		digitalWrite(FRS_SEL1, select1);
		digitalWrite(FRS_SEL0, select0);
	}

	delay(100);
}

void API::select_resistor(int channel, int resistor) {
	if (channel == 0) {
		Rsel_FSU0 = Rvals[resistor];
	} else if (channel == 1) {
		Rsel_FSU1 = Rvals[resistor];
	}

	int select1 = resistor>>1 & 0x1;
	int select0 = resistor>>0 & 0x1;

	int ENA = (channel == 0 ? ENA_FSU0 : ENA_FSU1);
	int ENB = (channel == 0 ? ENB_FSU0 : ENB_FSU1);
	int SEL1 = (channel == 0 ? SEL1_FSU0 : SEL1_FSU1);
	int SEL0 = (channel == 0 ? SEL0_FSU0 : SEL0_FSU1);

	digitalWrite(ENA, HIGH);
	digitalWrite(ENB, HIGH);
	digitalWrite(SEL1, select1);
	digitalWrite(SEL0, select0);

	delay(100);
}

/**
 * DAC0 forses on FSU channel 0, DAC1 forces on FSU channel 1
 */
void API::force_voltage(int channel, double voltage) {
	DAC.set(channel, voltage);

	double forced_voltage = measure_voltage(channel);
	if(!approximatelyEqual(voltage, forced_voltage, 0.0001)) {
		if (forced_voltage > voltage) {
			force_voltage_servo(channel, voltage-0.0001, voltage);
		} else if (forced_voltage < voltage) {
			force_voltage_servo(channel, voltage+0.0001, voltage);
		}
	}
}

void API::force_voltage_servo(int channel, double voltage, double orig_voltage) {
	DAC.set(channel, voltage);

	double forced_voltage = measure_voltage(channel);
	if(!approximatelyEqual(orig_voltage, forced_voltage, 0.0001)) {
		if (forced_voltage > orig_voltage) {
			force_voltage_servo(channel, voltage-0.0001, orig_voltage);
		} else if (forced_voltage < orig_voltage) {
			force_voltage_servo(channel, voltage+0.0001, orig_voltage);
		}
	}
}

/**
 * ADC0 senses on FSU channel 0, ADC1 senses on FSU channel 1
 */
double API::measure_voltage(int channel) {
	return ADC1256.read(channel);
}

/**
 * DAC0 controls Vin of FSU0, DAC1 controls Vin of FSU1
 */
void API::force_current(int channel, double current) {
	/* FIXME: Add range selection */
	
	double Rsel = (channel == 0 ? Rsel_FSU0 : Rsel_FSU1);
	double voltage = current * Rsel;
	// force_current calls force_voltage in order to servo the voltage to desired value
	// this is because servoing the current proved to be too finicky in the low range that we are working in
	force_voltage(channel, voltage);
}

/**
 * Base formula for current is (Isns - Vout) / Rsel
 * FSU channel 0 uses ADC2 on Isns and ADC0 on Vout. FSU channel 1 uses ADC3 on Isns and ADC1 on Vout
 */
double API::measure_current(int channel) {
	double Rsel = (channel == 0 ? Rsel_FSU0 : Rsel_FSU1);
	return (ADC1256.read(channel + 2) - ADC1256.read(channel)) / Rsel;
}

void API::program_dut(byte addr, byte payload) {
	Wire.beginTransmission(I2C_ADDR);
  	Wire.write(0xd1);
  	Wire.write(addr);
  	Wire.write(payload);
  	Wire.endTransmission();
}

void API::program_dut(byte addr, byte *payload, int num_bytes) {
	Wire.beginTransmission(I2C_ADDR);
  	Wire.write(0xd1);
  	Wire.write(addr);
  	Wire.write(payload, num_bytes);
  	Wire.endTransmission();
}

byte API::read_dut(byte addr) {
	Wire.beginTransmission(I2C_ADDR);
	Wire.write(0xd5);
	Wire.write(addr);
	Wire.endTransmission();

	byte result = 0x00;
	Wire.requestFrom(I2C_ADDR, 1);
	if (Wire.available()) {
    	result = Wire.read();
  	}
  	return result;
}

void API::read_dut(byte addr, byte *results, int num_bytes) {
	Wire.beginTransmission(I2C_ADDR);
	Wire.write(0xe5);
	Wire.write(addr);
	Wire.endTransmission();

	int idx = 0;
	Wire.requestFrom(I2C_ADDR, num_bytes);
	while (Wire.available()) {
    	results[idx] = Wire.read();
    	idx++;
  	}
}