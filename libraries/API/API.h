#ifndef __API_H__
#define __API_H__

#include <ADS1256.h>
#include <DAC81408.h>
#include <Wire.h>

class API {
	public:
		API();
		bool init();
		void select_pin(int channel, int pin, bool force);
		void select_resistor(int channel, int resistor);
		void force_voltage(int channel, double voltage);
		double measure_voltage(int channel);
		void force_current(int channel, double current);
		double measure_current(int channel);
		void program_dut(byte addr, byte payload);
		void program_dut(byte addr, byte *payload, int num_bytes);
		byte read_dut(byte addr);
		void read_dut(byte addr, byte *results, int num_bytes);
	private:
		void force_voltage_servo(int channel, double voltage, double orig_voltage);
		void set_dmux(int channel, int pin);
		bool initSuccess;
		DAC81408 DAC;
		ADS1256 ADC1256;
		double Rsel_FSU0;
		double Rsel_FSU1;
		double Rvals[4] = {39000, 2000, 100, 5};
};

#endif /*__API_H__*/