#include <API.h>

#define NUM_DECIMALS 6 // number of decimals to print when sending a double value

String command; // api commands are sent as a string terminated by \n, i.e. "force_v\n"
String in_voltage, in_current; // when reading a voltage/current to force, it comes in as a string since it is a decimal value
byte data[3] = {0x00, 0x00, 0x00}; // data for commands comes in as a number of bytes

API API;
bool initSuccess;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    if (command == "init") {
      initSuccess = API.init();
      if (initSuccess) {
        Serial.println("API initialization successful");
      } else {
        Serial.print("Error in initialization. ");
        while(!initSuccess) {
          Serial.println("Re-trying init...");
          initSuccess = API.init();
        }
      }
    } else if (command == "select_pin") {
      Serial.readBytes(data, 3);
      API.select_pin(data[0], data[1], data[2]);
    } else if (command == "select_r") {
      Serial.readBytes(data, 2);
      API.select_resistor(data[0], data[1]);
    } else if (command == "force_v") {
      Serial.readBytes(data, 1);
      in_voltage = Serial.readStringUntil('\n');
      API.force_voltage(data[0], in_voltage.toDouble());
    } else if (command == "measure_v") {
      Serial.readBytes(data, 1);
      double out_voltage = API.measure_voltage(data[0]);
      Serial.println(out_voltage, NUM_DECIMALS);
    } else if (command == "force_i") {
      Serial.readBytes(data, 1);
      in_current = Serial.readStringUntil('\n');
      API.force_current(data[0], in_current.toDouble());
    } else if (command == "measure_i") {
      Serial.readBytes(data, 1);
      double out_current = API.measure_current(data[0]);
      Serial.println(out_current, NUM_DECIMALS*2);
    } else if (command == "program_dut") {
      Serial.readBytes(data, 2);
      API.program_dut(data[0], data[1]);
    } else if (command == "program_dut12") {
      Serial.readBytes(data, 3);
      byte dut_data[2] = {data[1], data[2]};
      API.program_dut(data[0], dut_data, 2);
    } else if (command == "read_dut") {
      Serial.readBytes(data, 1);
      byte dut_read = API.read_dut(data[0]);
      Serial.println(dut_read, HEX);
    } else if (command == "read_dut12") {
      Serial.readBytes(data, 1);
      byte results[2] = {0x00, 0x00};
      API.read_dut(data[0], results, 2);
      Serial.println(results[0], HEX);
      Serial.println(results[1], HEX);
    }
  }
}
