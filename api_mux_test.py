import API

# arduino connected will be on port ttyACMx
# can use console command 'ls /dev/ttyACM*'
# to find which port the Arduino is currently on
device = API.Arduino('/dev/ttyACM0',9600)

print("MUX TEST")
pin=input("Please enter DUT pin you would like to set:\n  >> ")
voltage=input("Please enter voltage you would like to set on pin "+pin+":\n  >> ")
# skip 13, 14, 16, 17, 19, 26, 27, 28, 33, 42, 45
device.select_pin(0, int(pin))
device.force_voltage(0, float(voltage))