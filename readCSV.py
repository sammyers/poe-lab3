from serial import Serial
import csv

SERIAL_PORT = '/dev/cu.usbmodem1411' # This should probably be a command line argument but YOLO
BAUD_RATE = 115200 # This too

def read_values(arduino):
    with open('pid.csv', 'w+') as csvfile:
        while True:
            line = arduino.readline()
            if line:
                values = line.decode('utf-8')
                print(values) # Just so we know that it's working (and when it's finished)
                # The values come as a newline-terminated, comma-separated string of values,
                # so they can be written directly to CSV format
                csvfile.write(values)


if __name__ == '__main__':
    arduino = Serial(SERIAL_PORT, BAUD_RATE, timeout=.1)
    read_values(arduino)
