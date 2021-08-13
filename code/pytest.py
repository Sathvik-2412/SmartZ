# Importing Libraries
import serial
import time
arduino = serial.Serial(port='/dev/cu.usbserial-14220', baudrate=115200, timeout=.1)
def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline().decode("utf-8").strip('\n')
    return data
while True:
    num = input("Enter a number: ") # Taking input from user
    value = write_read(num)
    print(value) # printing the value
