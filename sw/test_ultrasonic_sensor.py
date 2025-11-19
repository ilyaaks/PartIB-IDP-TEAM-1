from machine import Pin, ADC
# from sw.libs.URM09_distance_sensor_python.DFRobot_URM09 import *
from utime import sleep
#The module will output analog voltage proportional to distance. After ADC sampled and output these data, the distance value can be obtained with a simple processing step.
#Importing the library for the ultrasonice sensor
#The sensor adopts the I2C communication 
def test(): 
    Max_rang = 520
    ADC_Solution = 1023
    input_pin = ADC(Pin(26))  
    while True:
        sensity_t = input_pin.read_u16()
        print("The ADC value is : ", sensity_t)
        sleep(0.5)

if  __name__ == "__main__":
    test()