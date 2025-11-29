from machine import Pin, PWM
from utime import sleep

class Actuator:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(Pin(PWMPin))  # set motor pwm pin
        self.pwm.freq(1000)  # set PWM frequency
        self.pwm.duty_u16(0)  # set duty cycle - 0=off
           
    def set(self, dir, speed):
        if dir == "extend":
            self.mDir.value(0)                     # forward = 0 reverse = 1 motor
        elif dir == "retract":
            self.mDir.value(1)       
        self.pwm.duty_u16(int(65535 * speed / 100))  # speed range 0-100 motor
    def off(self):
        self.pwm.duty_u16(0)

def test_actuator1():
    actuator1 = Actuator(dirPin=0, PWMPin=1)  # Actuator 1 controlled from Motor Driv1 #1, which is on GP0/1
    print("Testing Actuator 1")
    actuator1.set(dir ="retract", speed = 90)  # extend at 50% speed
    sleep(1.5)  # Give actuator time to retract (adjust time as needed)
    actuator1.off()
    print("Actuator test complete")
    
if __name__ == "__main__":
    actuator1 = Actuator(dirPin=0, PWMPin=1)
    try:
        test_actuator1()
    except KeyboardInterrupt:
        actuator1.off()
        print("Test interrupted by user")