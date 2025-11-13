from machine import Pin
from utime import sleep
from sw.test_motor import Motor
import sw.test_input
# line sensor 1 and 2 are at back of car
# to make sure the car is going straight
speed_wheel_left = 50
speed_wheel_right = 50
LINE_1_PIN = 21
LINE_2_PIN = 11
line_1 = Pin(LINE_1_PIN, Pin.IN, Pin.PULL_DOWN)
line_2 = Pin(LINE_2_PIN, Pin.IN, Pin.PULL_DOWN)
def motor_turn_left(speed=30):
    motor_left.Reverse(speed)
    motor_right.Forward(speed)
    sleep(2.1) # for speed 30, make a 90 degree turn
    motor_left.off()
    motor_right.off()


def motor_turn_right(speed=30):
    motor_left.Forward(speed)
    motor_right.Reverse(speed) 
    sleep(2.1) # for speed 30, make a 90 degree turn
    motor_left.off()
    motor_right.off()

def motor_go_straight(speed_left,speed_right, direction="forward", duration=2):
    if direction == "forward":
        motor_left.Forward(speed_left)
        motor_right.Forward(speed_right)
    elif direction == "reverse":
        motor_left.Reverse(speed_left)
        motor_right.Reverse(speed_right)
    else:
        print("Invalid direction. Use 'forward' or 'reverse'.")
        return

    sleep(duration)
    motor_left.off()
    motor_right.off()

    

def init():
    LINE_1_PIN_POWER = 20
    LINE_2_PIN_POWER = 12
    # Power the line sensors
    pin_power_1 = Pin(LINE_1_PIN_POWER, Pin.OUT)
    pin_power_1.value(1)  # Power on line sensor 1
    pin_power_2 = Pin(LINE_2_PIN_POWER, Pin.OUT)
    pin_power_2.value(1)  # Power on line sensor 2
    line_1.irq(handler=line_follower,trigger=Pin.IRQ_FALLING)
    line_2.irq(handler=line_follower,trigger=Pin.IRQ_FALLING)
    global motor_left, motor_right
    motor_left = Motor(dirPin=4, PWMPin=5)   # Motor left is controlled from Motor Driv2 #2, which is on GP2/3
    motor_right = Motor(dirPin=7, PWMPin=6)  # Motor right is controlled from Motor Driv2 #3, which is on GP6/7
    
    
def main():
    motor_go_straight(speed_wheel_left,speed_wheel_right, direction="forward", duration=2)
    # motor_go_straight(speed=50, direction="forward", duration=2)
    motor_go_straight(speed_wheel_left,speed_wheel_right, direction="reverse", duration=2)
    
 



    # keyboard interrupt to stop motors
    
    
    return
def line_follower(p):
    "Interrupt handler"
    #The interrupte routine that interrupts the main code after and changes the speeds accordingly
    sensor_left = line_2.value()
    sensor_right = line_1.value()
    #To edit the global variables we need this command
    global speed_wheel_left, speed_wheel_right

    if (sensor_right == 1 and sensor_left ==0):
        #Add the speed to the left wheel to correct the motion, but by how much?
        speed_wheel_left += 5
    elif (sensor_right == 0 and sensor_left == 1):
        speed_wheel_right += 5
    #Adjust the speed so that it could correct its path
    motor_go_straight(speed_wheel_left,speed_wheel_right, direction="forward", duration=2)
    while True:
        sensor_left_new = line_1.value()
        sensor_right_new = line_2.value()
        if (sensor_left_new == 1 and sensor_left == 0):
            speed_wheel_left -= 5
            motor_go_straight(speed_wheel_left,speed_wheel_right, direction="forward", duration=2)
            return
        elif (sensor_right_new == 1 and sensor_right == 0):
            speed_wheel_right -= 5
            motor_go_straight(speed_wheel_left,speed_wheel_right, direction="forward", duration=2)
            return
if __name__ == "__main__":
    init()
    try:
        main()
    except KeyboardInterrupt:
        #Turn everything off
        motor_left.off()
        motor_right.off()


