from machine import Pin
from utime import sleep
from sw.test_motor import Motor

# line sensor 1 and 2 are at back of car
# to make sure the car is going straight
LINE_1_PIN = 0
LINE_2_PIN = 0
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

def motor_go_straight(speed=50, direction="forward", duration=2):
    if direction == "forward":
        motor_left.Forward(speed)
        motor_right.Forward(speed)
    elif direction == "reverse":
        motor_left.Reverse(speed)
        motor_right.Reverse(speed)
    else:
        print("Invalid direction. Use 'forward' or 'reverse'.")
        return

    sleep(duration)
    motor_left.off()
    motor_right.off()

    

def init():
    global motor_left, motor_right
    motor_left = Motor(dirPin=4, PWMPin=5)   # Motor left is controlled from Motor Driv2 #2, which is on GP2/3
    motor_right = Motor(dirPin=7, PWMPin=6)  # Motor right is controlled from Motor Driv2 #3, which is on GP6/7
    

def main():
    init()

    motor_go_straight(speed=50, direction="forward", duration=2)
    motor_turn_left()
    # motor_go_straight(speed=50, direction="forward", duration=2)
    motor_go_straight(speed=50, direction="reverse", duration=2)
    




    # keyboard interrupt to stop motors
    
    
    return


if __name__ == "__main__":
    main()


