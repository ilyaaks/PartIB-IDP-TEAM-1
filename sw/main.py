from machine import Pin
from utime import sleep
from sw.test_motor import Motor

BASE_SPEED = 70
SPEED_ADJUSTMENT = 10
MIN_SPEED = 20
MAX_SPEED = 100

left_wheel_speed = BASE_SPEED
right_wheel_speed = BASE_SPEED
MID_RIGHT_PIN_NUMBER = 21
MID_LEFT_PIN_NUMBER = 20
FAR_RIGHT_PIN_NUMBER = 22
FAR_LEFT_PIN_NUMBER = 19

correction_needed = False  # Flag for interrupt handler

signal_mid_right = Pin(MID_RIGHT_PIN_NUMBER, Pin.IN, Pin.PULL_DOWN)
signal_mid_left = Pin(MID_LEFT_PIN_NUMBER, Pin.IN, Pin.PULL_DOWN)
signal_far_right = Pin(FAR_RIGHT_PIN_NUMBER, Pin.IN, Pin.PULL_DOWN)
signal_far_left = Pin(FAR_LEFT_PIN_NUMBER, Pin.IN, Pin.PULL_DOWN)
sensor_left_prev = 0
sensor_right_prev = 0
far_right_prev = 0
far_left_prev = 0
count_lines = 0

'''
case diagram:
    4       3
    +------+
    |      |
    |      |
    |      |
    +------+
    5  6,1  2
'''
turning_case = 0

def motor_turn_right():
    global signal_far_left,left_wheel_speed,right_wheel_speed,correction_needed
    left_wheel_speed = MIN_SPEED
    right_wheel_speed = MAX_SPEED


def motor_turn_left():
    global signal_far_left,left_wheel_speed,right_wheel_speed,correction_needed
    left_wheel_speed = MAX_SPEED
    right_wheel_speed = MIN_SPEED
    

def motor_go_straight(speed_left, speed_right, direction="forward"):
    if direction == "forward":
        motor_left.Forward(speed_left)
        motor_right.Forward(speed_right)
    elif direction == "reverse":
        motor_left.Reverse(speed_left)
        motor_right.Reverse(speed_right)


def motors_off():
    """Turn off both motors"""
    motor_left.off()
    motor_right.off()
    

def line_follower(p):
    """Interrupt handler for line sensors"""
    global left_wheel_speed, right_wheel_speed, correction_needed, sensor_left_prev, sensor_right_prev, count_lines,far_left_prev,far_right_prev
    # Read current sensor values
    sensor_mid_left = signal_mid_left.value()
    sensor_mid_right = signal_mid_right.value()
    sensor_far_left = signal_far_left.value()
    sensor_far_right = signal_far_right.value()
    # print("left: ", sensor_mid_left, " right: ", sensor_mid_right, " far left: ", sensor_far_left, " far right: ", sensor_far_right, " left speed: ", left_wheel_speed, " right speed: ", right_wheel_speed)
    if ((sensor_far_left == 1 or sensor_far_right == 1) and (far_right_prev == 0 and far_left_prev == 0)):  
        count_lines += 1
        far_right_prev = 1
        far_left_prev = 1
        print("Lines detected: ", count_lines)
    elif ((far_right_prev == 1 and far_left_prev == 1) and (sensor_far_left == 0 and sensor_far_right == 0)):
        far_right_prev = 0
        far_left_prev = 0
    # Both far sensors off line, stop or reverse
    # Adjust speeds based on sensor readings
    if sensor_mid_right == 1 and sensor_mid_left == 0:
        # Right sensor on line - car drifting right, turn left
        left_wheel_speed = max(BASE_SPEED - SPEED_ADJUSTMENT, MIN_SPEED)
        right_wheel_speed = min(BASE_SPEED + SPEED_ADJUSTMENT, MAX_SPEED)
        correction_needed = True
        
    elif sensor_mid_left == 1 and sensor_mid_right == 0:
        # Left sensor on line - car drifting left, turn right
        left_wheel_speed = min(BASE_SPEED + SPEED_ADJUSTMENT, MAX_SPEED)
        right_wheel_speed = max(BASE_SPEED - SPEED_ADJUSTMENT, MIN_SPEED)
        correction_needed = True
        
    elif sensor_mid_left == 1 and sensor_mid_right == 1:
        # Both on line - go straight
        left_wheel_speed = BASE_SPEED
        right_wheel_speed = BASE_SPEED
        correction_needed = True
        
    else:
        # Both sensors off line - use previous state to determine recovery direction
        if (sensor_left_prev == 1 and sensor_right_prev == 0):
            # Last seen left on line - turn left to find line
            motor_turn_left()
            correction_needed = True
        elif (sensor_right_prev == 1 and sensor_left_prev == 0):
            # Last seen right on line - turn right to find line
            motor_turn_right()
            correction_needed = True
        # If both were off previously, no correction needed - continue current trajectory
        
    sensor_left_prev = sensor_mid_left
    sensor_right_prev = sensor_mid_right

def init():
    # Set up interrupt handlers for line detection
    signal_mid_right.irq(handler=line_follower, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
    signal_mid_left.irq(handler=line_follower, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
    signal_far_left.irq(handler=line_follower, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
    signal_far_right.irq(handler=line_follower, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
    global motor_left, motor_right
    motor_left = Motor(dirPin=4, PWMPin=5)   # Motor left is controlled from Motor Driv2 #2, which is on GP2/3
    motor_right = Motor(dirPin=7, PWMPin=6)  # Motor right is controlled from Motor Driv2 #3, which is on GP6/7
    
    
def main():
    global left_wheel_speed, right_wheel_speed, correction_needed, count_lines, motor_turn_right, motor_turn_left, turning_case    
    print("Starting line follower")
    
    # Start moving forward
    motor_go_straight(left_wheel_speed, right_wheel_speed, direction="forward")
    
    while True:
        # Check if interrupt handler set correction flag
        if (count_lines == 2 and turning_case == 0):
            motor_turn_right()
            motor_go_straight(left_wheel_speed, right_wheel_speed, direction="forward")
            sleep(1)
            turning_case = 1
            count_lines = 0 
            print("turning_case 0 turning")
        elif (count_lines == 2 and turning_case == 1):
            print("Turning left")
            motor_turn_left()
            motor_go_straight(left_wheel_speed, right_wheel_speed, direction="forward")
            sleep(1)
            turning_case = 2
            count_lines = 0
            print("turning_case 1 turning")
        elif (count_lines == 8 and turning_case == 2):
            motor_turn_left()
            motor_go_straight(left_wheel_speed, right_wheel_speed, direction="forward")
            sleep(1)
            turning_case = 3
            count_lines = 0
            print("turning_case 2 turning")
            print("")
        elif (count_lines == 2 and turning_case == 3):
            motor_turn_left()
            motor_go_straight(left_wheel_speed, right_wheel_speed, direction="forward")
            sleep(1)
            turning_case = 4
            count_lines = 0
            print("turning_case 3 turning")
        elif (count_lines == 8 and turning_case == 4):
            motor_turn_left()
            motor_go_straight(left_wheel_speed, right_wheel_speed, direction="forward")
            sleep(2)
            turning_case = 5
            count_lines = 0
            print("turning_case 4 turning")
        elif (count_lines == 2 and turning_case == 5):
            motor_turn_right()
            motor_go_straight(left_wheel_speed, right_wheel_speed, direction="forward")
            sleep(1.5)
            motor_go_straight(70, 70, direction="forward")
            sleep (1.5)
            motors_off()
            break
        if correction_needed:
            # Apply the new speeds calculated by interrupt handler
            motor_go_straight(left_wheel_speed, right_wheel_speed, direction="forward")
            correction_needed = False
            
        # Small delay to allow system to respond
        sleep(0.01)


if __name__ == "__main__":
    init()
    try:
        main()
    except KeyboardInterrupt:
        #Turn everything off
        motors_off()


