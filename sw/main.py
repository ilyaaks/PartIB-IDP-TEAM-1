from machine import Pin
from utime import sleep
from sw.test_motor import Motor

LEFT_WHEEL_SPEED = 50
RIGHT_WHEEL_SPEED = 50
RIGHT_PIN_NUMBER = 21
LEFT_PIN_NUMBER = 11

BASE_SPEED = 40
SPEED_ADJUSTMENT = 10
MIN_SPEED = 20
MAX_SPEED = 60

correction_needed = False  # Flag for interrupt handler

signal_right = Pin(RIGHT_PIN_NUMBER, Pin.IN, Pin.PULL_DOWN)
signal_left = Pin(LEFT_PIN_NUMBER, Pin.IN, Pin.PULL_DOWN)

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


def motor_go_straight(speed_left, speed_right, direction="forward"):
    if direction == "forward":
        motor_left.Forward(speed_left)
        motor_right.Forward(speed_right)
    elif direction == "reverse":
        motor_left.Reverse(speed_left)
        motor_right.Reverse(speed_right)
    else:
        raise ValueError("Invalid direction: must be 'forward' or 'reverse'")
    


def line_follower(p):
    """Interrupt handler for line sensors"""
    global LEFT_WHEEL_SPEED, RIGHT_WHEEL_SPEED, correction_needed
    
    # Read current sensor values
    sensor_left = signal_left.value()
    sensor_right = signal_right.value()
    
    print("left: ", sensor_left, " right: ", sensor_right)
    
    # Adjust speeds based on sensor readings
    if sensor_right == 1 and sensor_left == 0:
        # Right sensor on line - car drifting right, turn left
        LEFT_WHEEL_SPEED = max(BASE_SPEED - SPEED_ADJUSTMENT, MIN_SPEED)
        RIGHT_WHEEL_SPEED = min(BASE_SPEED + SPEED_ADJUSTMENT, MAX_SPEED)
        correction_needed = True
        
    elif sensor_left == 1 and sensor_right == 0:
        # Left sensor on line - car drifting left, turn right
        LEFT_WHEEL_SPEED = min(BASE_SPEED + SPEED_ADJUSTMENT, MAX_SPEED)
        RIGHT_WHEEL_SPEED = max(BASE_SPEED - SPEED_ADJUSTMENT, MIN_SPEED)
        correction_needed = True
        
    elif sensor_left == 1 and sensor_right == 1:
        # Both on line - go straight
        LEFT_WHEEL_SPEED = BASE_SPEED
        RIGHT_WHEEL_SPEED = BASE_SPEED
        correction_needed = True
        
    else:
        # Both off line - maintain current speeds
        pass

    

def init():
    # Set up interrupt handlers for line detection
    signal_right.irq(handler=line_follower, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
    signal_left.irq(handler=line_follower, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
    global motor_left, motor_right
    motor_left = Motor(dirPin=4, PWMPin=5)   # Motor left is controlled from Motor Driv2 #2, which is on GP2/3
    motor_right = Motor(dirPin=7, PWMPin=6)  # Motor right is controlled from Motor Driv2 #3, which is on GP6/7
    
    
def main():
    global LEFT_WHEEL_SPEED, RIGHT_WHEEL_SPEED, correction_needed
    
    print("Starting line follower")
    
    # Start moving forward
    motor_go_straight(LEFT_WHEEL_SPEED, RIGHT_WHEEL_SPEED, direction="forward")
    
    while True:
        # Check if interrupt handler set correction flag
        if correction_needed:
            # Apply the new speeds calculated by interrupt handler
            motor_go_straight(LEFT_WHEEL_SPEED, RIGHT_WHEEL_SPEED, direction="forward")
            correction_needed = False
        
        # Small delay to allow system to respond
        sleep(0.01)


if __name__ == "__main__":
    init()

    try:
        main()
    except KeyboardInterrupt:
        #Turn everything off
        motor_left.off()
        motor_right.off()


