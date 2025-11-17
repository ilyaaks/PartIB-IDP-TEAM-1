from machine import Pin
from utime import sleep
from sw.test_motor import Motor


class LineFollowerRobot:
    """
    Line-following robot with integrated sensor processing and motor control.
    Encapsulates all state and behavior for autonomous line following.
    """
    
    # Configuration constants
    BASE_SPEED = 70
    SPEED_ADJUSTMENT = 10
    MIN_SPEED = 20
    MAX_SPEED = 100
    
    # Pin configurations
    MID_RIGHT_PIN = 21
    MID_LEFT_PIN = 20
    FAR_RIGHT_PIN = 22
    FAR_LEFT_PIN = 19
    
    def __init__(self):
        """Initialize robot hardware and state"""
        # Initialize hardware
        self._init_sensors()
        self._init_motors()
        
        # Initialize state variables
        self._init_state()
    
    def _init_sensors(self):
        """Initialize all sensor pins"""
        self.signal_mid_right = Pin(self.MID_RIGHT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_mid_left = Pin(self.MID_LEFT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_far_right = Pin(self.FAR_RIGHT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_far_left = Pin(self.FAR_LEFT_PIN, Pin.IN, Pin.PULL_DOWN)
    
    def _init_motors(self):
        """Initialize motor controllers"""
        self.motor_left = Motor(dirPin=4, PWMPin=5)   # Motor left is controlled from Motor Driv2 #2
        self.motor_right = Motor(dirPin=7, PWMPin=6)  # Motor right is controlled from Motor Driv2 #3
    
    def _init_state(self):
        """Initialize all state variables"""
        # Motor speeds
        self.left_wheel_speed = self.BASE_SPEED
        self.right_wheel_speed = self.BASE_SPEED
        self.correction_needed = False
        
        # Sensor state tracking
        self.sensor_left_prev = 0
        self.sensor_right_prev = 0
        self.far_right_prev = 0
        self.far_left_prev = 0
        
        # Navigation state
        self.count_lines = 0
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
        self.turning_case = 0
    
    def motor_turn_right(self):
        """Configure motor speeds for right turn"""
        self.left_wheel_speed = self.MIN_SPEED
        self.right_wheel_speed = self.MAX_SPEED
    
    def motor_turn_left(self):
        """Configure motor speeds for left turn"""
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed = self.MIN_SPEED
    
    def motor_go_straight(self, speed_left, speed_right, direction="forward"):
        """
        Execute straight movement in specified direction
        
        Args:
            speed_left: Speed for left motor (0-100)
            speed_right: Speed for right motor (0-100)
            direction: Either 'forward' or 'reverse'
        """
        if direction == "forward":
            self.motor_left.Forward(speed_left)
            self.motor_right.Forward(speed_right)
        elif direction == "reverse":
            self.motor_left.Reverse(speed_left)
            self.motor_right.Reverse(speed_right)
    
    def motors_off(self):
        """Turn off both motors"""
        self.motor_left.off()
        self.motor_right.off()
    
    def setup_interrupts(self):
        """Set up interrupt handlers for all sensors"""
        # Create a closure that captures self for the interrupt handler
        def handler(p):
            self.line_follower(p)
        
        # Attach interrupt handlers to all sensor pins
        self.signal_mid_right.irq(handler=handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self.signal_mid_left.irq(handler=handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self.signal_far_left.irq(handler=handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self.signal_far_right.irq(handler=handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
    
    def disable_interrupts(self):
        """Disable interrupt handlers for all sensors"""
        def handler(p):
            self.line_follower(p)
        
        self.signal_mid_right.irq(handler=handler, trigger=0)
        self.signal_mid_left.irq(handler=handler, trigger=0)
        self.signal_far_left.irq(handler=handler, trigger=0)
        self.signal_far_right.irq(handler=handler, trigger=0)

    def line_follower(self, p):
        """
        Interrupt handler for line sensors.
        Adjusts motor speeds based on sensor readings to keep robot on line.
        
        Args:
            p: Pin that triggered the interrupt
        """
        # Read current sensor values
        sensor_mid_left = self.signal_mid_left.value()
        sensor_mid_right = self.signal_mid_right.value()
        sensor_far_left = self.signal_far_left.value()
        sensor_far_right = self.signal_far_right.value()
        
        # Line counting logic - detect crossing a perpendicular line
        if ((sensor_far_left == 1 or sensor_far_right == 1) and 
            (self.far_right_prev == 0 and self.far_left_prev == 0)):
            self.count_lines += 1
            self.far_right_prev = 1
            self.far_left_prev = 1
            print("Lines detected:", self.count_lines)
        elif ((self.far_right_prev == 1 and self.far_left_prev == 1) and 
              (sensor_far_left == 0 and sensor_far_right == 0)):
            self.far_right_prev = 0
            self.far_left_prev = 0
        
        # Adjust speeds based on sensor readings
        if sensor_mid_right == 1 and sensor_mid_left == 0:
            # Right sensor on line - car drifting right, turn left
            self.left_wheel_speed = max(self.BASE_SPEED - self.SPEED_ADJUSTMENT, self.MIN_SPEED)
            self.right_wheel_speed = min(self.BASE_SPEED + self.SPEED_ADJUSTMENT, self.MAX_SPEED)
            self.correction_needed = True
            
        elif sensor_mid_left == 1 and sensor_mid_right == 0:
            # Left sensor on line - car drifting left, turn right
            self.left_wheel_speed = min(self.BASE_SPEED + self.SPEED_ADJUSTMENT, self.MAX_SPEED)
            self.right_wheel_speed = max(self.BASE_SPEED - self.SPEED_ADJUSTMENT, self.MIN_SPEED)
            self.correction_needed = True
            
        elif sensor_mid_left == 1 and sensor_mid_right == 1:
            # Both on line - go straight
            self.left_wheel_speed = self.BASE_SPEED
            self.right_wheel_speed = self.BASE_SPEED
            self.correction_needed = True
            
        else:
            # Both sensors off line - use previous state to determine recovery direction
            if self.sensor_left_prev == 1 and self.sensor_right_prev == 0:
                # Last seen left on line - turn left to find line
                self.motor_turn_left()
                self.correction_needed = True
            elif self.sensor_right_prev == 1 and self.sensor_left_prev == 0:
                # Last seen right on line - turn right to find line
                self.motor_turn_right()
                self.correction_needed = True
            # If both were off previously, no correction needed - continue current trajectory
        
        # Update previous sensor states
        self.sensor_left_prev = sensor_mid_left
        self.sensor_right_prev = sensor_mid_right
    
    def _execute_turn(self, turn_function, sleep_time, next_case, case_name):
        """
        Helper method to execute a turn and update state
        
        Args:
            turn_function: Method to call for turning (motor_turn_left or motor_turn_right)
            sleep_time: Time to sleep after initiating turn
            next_case: Next turning_case value
            case_name: Name of current case for logging
        """
        turn_function()
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, direction="forward")
        sleep(sleep_time)
        self.turning_case = next_case
        self.count_lines = 0
        print(f"turning_case {case_name} turning")
    
    def _handle_turning_cases(self):
        """
        Process turning logic based on current turning case.
        Returns True if final case is complete (should exit), False otherwise.
        """
        if self.count_lines == 2 and self.turning_case == 0:
            self._execute_turn(self.motor_turn_right, 1, 1, "0")
            
        elif self.count_lines == 2 and self.turning_case == 1:
            print("Turning left")
            self._execute_turn(self.motor_turn_left, 1, 2, "1")
            
        elif self.count_lines == 8 and self.turning_case == 2:
            self._execute_turn(self.motor_turn_left, 1, 3, "2")
            print("")
            
        elif self.count_lines == 2 and self.turning_case == 3:
            self._execute_turn(self.motor_turn_left, 1, 4, "3")
            
        elif self.count_lines == 8 and self.turning_case == 4:
            self._execute_turn(self.motor_turn_left, 2, 5, "4")
            
        elif self.count_lines == 2 and self.turning_case == 5:
            # Final approach sequence
            self.motor_turn_right()
            self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, direction="forward")
            sleep(1.5)
            self.motor_go_straight(70, 70, direction="forward")
            sleep(1.5)
            self.motors_off()
            return True  # Signal to exit main loop
        
        return False  # Continue running
    
    def run(self):
        """
        Main control loop for the line follower.
        Continuously monitors sensors and adjusts motors until sequence is complete.
        """
        print("Starting line follower")
        
        # Start moving forward
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, direction="forward")
        
        while True:
            # Handle turning cases - exit if final case is complete
            if self._handle_turning_cases():
                break
            
            # Apply corrections from interrupt handler if needed
            if self.correction_needed:
                self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, direction="forward")
                self.correction_needed = False
            
            # Small delay to allow system to respond
            sleep(0.01)


if __name__ == "__main__":
    # Create robot instance
    robot = LineFollowerRobot()
    
    # Set up interrupt handlers
    robot.setup_interrupts()
    
    try:
        # Run the main control loop
        robot.run()
    except KeyboardInterrupt:
        # Clean shutdown on Ctrl+C
        print("\nStopping...")
        robot.motors_off()
