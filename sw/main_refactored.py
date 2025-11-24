from machine import Pin, I2C
from utime import sleep, ticks_ms, ticks_diff
from sw.test_motor import Motor
from sw.libs.VL53L0X.VL53L0X import VL53L0X
# from sw.libs.DFRobot_URM09.DFRobot_URM09 import DFRobot_URM09
# from sw.libs.tcs3472_micropython.tcs3472 import tcs3472


class LineFollowerRobot:
    """
    Line-following robot with integrated sensor processing and motor control.
    Encapsulates all state and behavior for autonomous line following.
    """
    
    # Configuration constants
    BASE_SPEED = 50
    SPEED_ADJUSTMENT = 5
    MIN_SPEED = 35
    MAX_SPEED = 65
    
    # Proportional control parameters
    KP = 8  # Proportional gain for steering correction

    # Pin configurations
    MID_RIGHT_PIN = 26
    MID_LEFT_PIN = 27
    FAR_RIGHT_PIN = 28
    FAR_LEFT_PIN = 22

    SDA_PIN = 20
    SCL_PIN = 21

    # BUTTON_PIN = 9999
    # DEBOUNCE_MS = 200
    
    def __init__(self):
        """Initialize robot hardware and state"""
        # Initialize hardware
        self._init_sensors()
        self._init_motors()
        
        # Initialize state variables
        self._init_state()
        
        # Start distance sensor
        self.vl53l0.start()
    
    def _init_sensors(self):
        """Initialize all sensor pins"""
        self.signal_mid_right = Pin(self.MID_RIGHT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_mid_left = Pin(self.MID_LEFT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_far_right = Pin(self.FAR_RIGHT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_far_left = Pin(self.FAR_LEFT_PIN, Pin.IN, Pin.PULL_DOWN)
        
        # config I2C Bus
        i2c_bus_vl5310 = I2C(0, sda=Pin(self.SDA_PIN), scl=Pin(self.SCL_PIN))  # I2C0 on GP8 & GP9
        
        # Setup vl53l0 object
        self.vl53l0 = VL53L0X(i2c_bus_vl5310)
        self.vl53l0.set_Vcsel_pulse_period(self.vl53l0.vcsel_period_type[0], 18)
        self.vl53l0.set_Vcsel_pulse_period(self.vl53l0.vcsel_period_type[1], 14)

        # Setup digital button
        # self.button = Pin(self.BUTTON_PIN, Pin.IN, Pin.PULL_UP)  # Not connected yet

        # Setup colour sensor (not connected yet)
        # i2c_bus_tcs = I2C(id=0, sda=Pin(8), scl=Pin(9), freq=400000)  # I2C0 on GP8 & GP9
        # self.tcs = tcs3472(i2c_bus)  # TCS3472 is always at address 41 (0x29)

    
    def _init_motors(self):
        """Initialize motor controllers"""
        self.motor_left = Motor(dirPin=7, PWMPin=6)   # Motor left is controlled from Motor Driv2 #2
        self.motor_right = Motor(dirPin=4, PWMPin=5)  # Motor right is controlled from Motor Driv2 #3

    def _init_state(self):
        """Initialize all state variables"""
        # Motor speeds
        self.left_wheel_speed = self.BASE_SPEED
        self.right_wheel_speed = self.BASE_SPEED
        
        # Sensor state tracking
        self.sensor_left_prev = 0
        self.sensor_right_prev = 0
        self.on_perpendicular_line = False
        
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
        self.turning_case = 4
        self.is_running = True  # Start immediately since button not connected
        # self.last_button_time = 0
    
    def motor_turn_right(self):
        """Configure motor speeds for right turn"""
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed = 15
        
    
    def motor_turn_left(self):
        """Configure motor speeds for left turn"""
        self.left_wheel_speed = 15
        self.right_wheel_speed = self.MAX_SPEED
    
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
    
    def destroy(self):
        """Clean up resources - stop sensors and motors"""
        try:
            # self.vl53l0.stop()
            pass
        except Exception as e:
            print(f"Error stopping VL53L0X: {e}")
        finally:
            self.motors_off()
    
    def setup_interrupts(self):
        """Set up interrupt handlers for all sensors"""
        # Create closures that capture self for the interrupt handlers
        def line_handler(p):
            self.line_follower(p)
        
        def count_rising_handler(p):
            self.line_counter_rising(p)
        
        def count_falling_handler(p):
            self.line_counter_falling(p)
        
        # Attach interrupt handlers to mid sensor pins for line following
        self.signal_mid_right.irq(handler=line_handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self.signal_mid_left.irq(handler=line_handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        
        # Attach interrupt handlers to far sensor pins for line counting
        # Rising edge: count the line
        self.signal_far_left.irq(handler=count_rising_handler, trigger=Pin.IRQ_RISING)
        self.signal_far_right.irq(handler=count_rising_handler, trigger=Pin.IRQ_RISING)
        # Falling edge: reset the flag
        self.signal_far_left.irq(handler=count_falling_handler, trigger=Pin.IRQ_FALLING, append=True)
        self.signal_far_right.irq(handler=count_falling_handler, trigger=Pin.IRQ_FALLING, append=True)
    
        # Enable interrupts for button (not connected yet)
        # self.button.irq(handler=self.button_handler, trigger=Pin.IRQ_FALLING) 

    def disable_interrupts(self):
        """Disable interrupt handlers for all sensors"""
        self.signal_mid_right.irq(handler=None)
        self.signal_mid_left.irq(handler=None)
        self.signal_far_left.irq(handler=None)
        self.signal_far_right.irq(handler=None)

        # do NOT disable interrupt for button

    # def button_handler(self, p):
    #     """
    #     Interrupt handler for button press.
    #     Toggles robot state between running and stopped.

    #     Args:
    #         p: Pin that triggered the interrupt
    #     """
    #     current_time = ticks_ms()
        
    #     # Debounce: ignore if button pressed too soon after last press
    #     if ticks_diff(current_time, self.last_button_time) < self.DEBOUNCE_MS:
    #         return
        
    #     self.last_button_time = current_time
        
    #     # Toggle running state
    #     if self.is_running:
    #         # Stop the robot
    #         print("Button pressed - Stopping robot")
    #         self.is_running = False
    #         self.destroy()
    #     else:
    #         # Start the robot
    #         print("Button pressed - Starting robot")
    #         self.is_running = True
    #         # Reset state for fresh start
    #         self.count_lines = 0
    #         self.turning_case = 0
    #         self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, direction="forward")

    def line_counter_rising(self, p):
        """
        Interrupt handler for line counting (rising edge).
        Detects when robot crosses perpendicular lines using far sensors.
        Only triggered on rising edge (0→1 transition).
        
        Args:
            p: Pin that triggered the interrupt
        """
        if not self.is_running:
            return
        
        # Only count if not already on a perpendicular line (prevents double counting)
        if not self.on_perpendicular_line:
            self.count_lines += 1
            self.on_perpendicular_line = True
            print("Lines detected:", self.count_lines)
    
    def line_counter_falling(self, p):
        """
        Interrupt handler for resetting line counter flag (falling edge).
        Resets the flag when robot leaves the perpendicular line.
        Only triggered on falling edge (1→0 transition).
        
        Args:
            p: Pin that triggered the interrupt
        """
        if not self.is_running:
            return
        
        # Check if both far sensors are off before resetting
        sensor_far_left = self.signal_far_left.value()
        sensor_far_right = self.signal_far_right.value()
        
        if sensor_far_left == 0 and sensor_far_right == 0:
            # Both sensors off - we've left the perpendicular line
            self.on_perpendicular_line = False

    def line_follower(self, p):
        """
        Interrupt handler for line sensors.
        Adjusts motor speeds based on sensor readings to keep robot on line.
        
        Args:
            p: Pin that triggered the interrupt
        """

        if not self.is_running:
            return

        # Read current sensor values (only mid sensors needed for line following)
        sensor_mid_left = self.signal_mid_left.value()
        sensor_mid_right = self.signal_mid_right.value()

        print("mid left: ", sensor_mid_left, " mid right: ", sensor_mid_right)
        
        # Proportional control for line following
        # Calculate error: -1 (left of line), 0 (on line), +1 (right of line)
        error = 0
        
        if sensor_mid_right == 1 and sensor_mid_left == 0:
            # Right sensor on line - car drifting right
            error = 1  # Positive error: need to turn left
            
        elif sensor_mid_left == 1 and sensor_mid_right == 0:
            # Left sensor on line - car drifting left
            error = -1  # Negative error: need to turn right
            
        elif sensor_mid_left == 1 and sensor_mid_right == 1:
            # Both on line - go straight
            error = 0
            
        else:
            # Both sensors off line - use previous state for aggressive correction
            if self.sensor_left_prev == 1 and self.sensor_right_prev == 0:
                # Last seen left on line - turn left aggressively
                error = 1.5  # Larger error for stronger correction
            elif self.sensor_right_prev == 1 and self.sensor_left_prev == 0:
                # Last seen right on line - turn right aggressively
                error = -1.5  # Larger error for stronger correction
            else:
                # No correction needed - continue current trajectory
                error = 0
        
        # Apply proportional control if error detected
        if error != 0:
            # Calculate speed adjustment based on error magnitude
            correction = self.KP * error
            
            # Apply correction (positive error increases left, decreases right)
            self.left_wheel_speed = int(max(min(self.BASE_SPEED + correction, self.MAX_SPEED), self.MIN_SPEED))
            self.right_wheel_speed = int(max(min(self.BASE_SPEED - correction, self.MAX_SPEED), self.MIN_SPEED))
        else:
            # On track - maintain base speed
            self.left_wheel_speed = self.BASE_SPEED
            self.right_wheel_speed = self.BASE_SPEED
        
        # Apply motor speeds immediately (no delay waiting for main loop)
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, direction="forward")
        
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
        print(f"start: turning_case {case_name} turning")
        self.disable_interrupts()
        turn_function()
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, direction="forward")
        sleep(sleep_time)
        self.turning_case = next_case
        self.count_lines = 0
        self.setup_interrupts()
        print(f"end: turning_case {case_name} turning")

    def _calculate_distance(self) -> int:
        """
        Read distance from VL53L0X sensor.
        
        Returns:
            Distance in millimeters, or 9999 if sensor fails
        """
        distance = self.vl53l0.read()
        print(">>> Dist: ", distance)
        sleep(0.1)  # Wait for sensor reading
        return distance if distance is not None else 9999  # Return large value on failure to avoid false triggers
    
    def _handle_turning_cases(self):
        """
        Process turning logic based on current turning case.
        Returns True if final case is complete (should exit), False otherwise.
        """
        # Calculate distance once and cache it for all conditions
        distance = self._calculate_distance()

        if self.count_lines > 1 and distance < 250 and self.turning_case == 0:
            # if (sensor_far_right == 1):
            self._execute_turn(self.motor_turn_right, 1, 1, "0")
            

        elif self.count_lines > 1 and distance < 300 and self.turning_case == 1:
            # if (sensor_far_left == 1) :
            self._execute_turn(self.motor_turn_left, 1, 2, "1")
            
            
        elif self.count_lines > 6 and distance < 300 and self.turning_case == 2:
            # if (sensor_far_left == 1) :
            self._execute_turn(self.motor_turn_left, 1, 3, "2")
            
            
        elif self.count_lines > 1 and distance < 300 and self.turning_case == 3:
            # if (sensor_far_left == 1) :
            self._execute_turn(self.motor_turn_left, 1, 4, "3")
            
            
        elif self.count_lines > 6 and distance < 1000 and self.turning_case == 4:
            # if (sensor_far_left == 1) :
            self._execute_turn(self.motor_turn_left, 1.5, 5, "4")
            
            
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

    # Color detection disabled - not connected yet
    # class BASE_COLOUR(Enum):
    #     RED = (140, 60, 60)
    #     BLUE = (40, 120, 120)
    #     GREEN = (80, 120, 120)
    #     YELLOW = (120, 120, 40)

    # def trigger_colour_detection(self) -> (int, int, int):
    #     return None

    # def _colour(self) -> BASE_COLOUR:
    #     """
    #     Detect the color of the surface under the robot using the color sensor.

    #     Returns:
    #         BASE_COLOUR: The detected color
    #     """
    #     # Read RGB values from the color sensor
    #     r, g, b = self.trigger_colour_detection()

    #     # Determine the closest color based on RGB values
    #     min_distance = float('inf')
    #     closest_color = None

    #     for color in self.BASE_COLOUR:
    #         # Calculate Euclidean distance between sensor reading and color values
    #         distance = math.sqrt((r - color.value[0]) ** 2 + (g - color.value[1]) ** 2 + (b - color.value[2]) ** 2)

    #         # Update closest color if current distance is smaller
    #         if distance < min_distance:
    #             min_distance = distance
    #             closest_color = color

    #     return closest_color
    
    def run(self):
        """
        Main control loop for the line follower.
        Continuously monitors sensors and adjusts motors until sequence is complete.
        """
        # print("Robot ready. Press button to start...")
        # while not self.is_running:
        #     sleep(0.1)

        print("Start line follower")
        
        # Start moving forward
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, direction="forward")
        
        while self.is_running:
            # Handle turning cases - exit if final case is complete
            if self._handle_turning_cases():
                self.is_running = False
                self.destroy()  # Clean up when finished
                break
            
            # Longer sleep since interrupts handle line following immediately
            sleep(0.05)  # Reduced from 100Hz to 20Hz polling

        print("Robot stopped. Press button to restart...")


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
    finally:
        # Always clean up resources
        robot.destroy()
