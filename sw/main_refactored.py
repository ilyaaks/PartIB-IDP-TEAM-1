from machine import Pin, I2C, ADC
from utime import sleep, ticks_ms, ticks_diff
from sw.test_motor import Motor
from sw.path import Path
from sw.test_linear_actuator import Actuator
from sw.libs.VL53L0X.VL53L0X import VL53L0X
from sw.libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8801, DFRobot_TMF8701  # Second distance sensor
# from sw.libs.DFRobot_URM09.DFRobot_URM09 import DFRobot_URM09
from sw.libs.tcs3472_micropython.tcs3472 import tcs3472
from sw.colour_sensor import ColourSensor


class LineFollowerRobot:
    """
    Autonomous line-following robot with box detection, pickup, and sorting capabilities.
    
    Features:
    - Line following using IR sensors with interrupt-driven control
    - Distance measurement using VL53L0X and TMF8x01 sensors
    - Color detection for box sorting
    - Linear actuator control for box pickup/placement
    - Path planning algorithm for navigation between bays
    """

    # Motor speed configuration constants
    BASE_SPEED = 60           # Default speed for straight line following
    SPEED_ADJUSTMENT = 6      # Speed delta for line correction
    MIN_SPEED = 30            # Minimum motor speed
    MAX_SPEED = 80            # Maximum motor speed

    # Bay search order (unused - reserved for future implementation)
    SEARCH_LIST = [
        "B01", "B02", "B03", "B04", "B05", "B06",
        "B16", "B15", "B14", "B13", "B12", "B11",
        "A11", "A12", "A13", "A14", "A15", "A16",
        "A06", "A05", "A04", "A03", "A02", "A01"
    ]

    # Pin configurations for sensors and actuators for sensors and actuators
    MID_RIGHT_PIN = 26        # IR sensor for line following (right center)
    MID_LEFT_PIN = 16         # IR sensor for line following (left center) (instead of 27)
    FAR_RIGHT_PIN = 22        # IR sensor for line counting (right outer)
    FAR_LEFT_PIN = 28         # IR sensor for line counting (left outer)
    SDA_PIN = 20              # I2C data line for VL53L0X distance sensor
    SCL_PIN = 21              # I2C clock line for VL53L0X distance sensor
    YELLOW_LED = 14           # Status indicator LED

    TMF8X01_SDA_PIN = 18      # I2C data line for TMF8x01 ToF sensor
    TMF8X01_SCL_PIN = 19      # I2C clock line for TMF8x01 ToF sensor

    BUTTON_PIN = 17           # Start/stop button input
    DEBOUNCE_MS = 200         # Button debounce time in milliseconds

    def __init__(self):
        """Initialize robot hardware and state.
        
        Parameters:
            None
        
        Returns:
            None
        """
        # Initialize hardware
        self._init_sensors()
        self._init_motors()

        # Initialize state variables
        self._init_state()

        # Start distance sensor
        self.vl53l0.start()
        self.tof.begin()
        self.tof.start_measurement(calib_m=self.tof.eMODE_NO_CALIB, mode=self.tof.ePROXIMITY)
        
        # Track sensor states to avoid current draw conflicts
        self.vl53l0_running = True

    def _init_sensors(self):
        """Initialize all sensors and I2C peripherals.
        
        Sets up:
        - 4 IR line sensors (mid and far, left and right)
        - VL53L0X distance sensor (forward-facing)
        - TMF8x01 Time-of-Flight sensor (forward-facing)
        - TCS3472 color sensor
        - URM09 ultrasonic sensor (analog)
        - Button input for manual control
        - Status LED
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.signal_mid_right = Pin(self.MID_RIGHT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_mid_left = Pin(self.MID_LEFT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_far_right = Pin(self.FAR_RIGHT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_far_left = Pin(self.FAR_LEFT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.yellow_led = Pin(self.YELLOW_LED, Pin.OUT)
        self.yellow_led.value(1)

        # config I2C Bus
        i2c_bus_vl5310 = I2C(0, sda=Pin(self.SDA_PIN), scl=Pin(self.SCL_PIN))  # I2C0 on GP8 & GP9
        i2c_bus_tmf8x01 = I2C(1, sda=Pin(self.TMF8X01_SDA_PIN), scl=Pin(self.TMF8X01_SCL_PIN),
                              freq=100000)  # I2C1 on GP2 & GP3

        # Setup vl53l0 object
        self.vl53l0 = VL53L0X(i2c_bus_vl5310)
        self.vl53l0.set_Vcsel_pulse_period(self.vl53l0.vcsel_period_type[0], 18)
        self.vl53l0.set_Vcsel_pulse_period(self.vl53l0.vcsel_period_type[1], 14)

        # Setup TMF8x01 object
        try:
            self.tof = DFRobot_TMF8701(i2c_bus=i2c_bus_tmf8x01)
        except:
            raise Exception("TMF8x01 not connected properly")

        # Setup digital button
        self.button = Pin(self.BUTTON_PIN, Pin.IN, Pin.PULL_DOWN)

        # Setup colour sensor (initialized on demand to avoid current conflicts with VL53L0X):
        self.colour_sensor = None  # Lazy initialization

        # Setup the URM09 Ultrasonic sensor:
        self.Max_range = 500
        self.adc_pin = ADC(Pin(27))
        self.ADC_Resolution = 65535

    def _init_motors(self):
        """Initialize motor controllers for wheels and linear actuator.
        
        Sets up:
        - Left wheel motor (Motor Driver #2)
        - Right wheel motor (Motor Driver #3)
        - Linear actuator for box pickup mechanism
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.motor_left = Motor(dirPin=7, PWMPin=6)  # Motor left is controlled from Motor Driv2 #2
        self.motor_right = Motor(dirPin=4, PWMPin=5)  # Motor right is controlled from Motor Driv2 #3
        self.linear_actuator = Actuator(dirPin=0, PWMPin=1)

    def _init_state(self):
        """Initialize all runtime state variables and position the actuator.
        
        Initializes:
        - Motor speed variables
        - Direction flags for forward/reverse movement
        - Sensor state tracking for line following
        - Line counter for navigation
        - Button debouncing state
        - Box detection flags
        - Sets linear actuator to initial position
        
        Parameters:
            None
        
        Returns:
            None
        """
        # Motor speeds
        self.left_wheel_speed = self.BASE_SPEED
        self.right_wheel_speed = self.BASE_SPEED

        # Direction control
        self.direction_flag = "forward"  # Global flag: "forward" or "reverse"
        self.found_box = False
        # Sensor state tracking
        self.sensor_left_prev = 0
        self.sensor_right_prev = 0
        self.on_perpendicular_line = False

        # Navigation state
        self.count_lines = 0
        self.is_running = False  # Wait for button press to start
        self.last_button_time = 0
        self.list_of_bays = ["GREEN", "RED", "BLUE", "YELLOW"]

        # self._actuator_initial_position()

    def _actuator_initial_position(self):
        """Move linear actuator to initial (raised) position for navigation.
        
        Extends actuator fully, then retracts to clear ground and boxes.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.linear_actuator.set("extend", 90)
        sleep(4)
        self.linear_actuator.set("retract", 90)
        sleep(2.5)
        self.linear_actuator.off()

    def _actuator_final_position(self):
        """Extend linear actuator to final position for box placement.
        
        Fully extends actuator to release box into sorting bay.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.linear_actuator.set("extend", 90)
        sleep(4)
        self.linear_actuator.off()

    def motor_turn_right(self):
        """Execute a right turn maneuver on the line.
        
        Turns robot right by rotating left wheel forward and right wheel backward,
        then waits for mid-left sensor to detect line before straightening.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.motors_off()
        sleep(0.1)
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed = -1 * self.MIN_SPEED
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)
        sleep(1.52)
        while (self.signal_mid_left.value() == 0):
            pass

    def motor_turn_left(self):
        """Execute a left turn maneuver on the line.
        
        Turns robot left by rotating right wheel forward and left wheel backward,
        then waits for mid-right sensor to detect line before straightening.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.motors_off()
        sleep(0.1)
        self.left_wheel_speed = -1 * self.MIN_SPEED
        self.right_wheel_speed = self.MAX_SPEED
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)
        sleep(1.52)
        while (self.signal_mid_right.value() == 0):
            pass

    def motor_turn_right_bay(self):
        """Execute a right turn specifically for entering/exiting bays.
        
        Similar to motor_turn_right but with adjusted timing and speed
        for sharper turns required when approaching collection bays.
        
        Parameters:
            None
        
        Returns:
            None
        """
        # while (self.signal_mid_right.value()== 0 or self.signal_mid_left.value()==0):
        #     pass
        # Turn right
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed = 10
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)
        sleep(1.5)
        while (self.signal_mid_left.value() == 0):
            pass
        # Adjust to line
        self.right_wheel_speed = -self.BASE_SPEED
        self.left_wheel_speed = self.BASE_SPEED
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)
        sleep(0.4)
        while (self.signal_mid_left.value() == 0):
            pass

    def motor_turn_left_bay(self):
        """Execute a left turn specifically for entering/exiting bays.
        
        Similar to motor_turn_left but with adjusted timing and speed
        for sharper turns required when approaching collection bays.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.right_wheel_speed = self.MAX_SPEED
        self.left_wheel_speed = -self.BASE_SPEED
        self.direction_flag = "forward"
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)
        sleep(1.5)
        while (self.signal_mid_left.value() == 0):
            pass
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed = self.BASE_SPEED
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)

    def motor_turn_right_back(self):
        """Execute a right turn while reversing.
        
        Used when exiting bays - reverses with asymmetric wheel speeds
        to turn right, then switches to forward once line is detected.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed =-self.BASE_SPEED
        self.direction_flag = "reverse"
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        sleep(1.5)
        while (self.signal_mid_left.value() == 0):
            pass
        self.right_wheel_speed = self.MAX_SPEED
        self.left_wheel_speed = 0
        self.direction_flag = "forward"
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        # Wait until the mid right sensor detects the line
        while (self.signal_mid_right.value() == 0):
            pass

    def motor_turn_left_back(self):
        """Execute a left turn while reversing.
        
        Used when exiting bays - reverses with asymmetric wheel speeds
        to turn left, then switches to forward once line is detected.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.left_wheel_speed = 0
        self.right_wheel_speed = self.MAX_SPEED
        self.direction_flag = "reverse"
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        sleep(1.5)  # Usually about this delay is used for a turn
        while (self.signal_mid_right.value() == 0):  # Wait until the mid right sensor detects the line
            pass
        self.right_wheel_speed = 0
        self.left_wheel_speed = self.MAX_SPEED
        self.direction_flag = "forward"
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        # Wait until the mid left sensor detects the line
        while (self.signal_mid_left.value() == 0):
            pass

    def _reverse_on_the_spot(self):
        """Perform a 180-degree turn in place by reversing motors in opposite directions.
        
        Rotates robot on its axis for approximately 2 seconds to face opposite direction.
        Used after completing box placement to return to navigation mode.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.direction_flag = "reverse"
        left_wheel = -self.MAX_SPEED
        right_wheel = self.MAX_SPEED
        self.motor_turn(left_wheel, right_wheel)
        # Wait until both mid sensors detect the line
        # TODO: Develop what time it takes to reverse like that
        sleep(2)
        self.motors_off()

    def motor_go_straight(self, speed_left, speed_right, delay=0):
        """Execute straight movement using the global direction_flag.
        
        Parameters:
            speed_left (int): Speed for left motor (0-100)
            speed_right (int): Speed for right motor (0-100)
            delay (float): Optional delay in seconds after moving (default: 0)
        
        Returns:
            None
        """
        if self.direction_flag == "forward":
            self.motor_left.Forward(speed_left)
            self.motor_right.Forward(speed_right)
        elif self.direction_flag == "reverse":
            self.motor_left.Reverse(speed_left)
            self.motor_right.Reverse(speed_right)
        if delay > 0:
            sleep(delay)

    def motor_turn(self, speed_left, speed_right):
        """Execute motor movement with independent wheel control.
        
        Automatically determines direction based on speed signs:
        - Both positive: Move forward/straight
        - Left positive, right negative: Turn right
        - Left negative, right positive: Turn left
        
        Parameters:
            speed_left (int): Speed for left motor (-100 to 100, negative = reverse)
            speed_right (int): Speed for right motor (-100 to 100, negative = reverse)
        
        Returns:
            None
        """
        if speed_left >= 0 and speed_right >= 0:
            self.motor_go_straight(speed_left, speed_right)
        elif speed_left >= 0 and speed_right <= 0:
            speed_right = abs(speed_right)
            self.motor_left.Forward(speed_left)
            self.motor_right.Reverse(speed_right)
        elif speed_left <= 0 and speed_right >= 0:
            # Turn left: left motor reverse, right motor forward
            speed_left = abs(speed_left)
            self.motor_left.Reverse(speed_left)
            self.motor_right.Forward(speed_right)

    def motors_off(self):
        """Turn off both motors.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.motor_left.off()
        self.motor_right.off()

    def destroy(self):
        """Clean up resources - stop sensors and motors.
        
        Parameters:
            None
        
        Returns:
            None
        """
        try:
            if self.vl53l0_running:
                self.vl53l0.stop()
                self.vl53l0_running = False
        except Exception as e:
            print(f"Error stopping VL53L0X: {e}")
        finally:
            self.motors_off()

    def setup_interrupts(self):
        """Set up interrupt handlers for all sensors.
        
        Parameters:
            None
        
        Returns:
            None
        """

        # Create closures that capture self for the interrupt handlers
        def line_handler(p):
            self.line_follower(p)

        def count_handler(p):
            self.line_counter(p)

        # Attach interrupt handlers to mid sensor pins for line following
        self.signal_mid_right.irq(handler=line_handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self.signal_mid_left.irq(handler=line_handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)

        # Attach interrupt handlers to far sensor pins for line counting (both rising and falling edges)
        self.signal_far_left.irq(handler=count_handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self.signal_far_right.irq(handler=count_handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)

        # Enable interrupts for button
        self.button.irq(handler=self.button_handler, trigger=Pin.IRQ_FALLING)

    def disable_interrupts(self):
        """Disable interrupt handlers for all sensors.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.signal_mid_right.irq(handler=None)
        self.signal_mid_left.irq(handler=None)
        # self.signal_far_left.irq(handler=None)
        # self.signal_far_right.irq(handler=None)
        self.sensor_left_prev = 0
        self.sensor_right_prev = 0

    def _path_algorithm(self, current_position, desired_position):
        """Navigate robot from current position to desired position using predefined paths.
        
        Position format:
        - "G0": Ground/start position
        - "B0X" / "B1X": Bay B, side 0/1, line X (1-6)
        - "A0X" / "A1X": Bay A, side 0/1, line X (1-6)
        - Color names: Sorting bay destinations
        
        Executes the path by calling appropriate Path class methods and executing turns.
        
        Parameters:
            current_position (str): Starting position (format: "G0", "B01"-"B16", "A01"-"A16")
            desired_position (str): Target position (same format, or color bay: "GREEN", "RED", "BLUE", "YELLOW")
        
        Returns:
            None
        """

        if (current_position.startswith("G") and desired_position.startswith("B0")):
            turns, lines = Path._path_G_to_B0(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position.startswith("G") and desired_position.startswith("B1")):
            turns, lines = Path._path_G_to_B1(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position.startswith("G") and desired_position.startswith("A0")):
            turns, lines = Path._path_G_to_A0(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position.startswith("G") and desired_position.startswith("A1")):
            turns, lines = Path._path_G_to_A1(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position.startswith("B0") and desired_position.startswith("G")):
            turns, lines = Path._path_B0_to_G(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position.startswith("B1") and desired_position.startswith("G")):
            turns, lines = Path._path_B1_to_G(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        # B0 to color bays
        if (current_position.startswith("B0") and desired_position == "GREEN"):
            turns, lines = Path._path_B0_to_Green_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("B0") and desired_position == "RED"):
            turns, lines = Path._path_B0_to_Red_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("B0") and desired_position == "BLUE"):
            turns, lines = Path._path_B0_to_Blue_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("B0") and desired_position == "YELLOW"):
            turns, lines = Path._path_B0_to_Yellow_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("B0") and desired_position.startswith("B1")):
            turns, lines = Path._path_B0_to_B1(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        # B1 to color bays
        if (current_position.startswith("B1") and desired_position == "GREEN"):
            turns, lines = Path._path_B1_to_Green_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("B1") and desired_position == "RED"):
            turns, lines = Path._path_B1_to_Red_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("B1") and desired_position == "BLUE"):
            turns, lines = Path._path_B1_to_Blue_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("B1") and desired_position == "YELLOW"):
            turns, lines = Path._path_B1_to_Yellow_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position.startswith("A0") and desired_position.startswith("G")):
            turns, lines = Path._path_A0_to_G(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("A0") and desired_position == "GREEN"):
            turns, lines = Path._path_A0_to_Green_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("A0") and desired_position == "RED"):
            turns, lines = Path._path_A0_to_Red_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("A0") and desired_position == "BLUE"):
            turns, lines = Path._path_A0_to_Blue_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("A0") and desired_position == "YELLOW"):
            turns, lines = Path._path_A0_to_Yellow_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("A0") and desired_position.startswith("A1")):
            turns, lines = Path._path_A0_to_A1(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position.startswith("A1") and desired_position == "GREEN"):
            turns, lines = Path._path_A1_to_Green_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("A1") and desired_position == "RED"):
            turns, lines = Path._path_A1_to_Red_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("A1") and desired_position == "BLUE"):
            turns, lines = Path._path_A1_to_Blue_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("A1") and desired_position == "YELLOW"):
            turns, lines = Path._path_A1_to_Yellow_bay(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        if (current_position.startswith("A1") and desired_position.startswith("G")):
            turns, lines = Path._path_A1_to_G(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

    def button_handler(self, p):
        """Interrupt handler for button press.
        
        Toggles robot state between running and stopped.
        
        Parameters:
            p (Pin): Pin object that triggered the interrupt
        
        Returns:
            None
        """
        current_time = ticks_ms()

        # Debounce: ignore if button pressed too soon after last press
        if ticks_diff(current_time, self.last_button_time) < self.DEBOUNCE_MS:
            return

        self.last_button_time = current_time

        # Toggle running state
        if self.is_running:
            # Stop the robot
            print("=== Button pressed - Stopping robot ===")
            self.is_running = False
            self.destroy()
        else:
            # Start the robot
            print("=== Button pressed - Starting robot ===")
            self.is_running = True
            # Reset state for fresh start
            self.count_lines = 0

    def line_counter(self, p):
        """Interrupt handler for line counting (both rising and falling edges).
        
        Detects when robot crosses perpendicular lines using far sensors.
        - Rising edge (0→1): Increments line count
        - Falling edge (1→0): Resets the on_perpendicular_line flag when both sensors are off
        
        Parameters:
            p (Pin): Pin object that triggered the interrupt
        
        Returns:
            None
        """
        if not self.is_running:
            return

        # Read current sensor values
        sensor_far_left = self.signal_far_left.value()
        sensor_far_right = self.signal_far_right.value()

        # Check if either sensor is detecting a line (rising edge case)
        if (sensor_far_left == 1 or sensor_far_right == 1) and not self.on_perpendicular_line:
            # Rising edge detected - count the line
            self.count_lines += 1
            self.on_perpendicular_line = True
            print("Lines detected:", self.count_lines)

        # Check if both sensors are off the line (falling edge case)
        elif sensor_far_left == 0 and sensor_far_right == 0 and self.on_perpendicular_line:
            # Falling edge detected - reset the flag
            self.on_perpendicular_line = False
            print("Left perpendicular line")

    def line_follower(self, p):
        """Interrupt handler for line sensors.
        
        Adjusts motor speeds based on sensor readings to keep robot on line.
        
        Parameters:
            p (Pin): Pin object that triggered the interrupt
        
        Returns:
            None
        """

        # print("mid left:", self.signal_mid_left.value(), " mid right:", self.signal_mid_right.value())

        if not self.is_running:
            return

        # Read current sensor values (only mid sensors needed for line following)
        sensor_mid_left = self.signal_mid_left.value()
        sensor_mid_right = self.signal_mid_right.value()
        # print("mid left:", sensor_mid_left, " mid right:", sensor_mid_right)

        if sensor_mid_right == 1 and sensor_mid_left == 0:
            # Right sensor on line - car drifting left
            self.left_wheel_speed = min(self.BASE_SPEED + self.SPEED_ADJUSTMENT, self.MAX_SPEED)
            self.right_wheel_speed = max(self.BASE_SPEED - self.SPEED_ADJUSTMENT, self.MIN_SPEED)

        elif sensor_mid_left == 1 and sensor_mid_right == 0:
            # Left sensor on line - car drifting right
            self.left_wheel_speed = max(self.BASE_SPEED - self.SPEED_ADJUSTMENT, self.MIN_SPEED)
            self.right_wheel_speed = min(self.BASE_SPEED + self.SPEED_ADJUSTMENT, self.MAX_SPEED)

        elif sensor_mid_left == 1 and sensor_mid_right == 1:
            # Both on line - go straight
            self.left_wheel_speed = self.BASE_SPEED
            self.right_wheel_speed = self.BASE_SPEED

        else:
            # Both sensors off line - use previous state for aggressive correction
            if self.sensor_left_prev == 1 and self.sensor_right_prev == 0:
                # Last seen left on line - turn left aggressively
                self.left_wheel_speed = self.MIN_SPEED
                self.right_wheel_speed = self.MAX_SPEED
            elif self.sensor_right_prev == 1 and self.sensor_left_prev == 0:
                # Last seen right on line - turn right aggressively
                self.left_wheel_speed = self.MAX_SPEED
                self.right_wheel_speed = self.MIN_SPEED
            else:
                # On track - maintain base speed
                self.left_wheel_speed = self.BASE_SPEED
                self.right_wheel_speed = self.BASE_SPEED

        # Apply proportional control if error detected

        # Apply motor speeds immediately (no delay waiting for main loop)
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)

        # Update previous sensor states
        self.sensor_left_prev = sensor_mid_left
        self.sensor_right_prev = sensor_mid_right

    def _execute_turn(self, turn_function, case_name):
        """Helper method to execute a turn and update state.
        
        Parameters:
            turn_function (callable): Method to call for turning (motor_turn_left or motor_turn_right)
            case_name (str): Name of current case for logging
        
        Returns:
            None
        """
        print(f"start: {case_name} turning")
        # Allow the turn to be done
        self.disable_interrupts()
        turn_function()
        self.count_lines = 0
        # Restrat the interrupts after the turn and reset the speed
        self.direction_flag = "forward"
        self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED)
        self.setup_interrupts()

    def _calculate_distance(self) -> int:
        """Read distance from VL53L0X sensor.
        
        Parameters:
            None
        
        Returns:
            int: Distance in millimeters, or 9999 if sensor fails
        """
        distance = self.vl53l0.read()
        sleep(0.1)  # Wait for sensor reading
        return distance if distance is not None else 9999  # Return large value on failure to avoid false triggers

    def _calculate_distance_tof(self) -> int:
        """Read distance from TMF8x01 Time-of-Flight sensor.
        
        Parameters:
            None
        
        Returns:
            int: Distance in millimeters, or 9999 if data not ready or sensor fails
        """
        if self.tof.is_data_ready():
            distance = self.tof.get_distance_mm()
            return distance if distance is not None else 9999
        return 9999  # Return large value when data not ready to avoid false triggers

    def _calculate_distance_URM09(self) -> int:
        """Read distance from URM09 ultrasonic sensor via analog input.
        
        Converts ADC reading to distance using sensor's voltage-distance relationship.
        
        Parameters:
            None
        
        Returns:
            int: Distance in millimeters, or 8173 on error
        """
        adc_value = self.adc_pin.read_u16()
        distance = (adc_value / self.ADC_Resolution) * self.Max_range
        if distance is not None:
            return distance
        else:
            print("URM09 sensor error")
            return 8173

    def _handle_identifying_cases(self, turns, lines):
        """Navigate through a sequence of turns based on line counts.
        
        Parameters:
            turns (list): List of turn directions ("right_forward", "left_forward", "right_backward", "left_backward", "straight", "backward")
            lines (list): List of line counts at which to execute each turn
        
        Returns:
            None
        """
        current_instruction = 0  # Track which instruction we're on
        while current_instruction < len(lines):
            # Check if we've reached the line count for the current instruction
            if self.count_lines >= lines[current_instruction]:
                print(f"Lines matched: {lines[current_instruction]} for turn: {turns[current_instruction]}")

                if turns[current_instruction] == "right_forward":
                    self.direction_flag = "forward"
                    self._execute_turn(self.motor_turn_right, str(current_instruction))
                elif turns[current_instruction] == "left_forward":
                    self.direction_flag = "forward"
                    self._execute_turn(self.motor_turn_left, str(current_instruction))
                elif turns[current_instruction] == "straight":
                    # Continue straight - just increment instruction counter
                    self.direction_flag = "forward"
                    self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED)
                elif turns[current_instruction] == "stop":
                    self.motors_off()
                # Move to next instruction
                current_instruction += 1

            # Small delay to avoid tight loop
            sleep(0.05)
        print("Finished all turns in path")

    def _go_to_next_bay(self, current_bay: str):
        """Move the robot to the next bay in the search list.
        
        Parameters:
            current_bay (str): Current bay position as a string (e.g., "B0", "B1", "A0", "A1")
        
        Returns:
            str: Next bay identifier or "G0" if all bays searched
        """
        if (current_bay == "B0"):
            self._path_algorithm("B06", "B16")
            return "B1"
        elif (current_bay == "B1"):
            self._path_algorithm("B11", "B0")
            return "B0"
        elif (current_bay == "A1"):
            self._path_algorithm("A16", "G0")
            return "G0"
        elif (current_bay == "A0"):
            self._path_algorithm("A06", "A16")
            return "A1"

    def pick_box(self, start="G0", destination="A01"):
        """Navigate to a bay, search for a box, identify its color, and pick it up.
        
        Process:
        1. Navigate to specified bay
        2. Search each line in bay for boxes using distance sensors
        3. When box detected, turn into bay
        4. Identify box color
        5. Pick up box with actuator
        6. Return to main line
        
        Parameters:
            start (str): Starting position (default: "G0" - ground position)
            destination (str): Initial bay to search (default: "A01")
        
        Returns:
            tuple: (current_position, destination_bay)
                - current_position (str): Bay where box was found (e.g., "B03")
                - destination_bay (str): Color-coded destination ("GREEN", "RED", "BLUE", "YELLOW", or "G0")
        """
        self._path_algorithm(start, destination)
        temp_line_counter = int(destination[2])
        self.found_box = False

        current_bay = destination[0:2]  # It doesn`t matter the specific bay number, just the side
        self.direction_flag = "forward"
        self.motor_go_straight(self.MIN_SPEED, self.MIN_SPEED)
        
        while (not self.found_box):
            # Wait until we reach a perpendicular line
            self.moving_between_lines(current_bay)
            
            # Now we're on a line - check for box
            if self.is_box(current_bay):
                # Box found - is_box() sets self.found_box = True
                break
            else:
                # No box found on this line, increment counter
                temp_line_counter += 1
                
                if temp_line_counter > 6:
                    current_bay = self._go_to_next_bay(current_bay)
                    temp_line_counter = 1
                    if current_bay == "G0":
                        print("No boxes found in any bay.")
                        return "G0", "G0"

        current_position = current_bay + str(temp_line_counter)
        self._before_pick_box(current_bay)
        colour = self._do_pick_box()
        self._after_pick_box(current_bay)

        # Determine destination based on detected colour
        if colour is not None:
            if colour == "GREEN":
                position_to_go = "GREEN"
            elif colour == "RED":
                position_to_go = "RED"
            elif colour == "BLUE":
                position_to_go = "BLUE"
            elif colour == "YELLOW":
                position_to_go = "YELLOW"
            else:
                position_to_go = "G0"
        else:
            position_to_go = "G0"

        return current_position, position_to_go

    def moving_between_lines(self, position):
        """Wait until robot crosses to the next perpendicular line.
        
        Uses appropriate far sensor based on bay side to detect line crossing.
        
        Parameters:
            position (str): Current bay position ("B0", "B1", "A0", or "A1")
        
        Returns:
            None
        """
        if position == "B0" or position == "A1":
            while (self.signal_far_left.value() == 0):
                pass
        elif position == "B1" or position == "A0":
            while (self.signal_far_right.value() == 0):
                pass

    def is_box(self, position, timeout_ms=5000) -> bool:
        """Check for box presence while crossing a line using distance sensors.
        
        Uses appropriate distance sensor (ToF for B0/A1, URM09 for B1/A0)
        to detect boxes within 300mm during line crossing.
        
        Parameters:
            position (str): Current bay position ("B0", "B1", "A0", or "A1")
            timeout_ms (int): Maximum time to wait for sensor to go low (default: 5000ms)
        
        Returns:
            bool: True if box detected, False otherwise. Also sets self.found_box flag.
        """
        start_time = ticks_ms()
        
        if position == "B0" or position == "A1":
            while (self.signal_far_left.value() == 1):
                if ticks_diff(ticks_ms(), start_time) > timeout_ms:
                    print("Timeout waiting for far_left sensor in is_box()")
                    return False
                distance = self._calculate_distance_tof()
                if distance is not None and distance < 300:
                    self.found_box = True
                    return True
                sleep(0.05)
            return False
        elif position == "B1" or position == "A0":
            while (self.signal_far_right.value() == 1):
                if ticks_diff(ticks_ms(), start_time) > timeout_ms:
                    print("Timeout waiting for far_right sensor in is_box()")
                    return False
                print(self._calculate_distance_URM09())
                if (self._calculate_distance_URM09() < 21):
                    print("Box detected by URM09")
                    self.found_box = True
                    return True
                sleep(0.05)
            return False

    def _before_pick_box(self, current_position: str):
        """Turn into bay to approach detected box.
        
        Executes appropriate turn (left for B0/A1, right for B1/A0) to face box.
        Line following interrupts will auto-correct position after turn.
        
        Parameters:
            current_position (str): Current bay side ("B0", "B1", "A0", or "A1")
        
        Returns:
            None
        """
        if current_position[0:2] == "B0" or current_position[0:2] == "A1":
            self._execute_turn(self.motor_turn_left_bay, f"before_pick_box_{current_position}")
            self.motors_off()

        elif current_position[0:2] == "A0" or current_position[0:2] == "B1":
            self._execute_turn(self.motor_turn_right_bay, f"before_pick_box_{current_position}")
            self.motors_off()
            # Wait until centered on line or timeout

    def _after_pick_box(self, current_position):
        """Return robot to main line after picking up box.
        
        Process:
        1. Reverse out of bay
        2. Execute backward turn to face main line
        3. Move forward until fully on line
        
        Parameters:
            current_position (str): Current bay side ("B0", "B1", "A0", or "A1")
        
        Returns:
            None
        """
        if current_position[0:2] == "B0" or current_position[0:2] == "A1":
            self.direction_flag = "reverse"
            self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, delay=0.5)
            self._execute_turn(self.motor_turn_right_back, "after_pick_box")
            self.direction_flag = "forward"
            self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
            while (self.signal_far_right.value() == 1):
                pass
        elif current_position[0:2] == "B1" or current_position[0:2] == "A0":
            self.direction_flag = "reverse"
            self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, delay=0.5)
            self._execute_turn(self.motor_turn_left_back, "after_pick_box")
            self.direction_flag = "forward"
            self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED)
            # Wait until the far left sensor is off the line 
            while self.signal_far_left.value() == 1:
                pass
        # self.motors_off()

    def _reinit_vl53l0(self):
        """Re-initialize VL53L0X sensor after power conflict.
        
        Fully re-creates the sensor object to recover from I2C errors
        caused by current draw conflicts with colour sensor.
        
        Parameters:
            None
        
        Returns:
            None
        """
        print("Re-initializing VL53L0X sensor...")
        sleep(0.5)  # Allow current to settle before re-init
        i2c_bus_vl5310 = I2C(0, sda=Pin(self.SDA_PIN), scl=Pin(self.SCL_PIN))
        self.vl53l0 = VL53L0X(i2c_bus_vl5310)
        self.vl53l0.set_Vcsel_pulse_period(self.vl53l0.vcsel_period_type[0], 18)
        self.vl53l0.set_Vcsel_pulse_period(self.vl53l0.vcsel_period_type[1], 14)
        self.vl53l0.start()
        self.vl53l0_running = True
        sleep(0.1)  # Allow sensor to stabilize

    def _start_vl53l0(self):
        """Start VL53L0X distance sensor if not already running.
        
        Used for power management to avoid current draw conflicts with colour sensor.
        If starting fails due to I2C error, attempts full re-initialization.
        
        Parameters:
            None
        
        Returns:
            None
        """
        if not self.vl53l0_running:
            print("Starting VL53L0X sensor...")
            try:
                self.vl53l0.start()
                self.vl53l0_running = True
                sleep(0.1)  # Allow sensor to stabilize
            except OSError as e:
                print(f"VL53L0X start failed ({e}), re-initializing...")
                self._reinit_vl53l0()

    def _stop_vl53l0(self):
        """Stop VL53L0X distance sensor to reduce current draw.
        
        Used for power management to allow colour sensor to operate.
        
        Parameters:
            None
        
        Returns:
            None
        """
        if self.vl53l0_running:
            print("Stopping VL53L0X sensor...")
            self.vl53l0.stop()
            self.vl53l0_running = False
            sleep(0.1)  # Allow current to settle

    def _init_colour_sensor(self):
        """Initialize colour sensor on demand.
        
        Lazy initialization to avoid current draw issues at startup.
        Must stop VL53L0X before using colour sensor.
        
        Parameters:
            None
        
        Returns:
            None
        """
        if self.colour_sensor is None:
            print("Initializing colour sensor...")
            self.colour_sensor = ColourSensor()
            sleep(0.2)  # Allow sensor to stabilize

    def _do_pick_box(self):
        """Approach box, identify color, and activate pickup mechanism.
        
        Attempts up to 3 times to:
        1. Approach box to optimal distance (using VL53L0X)
        2. Stop VL53L0X to allow colour sensor to work
        3. Detect box color using color sensor
        4. Restart VL53L0X after colour detection
        5. Activate linear actuator to secure box
        
        If color detection fails, reverses and retries.
        
        Parameters:
            None
        
        Returns:
            str: Detected color name ("GREEN", "RED", "BLUE", "YELLOW") or None on failure
        """
        colour_detected = False
        number_of_attempts = 0
        identified_colour = None

        # Identify the colour of the box before picking it up
        while not colour_detected and number_of_attempts < 3:
            # Ensure VL53L0X is running for approach
            self._start_vl53l0()
            self._approach_box()
            
            # Stop VL53L0X before using colour sensor to avoid current conflicts
            self._stop_vl53l0()
            
            # Initialize colour sensor if needed (lazy init)
            self._init_colour_sensor()
            
            identified_colour = self.colour_sensor.detect_colour()
            print("Identified colour:", identified_colour)

            if identified_colour is not None:
                colour_detected = True
                # Don't restart VL53L0X here - actuator operation doesn't need it
                # It will be re-initialized later when navigation resumes
                self.linear_actuator.set("retract", 90)
                sleep(3)
                self.linear_actuator.off()
            else:
                print("Couldn't identify colour, redoing the pick up.")
                # Re-initialize VL53L0X for reversing and retry (full re-init after colour sensor use)
                self._reinit_vl53l0()
                self.direction_flag = "reverse"
                self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED, delay=1)
                self.motors_off()
                number_of_attempts += 1

        if not colour_detected:
            print("Failed to identify colour after 3 attempts, aborting pick up.")
            # Re-initialize VL53L0X for further operations
            self._reinit_vl53l0()
            self.motors_off()
            return None

        return identified_colour

    def _approach_box(self):
        """Move slowly toward box until reaching optimal distance for color detection.
        
        Uses VL53L0X distance sensor to stop at 60mm from box,
        ideal distance for color sensor accuracy.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.direction_flag = "forward"
        self.motor_go_straight(self.MIN_SPEED, self.MIN_SPEED)
        while (self._calculate_distance() > 60):
            pass
        self.motors_off()

    def _place_box(self):
        """Place box in sorting bay by extending actuator and reversing out.
        
        Process:
        1. Move forward into bay
        2. Extend actuator to release box
        3. Reverse out of bay
        4. Return actuator to initial position
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.direction_flag = "forward"
        self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED, delay=3)
        self.motors_off()
        self._actuator_final_position()
        self.linear_actuator.off()
        self.direction_flag = "reverse"
        self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED, delay=1.5)
        self.motors_off()
        self._actuator_initial_position()

    def run(self):
        """Main control loop for the line follower.
        
        Continuously monitors sensors and adjusts motors until sequence is complete.
        Waits for button press to start, then executes box search, pickup, and placement.
        
        Parameters:
            None
        
        Returns:
            None
        """
        print("Robot ready. Press button to start...")
        while not self.is_running:
            sleep(0.1)

        print("Start line follower")
        # Start the flashing LED to indicate running
        self.yellow_led.value(0)
        # Start the algorithm to pick boxes
        # self._path_algorithm("G0", "A16")
        # self._before_pick_box("A16")
        # self._do_pick_box()
        # self.pick_box(start="G0", destination="A01")
        self._do_pick_box()

        # Last line of the run function
        print("Run function has ended.")


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
