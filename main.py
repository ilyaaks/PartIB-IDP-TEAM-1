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
    
    Hardware Components:
    - 4 IR line sensors (2 for line following, 2 for line counting)
    - 2 DC motors with motor drivers for differential drive
    - Linear actuator for box pickup mechanism
    - VL53L0X laser distance sensor (forward-facing, I2C bus 0)
    - TMF8x01 Time-of-Flight sensor (forward-facing, I2C bus 1)
    - TCS3472 colour sensor (on-demand initialisation)
    - URM09 ultrasonic sensor (analog ADC input)
    - Button for start/stop control
    - Status LED indicator
    
    Key Features:
    - Interrupt-driven line following for responsive control
    - Multi-sensor distance measurement for box detection
    - Box colour detection (GREEN, RED, BLUE, YELLOW)
    - Sophisticated path planning between bay locations
    - Power management to avoid sensor current conflicts
    - Automatic bay search algorithm
    
    Navigation System:
    - Uses grid-based coordinate system (e.g., "B01", "A16", "G0")
    - Counts perpendicular line crossings for position tracking
    - Executes complex turn sequences for path following
    """

    # Motor speed configuration constants
    BASE_SPEED = 65           # Default speed for straight line following (balanced for accuracy)
    SPEED_ADJUSTMENT = 8      # Speed delta for proportional line correction (added/subtracted from base)
    MIN_SPEED = 30            # Minimum motor speed (prevents stalling during tight corrections)
    MAX_SPEED = 80            # Maximum motor speed (limited to maintain control and avoid overshooting)

    # Bay search order (unused - reserved for reference)
    SEARCH_LIST = [
        "B01", "B02", "B03", "B04", "B05", "B06",
        "B16", "B15", "B14", "B13", "B12", "B11",
        "A11", "A12", "A13", "A14", "A15", "A16",
        "A06", "A05", "A04", "A03", "A02", "A01"
    ]

    # Pin configurations for sensors and actuators for sensors and actuators
    MID_RIGHT_PIN = 26        # IR sensor for line following (right center)
    MID_LEFT_PIN = 16         # IR sensor for line following (left center)
    FAR_RIGHT_PIN = 22        # IR sensor for line counting (right outer)
    FAR_LEFT_PIN = 28         # IR sensor for line counting (left outer)
    VL53L0X_SDA_PIN = 20      # I2C data line for VL53L0X distance sensor
    VL53L0X_SCL_PIN = 21      # I2C clock line for VL53L0X distance sensor
    YELLOW_LED = 14           # Status indicator LED

    TMF8X01_SDA_PIN = 18      # I2C data line for TMF8x01 ToF sensor
    TMF8X01_SCL_PIN = 19      # I2C clock line for TMF8x01 ToF sensor

    BUTTON_PIN = 17           # Start/stop button input
    DEBOUNCE_MS = 200         # Button debounce time in milliseconds

    def __init__(self):
        """Initialise robot hardware and state.
        
        Parameters:
            None
        
        Returns:
            None
        """
        # Initialise hardware
        self._init_sensors()
        self._init_motors()

        # Initialise state variables
        self._init_state()

        # Start distance sensor
        self.vl53l0.start()

        # ToF sensor is not used in the actual competition
        # self.tof.begin()
        # self.tof.start_measurement(calib_m=self.tof.eMODE_NO_CALIB, mode=self.tof.ePROXIMITY)
        
        # Track sensor states to avoid current draw conflicts
        self.vl53l0_running = True

    def _init_sensors(self):
        """Initialise all sensors and I2C peripherals.
        
        Sets up:
        - 4 IR line sensors (mid and far, left and right) - digital inputs for line detection
        - VL53L0X distance sensor - laser ranging for precise box approach
        - TMF8x01 Time-of-Flight sensor - wider beam for initial box detection
        - TCS3472 colour sensor - RGB colour detection for box sorting
        - URM09 ultrasonic sensor - alternative distance measurement
        - Button input - manual start/stop control with debouncing
        - Status LED - visual feedback for robot state
        
        Note: Colour sensor is initialised on-demand (_init_colour_sensor) to avoid
        current draw conflicts with VL53L0X on the same power rail.
        
        Parameters:
            None
        
        Returns:
            None
        """
        # Configure IR line sensors as digital inputs with pull-down resistors (LOW when off line, HIGH when on black line)
        self.signal_mid_right = Pin(self.MID_RIGHT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_mid_left = Pin(self.MID_LEFT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_far_right = Pin(self.FAR_RIGHT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_far_left = Pin(self.FAR_LEFT_PIN, Pin.IN, Pin.PULL_DOWN)
        
        # Configure status LED (initially HIGH = off, will be set LOW when running)
        self.yellow_led = Pin(self.YELLOW_LED, Pin.OUT)
        self.yellow_led.value(1)  # Start with LED off

        # Configure I2C buses (separate buses to avoid address conflicts and timing issues)
        i2c_bus_vl5310 = I2C(0, sda=Pin(self.VL53L0X_SDA_PIN), scl=Pin(self.VL53L0X_SCL_PIN))  # I2C0: VL53L0X laser sensor
        i2c_bus_tmf8x01 = I2C(1, sda=Pin(self.TMF8X01_SDA_PIN), scl=Pin(self.TMF8X01_SCL_PIN),
                              freq=100000)  # I2C1: TMF8x01 ToF sensor (100kHz for stability)

        # Initialise VL53L0X laser distance sensor
        self.vl53l0 = VL53L0X(i2c_bus_vl5310)
        # Configure VCSEL (Vertical Cavity Surface Emitting Laser) pulse periods for optimal performance
        self.vl53l0.set_Vcsel_pulse_period(self.vl53l0.vcsel_period_type[0], 18) 
        self.vl53l0.set_Vcsel_pulse_period(self.vl53l0.vcsel_period_type[1], 14)

        # Setup TMF8x01 object
        try:
            self.tof = DFRobot_TMF8701(i2c_bus=i2c_bus_tmf8x01)
        except:
            raise Exception("TMF8x01 not connected properly")

        # Setup start/stop button (active LOW when pressed due to pull-down)
        self.button = Pin(self.BUTTON_PIN, Pin.IN, Pin.PULL_DOWN)

        # Setup colour sensor (initialised on demand to avoid large current spike conflicts with VL53L0X)
        self.colour_sensor = None  # Lazy initialisation via _init_colour_sensor() only when needed
        self.red_led = Pin(10, Pin.OUT)  # Control pin for colour sensor's illumination LED
        self.red_led.value(1)  # Turn off LED initially (active LOW)
        
        # Setup URM09 ultrasonic sensor via analog ADC (voltage output proportional to distance)
        self.Max_range = 500  # Maximum detection range in cm (sensor spec)
        self.adc_pin = ADC(Pin(27))  # Read analog voltage from URM09 output
        self.ADC_Resolution = 65535  # 16-bit ADC resolution

    def _init_motors(self):
        """Initialise motor controllers for wheels and linear actuator.
        
        Sets up:
        - Left wheel motor (DIR=GP7, PWM=GP6) - forward/reverse control
        - Right wheel motor (DIR=GP4, PWM=GP5) - forward/reverse control
        - Linear actuator (DIR=GP0, PWM=GP1) - extend/retract for box pickup
        
        Motor objects handle:
        - PWM speed control (0-100% duty cycle)
        - Safe shutdown on errors
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.motor_left = Motor(dirPin=7, PWMPin=6)  
        self.motor_right = Motor(dirPin=4, PWMPin=5)  
        self.linear_actuator = Actuator(dirPin=0, PWMPin=1)

    def _init_state(self):
        """Initialise all runtime state variables and position the actuator.
        
        Initialises:
        - Motor speed variables (left_wheel_speed, right_wheel_speed, current_speed)
        - Direction flags (direction_flag: "forward" or "reverse")
        - Sensor state tracking (sensor_left_prev, sensor_right_prev) for edge detection
        - Line counter (count_lines) for navigation tracking
        - Button debouncing state (last_button_time) to prevent accidental double-presses
        - Box detection flags (found_box) for search algorithm
        - Bay list for sorting destinations
        - Robot running state (is_running) controlled by button
        - Perpendicular line detection flag (on_perpendicular_line)
        
        Parameters:
            None
        
        Returns:
            None
        """
        # Motor speed state
        self.left_wheel_speed = self.BASE_SPEED  # Current left motor speed
        self.right_wheel_speed = self.BASE_SPEED  # Current right motor speed
        self.current_speed = self.BASE_SPEED  # Baseline speed for proportional control adjustments

        # Direction and mission state
        self.direction_flag = "forward"  # Global movement direction: "forward" or "reverse"
        self.found_box = False  # Flag indicating box detected during search
        
        # Sensor state memory for edge detection and control decisions
        self.sensor_left_prev = 0  # Previous mid-left sensor state 
        self.sensor_right_prev = 0  # Previous mid-right sensor state 
        self.on_perpendicular_line = False  # Debounce flag to prevent counting same line multiple times

        # Navigation and control state
        self.count_lines = 0  # Number of perpendicular lines crossed (navigation waypoint counter)
        self.is_running = False  # Robot active state (toggled by button, starts False)
        self.last_button_time = 0  # Timestamp of last button press (milliseconds, for debouncing)
        self.list_of_bays = ["GREEN", "RED", "BLUE", "YELLOW"]  # Available sorting destinations

    def _actuator_initial_position(self):
        """Move linear actuator to initial position for navigation.
        
        Extends actuator fully, then retracts to clear ground and boxes.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.linear_actuator.set("extend", 90)
        sleep(8)
        self.linear_actuator.set("retract", 90)
        sleep(4.9)
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
        sleep(8)
        self.linear_actuator.off()

    def motor_turn_right(self):
        """Execute a right turn maneuver on the line.
        
        Turn sequence:
        1. Wait until both mid sensors detect line (perpendicular intersection)
        2. Move straight briefly (0.1s) to center on intersection
        3. Stop and pause (0.2s) for stability
        4. Execute turn: left wheel MAX_SPEED forward, right wheel MIN_SPEED backward
        5. Turn for 1.22 seconds (calibrated for ~90° turn)
        6. Wait for mid-left sensor to re-detect line (turn complete)
        
        Used for: Navigating right turns at intersections during path following
        
        Parameters:
            None
        
        Returns:
            None
        """
        # Wait until both mid sensors detect line (at perpendicular intersection)
        while(self.signal_mid_left.value() == 1 and self.signal_mid_right.value() == 1):
            pass
        # Move forward briefly to center robot on intersection before turning
        self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED)
        sleep(0.1)  # 100ms forward movement
        self.motors_off()
        sleep(0.2)  # 200ms pause for stability
        # Execute turn: left wheel forward, right wheel backward (rotate clockwise)
        left_wheel_speed = self.MAX_SPEED
        right_wheel_speed = -1 * self.MIN_SPEED
        self.motor_turn(left_wheel_speed, right_wheel_speed)
        sleep(1.22)  # Calibrated duration for ~90° turn at these speeds
        # Wait until mid-left sensor re-detects line (turn complete indicator)
        while (self.signal_mid_left.value() == 0):
            pass

    def motor_turn_left(self):
        """Execute a left turn maneuver on the line.
        
        Turn sequence:
        1. Wait until both mid sensors detect line (perpendicular intersection)
        2. Stop and pause (0.2s) for stability
        3. Execute turn: right wheel MAX_SPEED forward, left wheel MIN_SPEED backward
        4. Turn for 1.204 seconds (calibrated for ~90° turn)
        5. Wait for mid-right sensor to re-detect line (turn complete)
        
        Used for: Navigating left turns at intersections during path following
        
        Parameters:
            None
        
        Returns:
            None
        """
        while(self.signal_mid_left == 1 and self.signal_mid_right == 1):
            pass
        self.motors_off()
        sleep(0.2)
        self.left_wheel_speed = -1 * self.MIN_SPEED
        self.right_wheel_speed = self.MAX_SPEED
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)
        sleep(1.204)
        while (self.signal_mid_right.value() == 0):
            pass

    def motor_turn_right_bay(self):
        """Execute a right turn specifically for entering bays.
        
        Bay entry turn sequence:
        1. Move straight at BASE_SPEED for 0.5s (position past intersection)
        2. Execute sharp turn: both wheels at MAX_SPEED in opposite directions
        3. Turn for 0.85s
        4. Wait until either mid sensor detects bay line
        
        Difference from motor_turn_right:
        - Faster turn speed (MAX vs MIN on reversing wheel)
        - Shorter turn duration (0.85s vs 1.22s)
        - Designed for tighter bay entrances
        
        Used for: Entering bays B1 and A0 from main line
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED, delay = 0.5)
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed = -self.MAX_SPEED
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)
        sleep(0.85)
        while (self.signal_mid_left.value() == 0 and self.signal_mid_right.value() == 0):
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
        self.motors_off()
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed = self.BASE_SPEED
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)
        sleep(0.1)
        while (self.signal_mid_right.value() == 0):
            pass

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
        self.direction_flag = "forward"
        self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED, delay = 0.45)
        right_wheel_speed = -self.MAX_SPEED
        left_wheel_speed = self.MAX_SPEED
        self.motor_turn(left_wheel_speed, right_wheel_speed)
        sleep(0.85)
        while (self.signal_mid_left.value() == 0 ):
            pass
        self.motors_off()
        

    def _reverse_on_the_spot(self):
        """Perform a 180-degree turn in place by reversing motors in opposite directions.
        
        Rotates robot on its axis for approximately 2 seconds to face opposite direction.
        Used after completing box placement to return to navigation mode.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.direction_flag = "forward"
        left_wheel = -self.MAX_SPEED
        right_wheel = self.MAX_SPEED
        self.disable_interrupts()
        self.motor_turn(left_wheel,right_wheel)
        sleep(1.659)
        while (self.signal_mid_left.value() == 0 and self.signal_mid_right.value() == 0):
            pass
        self.setup_interrupts()
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
        # Update baseline speed only when both wheels equal (straight motion, not turning)
        # This preserves the reference speed for proportional control calculations
        if speed_left == speed_right:
            self.current_speed = speed_left
        
        # Apply movement based on global direction flag
        if self.direction_flag == "forward":
            self.motor_left.Forward(speed_left)
            self.motor_right.Forward(speed_right)
        elif self.direction_flag == "reverse":
            self.motor_left.Reverse(speed_left)
            self.motor_right.Reverse(speed_right)
        # Optional blocking delay (used for timed maneuvers)
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
        # Case 1: Both speeds positive → straight motion (use direction_flag)
        if speed_left >= 0 and speed_right >= 0:
            self.motor_go_straight(speed_left, speed_right)
        # Case 2: Left positive, right negative → turn right (rotate clockwise)
        elif speed_left >= 0 and speed_right <= 0:
            speed_right = abs(speed_right)  # Convert to positive for PWM
            self.motor_left.Forward(speed_left)  # Left wheel pushes forward
            self.motor_right.Reverse(speed_right)  # Right wheel pulls backward
            # Don't update current_speed during turns (differential speeds, not baseline)
        # Case 3: Left negative, right positive → turn left (rotate counter-clockwise)
        elif speed_left <= 0 and speed_right >= 0:
            speed_left = abs(speed_left)  # Convert to positive for PWM
            self.motor_left.Reverse(speed_left)  # Left wheel pulls backward
            self.motor_right.Forward(speed_right)  # Right wheel pushes forward
            # Don't update current_speed during turns (differential speeds, not baseline)

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
            self.disable_interrupts()

    def setup_interrupts(self):
        """Set up interrupt handlers for all sensors.
        
        Parameters:
            None
        
        Returns:
            None
        """

        # Create closure wrappers to capture self (required for instance method callbacks)
        def line_handler(p):
            self.line_follower(p)  # Real-time line following control

        def count_handler(p):
            self.line_counter(p)  # Navigation waypoint tracking

        # Attach interrupt handlers to mid sensors for line following (both edges for immediate response)
        # Rising edge: sensor detects line (white→black transition)
        # Falling edge: sensor loses line (black→white transition)
        self.signal_mid_right.irq(handler=line_handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self.signal_mid_left.irq(handler=line_handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)

        # Attach interrupt handlers to far sensors for line counting (edge-triggered state machine)
        self.signal_far_left.irq(handler=count_handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self.signal_far_right.irq(handler=count_handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)

        # Enable button interrupt (falling edge = button press with pull-down resistor)
        self.button.irq(handler=self.button_handler, trigger=Pin.IRQ_FALLING)
        self.on_perpendicular_line = False  # Reset line detection state

    def disable_interrupts(self):
        """Disable interrupt handlers for all sensors.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.signal_mid_right.irq(handler=None)
        self.signal_mid_left.irq(handler=None)
        self.signal_far_left.irq(handler=None)
        self.signal_far_right.irq(handler=None)
        self.sensor_left_prev = 0
        self.sensor_right_prev = 0
        self.on_perpendicular_line = True

    def _path_algorithm(self, current_position, desired_position):
        """Navigate robot from current position to desired position using predefined paths.
        
        Position format:
        - "G0": Ground/start position
        - "B0X" / "B1X": Bay B, side 0/1, line X (1-6)
        - "A0X" / "A1X": Bay A, side 0/1, line X (1-6)
        - Colour names: Sorting bay destinations
        
        Executes the path by calling appropriate Path class methods and executing turns.
        
        Parameters:
            current_position (str): Starting position (format: "G0", "B01"-"B16", "A01"-"A16")
            desired_position (str): Target position (same format, or colour bay: "GREEN", "RED", "BLUE", "YELLOW")
        
        Returns:
            None
        """

        # ========== GROUND (G0) TO BAYS ==========
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

        # ========== BAY B0 PATHS ==========
        # B0 to ground
        if (current_position.startswith("B0") and desired_position.startswith("G")):
            turns, lines = Path._path_B0_to_G(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        # B0 to B1
        if (current_position.startswith("B0") and desired_position.startswith("B1")):
            turns, lines = Path._path_B0_to_B1(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        # B0 to colour bays
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

        # ========== BAY B1 PATHS ==========
        # B1 to ground
        if (current_position.startswith("B1") and desired_position.startswith("G")):
            turns, lines = Path._path_B1_to_G(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        # B1 to colour bays
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

        # ========== BAY A0 PATHS ==========
        # A0 to ground
        if (current_position.startswith("A0") and desired_position.startswith("G")):
            turns, lines = Path._path_A0_to_G(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        # A0 to A1
        if (current_position.startswith("A0") and desired_position.startswith("A1")):
            turns, lines = Path._path_A0_to_A1(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        # A0 to colour bays
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

        # ========== BAY A1 PATHS ==========
        # A1 to ground
        if (current_position.startswith("A1") and desired_position.startswith("G")):
            turns, lines = Path._path_A1_to_G(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        # A1 to colour bays
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
        
        # ========== GREEN BAY RETURN PATHS ==========
        if (current_position.startswith("GREEN") and desired_position.startswith("B0")):
            turns, lines = Path._path_Green_bay_to_B0(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position.startswith("GREEN") and desired_position.startswith("B1")):
            turns, lines = Path._path_Green_bay_to_B1(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position.startswith("GREEN") and desired_position.startswith("A0")):
            turns, lines = Path._path_Green_bay_to_A0(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position.startswith("GREEN") and desired_position.startswith("A1")):
            turns, lines = Path._path_Green_bay_to_A1(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        
        # ========== RED BAY RETURN PATHS ==========
        if (current_position == "RED" and desired_position.startswith("B0")):
            turns, lines = Path._path_Red_bay_to_B0(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position == "RED" and desired_position.startswith("B1")):
            turns, lines = Path._path_Red_bay_to_B1(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position == "RED" and desired_position.startswith("A0")):
            turns, lines = Path._path_Red_bay_to_A0(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position == "RED" and desired_position.startswith("A1")):
            turns, lines = Path._path_Red_bay_to_A1(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        
        # ========== BLUE BAY RETURN PATHS ==========
        if (current_position == "BLUE" and desired_position.startswith("B0")):
            turns, lines = Path._path_Blue_bay_to_B0(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position == "BLUE" and desired_position.startswith("B1")):
            turns, lines = Path._path_Blue_bay_to_B1(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position == "BLUE" and desired_position.startswith("A0")):
            turns, lines = Path._path_Blue_bay_to_A0(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position == "BLUE" and desired_position.startswith("A1")):
            turns, lines = Path._path_Blue_bay_to_A1(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)
        
        # ========== YELLOW BAY RETURN PATHS ==========
        if (current_position == "YELLOW" and desired_position.startswith("B0")):
            turns, lines = Path._path_Yellow_bay_to_B0(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position == "YELLOW" and desired_position.startswith("B1")):
            turns, lines = Path._path_Yellow_bay_to_B1(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position == "YELLOW" and desired_position.startswith("A0")):
            turns, lines = Path._path_Yellow_bay_to_A0(current_position, desired_position)
            self._handle_identifying_cases(turns, lines)

        if (current_position == "YELLOW" and desired_position.startswith("A1")):
            turns, lines = Path._path_Yellow_bay_to_A1(current_position, desired_position)
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
        
        State machine:
        - Rising edge (0→1): When either far sensor detects line AND not already on line:
          * Increments count_lines (used for navigation waypoints)
          * Sets on_perpendicular_line flag to prevent double-counting
        - Falling edge (1→0): When both far sensors off line AND flag is set:
          * Resets on_perpendicular_line flag
          * Ready to count next line crossing
        
        Edge detection prevents:
        - Double-counting when transitioning across wide lines
        - Noise from sensor bouncing
        - Counting the same line multiple times
        
        Used for: Navigation by counting intersections to reach target bays
        
        Parameters:
            p (Pin): Pin object that triggered the interrupt (signal_far_left or signal_far_right)
        
        Returns:
            None
        """
        # Ignore interrupts when robot is stopped (button not pressed)
        if not self.is_running:
            return

        # Sample current far sensor states
        sensor_far_left = self.signal_far_left.value()
        sensor_far_right = self.signal_far_right.value()

        # STATE TRANSITION: Off-line → On-line (rising edge)
        # When either far sensor detects perpendicular line AND we're not already counting
        if (sensor_far_left == 1 or sensor_far_right == 1) and not self.on_perpendicular_line:
            self.count_lines += 1  # Increment waypoint counter
            self.on_perpendicular_line = True  # Set flag to prevent double-counting during crossing
            print("Lines detected:", self.count_lines)

        # STATE TRANSITION: On-line → Off-line (falling edge)
        # When both far sensors clear the line AND flag is set
        elif sensor_far_left == 0 and sensor_far_right == 0 and self.on_perpendicular_line:
            self.on_perpendicular_line = False  # Reset flag, ready for next line
            print("Left perpendicular line")

    def line_follower(self, p):
        """Interrupt handler for proportional line following control.
        
        Implements real-time line following using mid sensors with proportional correction.
        
        Control logic:
        1. Right sensor HIGH, left LOW: Robot drifting left
           → Increase left speed, decrease right speed (turn right to correct)
        2. Left sensor HIGH, right LOW: Robot drifting right
           → Increase right speed, decrease left speed (turn left to correct)
        3. Both sensors HIGH: On line (perpendicular crossing or wide line)
           → Equal speeds, go straight
        4. Both sensors LOW: Completely off line
           → Use previous sensor state for aggressive correction
           → If last saw left sensor: turn hard left
           → If last saw right sensor: turn hard right
        
        Speed adjustments:
        - Normal correction: ±SPEED_ADJUSTMENT from current_speed
        - Aggressive correction: MIN_SPEED to MAX_SPEED differential
        - Clamped to [MIN_SPEED, MAX_SPEED] range
        
        Advantages:
        - Interrupt-driven: sub-millisecond response time
        - Proportional control: smooth corrections
        - State memory: handles brief sensor dropouts
        
        Parameters:
            p (Pin): Pin object that triggered the interrupt (signal_mid_left or signal_mid_right)
        
        Returns:
            None
        """

        if not self.is_running:
            return

        # Read current sensor values (only mid sensors needed for line following)
        sensor_mid_left = self.signal_mid_left.value()
        sensor_mid_right = self.signal_mid_right.value()

        # CASE 1: Right sensor HIGH, left LOW → drifting LEFT of line
        if sensor_mid_right == 1 and sensor_mid_left == 0:
            # Correction: speed up left wheel, slow down right (steer right back to center)
            self.left_wheel_speed = min(self.current_speed + self.SPEED_ADJUSTMENT, self.MAX_SPEED)
            self.right_wheel_speed = max(self.current_speed - self.SPEED_ADJUSTMENT, self.MIN_SPEED)

        # CASE 2: Left sensor HIGH, right LOW → drifting RIGHT of line
        elif sensor_mid_left == 1 and sensor_mid_right == 0:
            # Correction: speed up right wheel, slow down left (steer left back to center)
            self.left_wheel_speed = max(self.current_speed - self.SPEED_ADJUSTMENT, self.MIN_SPEED)
            self.right_wheel_speed = min(self.current_speed + self.SPEED_ADJUSTMENT, self.MAX_SPEED)

        # CASE 3: Both sensors HIGH → on perpendicular line or centered on thick line
        elif sensor_mid_left == 1 and sensor_mid_right == 1:
            # No correction needed, maintain straight trajectory
            self.left_wheel_speed = self.current_speed
            self.right_wheel_speed = self.current_speed

        # CASE 4: Both sensors LOW → completely off line (recovery mode)
        else:
            # Use sensor state memory to determine which way to correct
            if self.sensor_left_prev == 1 and self.sensor_right_prev == 0:
                # Was last on left side → turn left aggressively to find line
                self.left_wheel_speed = self.MIN_SPEED
                self.right_wheel_speed = self.MAX_SPEED
            elif self.sensor_right_prev == 1 and self.sensor_left_prev == 0:
                # Was last on right side → turn right aggressively to find line
                self.left_wheel_speed = self.MAX_SPEED
                self.right_wheel_speed = self.MIN_SPEED
            else:
                # No previous state or both were on → assume centered, go straight
                self.left_wheel_speed = self.current_speed
                self.right_wheel_speed = self.current_speed

        # Apply calculated motor speeds immediately (interrupt-driven = zero latency)
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)

        # Store current sensor states for next interrupt (enables recovery from dropout)
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
        # Restart the interrupts after the turn and maintain current speed
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
        distance = self.vl53l0.read()  # Blocking read from sensor
        sleep(0.1)  # Wait for measurement to complete (sensor timing requirement)
        return distance if distance is not None else 9999  # Return sentinel value 9999 on error to prevent false box detection

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
        """Navigate to a bay, search for a box, identify its colour, and pick it up.
        
        Complete box pickup sequence:
        1. Navigate from start position to destination bay using path algorithm
        2. Move slowly along main line adjacent to bay
        3. At each perpendicular line, check for box presence:
           - Uses is_box() with appropriate distance sensor (ToF or URM09)
           - Checks during line crossing for accurate positioning
        4. If no box found:
           - Increment line counter and continue to next bay line
           - If reached end of bay (line 7), switch to opposite bay side
           - If all bays searched, return to ground ("G0", "G0")
        5. When box detected:
           - Turn into bay (_before_pick_box)
           - Approach box to optimal distance
           - Identify colour (_do_pick_box with up to 3 retry attempts)
           - Activate actuator to secure box
           - Return to main line (_after_pick_box)
        6. Determine sorting destination based on detected colour
        
        Bay search order by side:
        - B0: lines 1→6, then switch to B1 at line 16
        - B1: lines 16→11, then switch to A1
        - A1: lines 16→11, then switch to A0 at line 6
        - A0: lines 6→1, then return to G0 if no boxes found
        
        Parameters:
            start (str): Starting position (default: "G0" - ground position)
            destination (str): Initial bay to search (default: "A01")
        
        Returns:
            tuple: (current_position, destination_bay)
                - current_position (str): Bay where box was found (e.g., "B03", or "G0" if none found)
                - destination_bay (str): Colour destination ("GREEN", "RED", "BLUE", "YELLOW", or "G0" if failed)
        """
        self._path_algorithm(start, destination)
        temp_line_counter = int(destination[2])
        self.found_box = False

        current_bay = destination[0:2]  # It doesn`t matter the specific bay number, just the side
        self.direction_flag = "forward"
        self.motor_go_straight(self.MIN_SPEED, self.MIN_SPEED)
        
        while (not self.found_box):
            # Wait until we reach a perpendicular line, then check for box
            
            # Now we're on a line - check for box
            if self.is_box(current_bay):
                print("Found a box!")
                # Box found - is_box() sets self.found_box = True
                break
            else:
                # No box found on this line, increment counter
                self.moving_between_lines(current_bay)
                temp_line_counter += 1
                
                if temp_line_counter > 7:
                    self.count_lines = 0
                    current_bay = self._go_to_next_bay(current_bay)
                    temp_line_counter = 1
                    if current_bay == "G0":
                        print("No boxes found in any bay.")
                        return "G0", "G0"

        current_position = current_bay + str(temp_line_counter)
        print(current_position, "type is", type(current_position))
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
        self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED)
        if position == "B0" or position == "A1":
            while (self.signal_far_left.value() == 0):
                pass
        elif position == "B1" or position == "A0":
            while (self.signal_far_right.value() == 0):
                pass

    def is_box(self, position, timeout_ms=5000) -> bool:
        """Check for box presence while crossing a perpendicular line using distance sensors.
        
        Detection strategy:
        - For B0/A1 (left side): Uses TMF8x01 ToF sensor (forward-facing)
          * Monitors far_left sensor during line crossing
          * Detects boxes within 300mm
          * Returns to BASE_SPEED after crossing if no box found
        - For B1/A0 (right side): Uses URM09 ultrasonic sensor
          * Monitors far_right sensor during line crossing
          * Detects boxes within 25cm (250mm)
          * Slower approach speed (MIN_SPEED + 15) for better ultrasonic accuracy
        
        Process:
        1. Wait while appropriate far sensor is HIGH (on perpendicular line)
        2. During crossing, continuously check distance sensor
        3. If box detected within threshold, set found_box flag and return True
        4. If sensor goes LOW (left line) without detection, return False
        5. Timeout after 5 seconds to prevent infinite loops
        
        Parameters:
            position (str): Current bay position ("B0", "B1", "A0", or "A1")
            timeout_ms (int): Maximum time to wait for sensor to go low (default: 5000ms)
        
        Returns:
            bool: True if box detected, False otherwise. Also sets self.found_box flag.
        """
        start_time = ticks_ms()
        self.motors_off()
        if position == "B0" or position == "A1":
            while (self.signal_far_left.value() == 1):
                self.motor_go_straight(self.MIN_SPEED, self.MIN_SPEED, delay = 0.02)
                if ticks_diff(ticks_ms(), start_time) > timeout_ms:
                    print("Timeout waiting for far_left sensor in is_box()")
                    # self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED, delay = 0.1)
                    return False
                distance = self._calculate_distance_tof()
                if distance is not None and distance < 300:
                    self.found_box = True
                    return True
                sleep(0.05)
            self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED)
            return False
        elif position == "B1" or position == "A0":
            while (self.signal_far_right.value() == 1):
                self.motor_go_straight(self.MIN_SPEED+15, self.MIN_SPEED+15,delay = 0.5)
                if ticks_diff(ticks_ms(), start_time) > timeout_ms:
                    print("Timeout waiting for far_right sensor in is_box()")
                    return False
                print(self._calculate_distance_URM09())
                if (self._calculate_distance_URM09() <= 25):
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
            self._execute_turn(self.motor_turn_left, f"before_pick_box_{current_position}")
            self.motors_off()

        elif current_position[0:2] == "A0" or current_position[0:2] == "B1":
            self._execute_turn(self.motor_turn_right_bay, f"before_pick_box_{current_position}")
            self.motors_off()

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
            self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED, delay=0.5)
            self._execute_turn(self.motor_turn_right_back, "after_pick_box")
            self.direction_flag = "forward"
            self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED)
            while (self.signal_far_right.value() == 1):
                pass
        elif current_position[0:2] == "B1" or current_position[0:2] == "A0":
            self.direction_flag = "reverse"
            self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED)
            while (self.signal_far_right.value() == 0 and self.signal_far_left.value() == 0):
                pass
            self._execute_turn(self.motor_turn_left_back, "after_pick_box")
            self.motors_off()
            sleep(0.1)
            self.direction_flag = "forward"
            self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED)
        # self.motors_off()
        # 

    def _reinit_vl53l0(self):
        """Re-initialise VL53L0X sensor after power conflict.
        
        Fully re-creates the sensor object to recover from I2C errors
        caused by current draw conflicts with colour sensor.
        
        Parameters:
            None
        
        Returns:
            None
        """
        print("Re-initialising VL53L0X sensor...")
        sleep(0.5)  # Allow current to settle before re-init
        i2c_bus_vl5310 = I2C(0, sda=Pin(self.VL53L0X_SDA_PIN), scl=Pin(self.VL53L0X_SCL_PIN))
        self.vl53l0 = VL53L0X(i2c_bus_vl5310)
        self.vl53l0.set_Vcsel_pulse_period(self.vl53l0.vcsel_period_type[0], 18)
        self.vl53l0.set_Vcsel_pulse_period(self.vl53l0.vcsel_period_type[1], 14)
        self.vl53l0.start()
        self.vl53l0_running = True
        sleep(0.1)  # Allow sensor to stabilise

    def _start_vl53l0(self):
        """Start VL53L0X distance sensor if not already running.
        
        Power management context:
        - VL53L0X and colour sensor share power rail and have high current draw
        - Cannot operate simultaneously without brown-out issues
        - This method safely restarts VL53L0X after colour sensor usage
        
        Recovery mechanism:
        - If start() fails with OSError (common after colour sensor use)
        - Performs full re-initialisation with I2C bus reset
        - Reconfigures VCSEL pulse periods for optimal performance
        
        Used before: Box approach sequences, distance measurements during navigation
        
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
                sleep(0.1)  # Allow sensor to stabilise
            except OSError as e:
                print(f"VL53L0X start failed ({e}), re-initialising...")
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
        """Initialise colour sensor on demand.
        
        Lazy initialisation to avoid current draw issues at startup.
        Must stop VL53L0X before using colour sensor.
        
        Parameters:
            None
        
        Returns:
            None
        """
        if self.colour_sensor is None:
            print("Initialising colour sensor...")
            self.colour_sensor = ColourSensor()
            sleep(0.2)  # Allow sensor to stabilise

    def _do_pick_box(self):
        """Approach box, identify colour, and activate pickup mechanism.
        
        Multi-stage pickup with retry logic (up to 3 attempts):
        
        Attempt sequence:
        1. Start VL53L0X sensor (power management)
        2. Approach box using _approach_box() (target: 62mm distance)
        3. Stop VL53L0X to prevent current conflict
        4. Initialise colour sensor (lazy init if needed)
        5. Detect box colour using colour sensor
        6. If successful:
           - Don't restart VL53L0X (actuator doesn't need it)
           - Retract actuator for 1.5s to secure box
           - Re-enable interrupts for navigation
        7. If failed:
           - Re-initialise VL53L0X (full reset after colour sensor use)
           - Re-enable interrupts
           - Reverse for 1 second
           - Increment attempt counter and retry
        
        Why 3 attempts?
        - Colour sensor needs precise distance and lighting
        - Initial approach may be sub-optimal
        - Reversing and re-approaching improves positioning
        
        Failure modes:
        - After 3 failed attempts: abort pickup, return None
        - Robot will reverse out of bay without box
        
        Parameters:
            None
        
        Returns:
            str: Detected colour name ("GREEN", "RED", "BLUE", "YELLOW") or None on failure
        """
        colour_detected = False
        number_of_attempts = 0
        identified_colour = None

        # Identify the colour of the box before picking it up
        while not colour_detected and number_of_attempts < 3:
            # Ensure VL53L0X is running for approach
            self._start_vl53l0()
            approached_box = self._approach_box()
            
            # Stop VL53L0X before using colour sensor to avoid current conflicts
            self._stop_vl53l0()
            
            # Initialise colour sensor if needed (lazy init)
            identified_colour = None
            if approached_box:
                print("Approached box successfully.")
                self.disable_interrupts()
                self.motors_off()
                self._init_colour_sensor()
                identified_colour = self.colour_sensor.detect_colour()
                print("Identified colour:", identified_colour)

            if identified_colour is not None:
                colour_detected = True
                # Don't restart VL53L0X here - actuator operation doesn't need it
                # It will be re-initialised later when navigation resumes
                self.linear_actuator.set("retract", 90)
                sleep(1.5)
                self.linear_actuator.off()
                self.setup_interrupts()
            else:
                print("Couldn't identify colour, redoing the pick up.")
                # Re-initialise VL53L0X for reversing and retry (full re-init after colour sensor use)
                self._reinit_vl53l0()
                self.direction_flag = "reverse"
                self.setup_interrupts()
                self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED, delay=1)
                self.motors_off()
                number_of_attempts += 1

        if not colour_detected:
            print("Failed to identify colour after 3 attempts, aborting pick up.")
            # Re-initialise VL53L0X for further operations
            self._reinit_vl53l0()
            self.motors_off()
            return None

        return identified_colour

    def _approach_box(self):
        """Move slowly toward box until reaching optimal distance for colour detection.
        
        Uses VL53L0X distance sensor to stop at 60mm from box,
        ideal distance for colour sensor accuracy.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.direction_flag = "forward"
        self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED)  # Move slowly toward box
        start_time = ticks_ms()  # Record start time for timeout protection
        # Approach until within 62mm (optimal distance for colour sensor accuracy)
        # Distance threshold based on colour sensor focal length and lighting requirements
        while (self._calculate_distance() > 62):
            # Timeout protection: prevent infinite loop if box moves or sensor fails
            if ticks_diff(ticks_ms(), start_time) > 1500:  # 1.5 second timeout
                print("Timeout approaching box")
                self.motors_off()
                return False  # Approach failed
            pass  # Continue approaching (VL53L0X reads continuously)
        self.motors_off()  # Stop at optimal distance
        return True  # Approach successful

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
        self.motors_off()  # Stop at bay entrance
        self._actuator_final_position()  # Extend actuator fully to release box (8 seconds)
        self.linear_actuator.off()  # Turn off actuator motor after extension
        # Reverse out of bay to avoid collision with placed box
        self.direction_flag = "reverse"
        self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED, delay=0.6)  # Reverse for 600ms
        self.motors_off()  # Stop for actuator retraction
        # Retract actuator back to transport position
        self.linear_actuator.set("retract", 90)  # Full power retraction
        sleep(4.9)  # Wait 4.9s for full retraction (calibrated timing)
        self.linear_actuator.off()  # Turn off actuator motor
        self._reverse_on_the_spot()  # Perform 180° turn to face forward again


    def run(self):
        """Main control loop - executes complete autonomous mission.
        
        Full mission sequence:
        
        Initialisation:
        1. Wait for button press (is_running flag)
        2. Position actuator to initial state (raised, clear of ground)
        3. Turn on yellow LED to indicate active state
        
        First box cycle:
        4. Search for box starting from G0 → A01
        5. Pick up box and identify colour
        6. Navigate from pickup location to appropriate colour bay
        7. Place box in sorting bay:
           - Extend actuator fully (release box)
           - Reverse out of bay
           - Retract actuator (return to transport position)
           - Perform 180° turn to face forward
        
        Second box cycle:
        8. Re-initialise VL53L0X (after power conflicts during first cycle)
        9. Search for second box starting from current colour bay → A01
        10. Pick up and place second box (repeat steps 5-7)
        
        Mission continues until:
        - Button pressed again (manual stop)
        - Exception raised (caught by main try-except)
        - No more boxes found in any bay
        
        Note: Current implementation handles 2 boxes. Extend with additional
        pick_box() calls for multi-box competition runs.
        
        Parameters:
            None
        
        Returns:
            None
        """
        # PHASE 1: Wait for button press to start mission
        print("Robot ready. Press button to start...")
        while not self.is_running:  # is_running toggled by button interrupt
            sleep(0.1)  # Polling loop with 100ms interval

        # PHASE 2: Initialise actuator and indicate running state
        print("Start line follower")
        self._actuator_initial_position()  # Raise actuator to navigation height
        self.yellow_led.value(0)  # Turn LED on (LOW = on) to indicate active
        
        # PHASE 3: First box cycle - search, pick, and place
        # Search for first box starting from ground position to bay A01
        current_position, next_position = self.pick_box(start="G0", destination="A01")
        self.motors_off()  # Stop after pickup
        sleep(0.1)  # Brief pause for stability
        # Navigate from pickup location to colour-matched sorting bay
        self.direction_flag = "forward"
        self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED)  # Start line following
        self._path_algorithm(current_position, next_position)  # Execute path to sorting bay
        self._place_box()  # Release box into sorting bay
        
        # PHASE 4: Prepare for second box cycle
        self.motors_off()  # Ensure stopped after placement
        # Critical: Re-initialise VL53L0X after colour sensor usage (I2C bus may be corrupted)
        self._reinit_vl53l0()
        sleep(0.1)  # Allow sensor to stabilise
        
        # PHASE 5: Second box cycle - search from current bay, pick, and place
        self.direction_flag = "forward"
        self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED)  # Resume line following
        # Search for second box starting from current sorting bay location
        current_position, next_position = self.pick_box(start=next_position, destination="A01")
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
