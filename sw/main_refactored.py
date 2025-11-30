from machine import Pin, I2C, ADC
from utime import sleep, ticks_ms, ticks_diff
from sw.test_motor import Motor
from sw.path import Path
from sw.test_linear_actuator import Actuator
from sw.libs.VL53L0X.VL53L0X import VL53L0X
from sw.libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8801, DFRobot_TMF8701 #Second distance sensor
# from sw.libs.DFRobot_URM09.DFRobot_URM09 import DFRobot_URM09
from sw.libs.tcs3472_micropython.tcs3472 import tcs3472
from sw.colour_sensor import *


class LineFollowerRobot:
    """
    Line-following robot with integrated sensor processing and motor control.
    Encapsulates all state and behavior for autonomous line following.
    """
    
    # Configuration constants
    BASE_SPEED = 60
    SPEED_ADJUSTMENT = 8
    MIN_SPEED = 30
    MAX_SPEED = 80

    SEARCH_LIST = [
        "B01", "B02", "B03", "B04", "B05", "B06", 
        "B16", "B15", "B14", "B13", "B12", "B11", 
        "A11", "A12", "A13", "A14", "A15", "A16",
        "A06", "A05", "A04", "A03", "A02", "A01"
    ]
    
    # Proportional control parameters
    KP = 4  # Proportional gain for steering correction

    # Pin configurations
    MID_RIGHT_PIN = 26
    MID_LEFT_PIN = 27
    FAR_RIGHT_PIN = 22
    FAR_LEFT_PIN = 28
    SDA_PIN = 20
    SCL_PIN = 21
    YELLOW_LED = 14

    TMF8X01_SDA_PIN = 18
    TMF8X01_SCL_PIN = 19

    BUTTON_PIN = 17
    DEBOUNCE_MS = 200
    
    def __init__(self):
        """Initialize robot hardware and state"""
        # Initialize hardware
        self._init_sensors()
        self._init_motors()
        
        # Initialize state variables
        self._init_state()
        
        # Start distance sensor
        self.vl53l0.start()
        self.tof.begin()
        self.tof.start_measurement(calib_m = self.tof.eMODE_NO_CALIB, mode = self.tof.ePROXIMITY)
    
    def _init_sensors(self):
        """Initialize all sensor pins"""
        self.signal_mid_right = Pin(self.MID_RIGHT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_mid_left = Pin(self.MID_LEFT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_far_right = Pin(self.FAR_RIGHT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_far_left = Pin(self.FAR_LEFT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.yellow_led = Pin(self.YELLOW_LED, Pin.OUT)
        self.yellow_led.value(1)
        
        # config I2C Bus
        i2c_bus_vl5310 = I2C(0, sda=Pin(self.SDA_PIN), scl=Pin(self.SCL_PIN))  # I2C0 on GP8 & GP9
        i2c_bus_tmf8x01 = I2C(1, sda=Pin(self.TMF8X01_SDA_PIN), scl=Pin(self.TMF8X01_SCL_PIN), freq=100000)  # I2C1 on GP2 & GP3
        
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
        
        #Setup colour sensor:
        self.colour_sensor = ColourSensor()

        #Setup the URM09 Ultrasonic sensor:
        self.Max_range = 500
        self.adc_pin = ADC(Pin(26))
        self.ADC_Resolution = 65535


    
    def _init_motors(self):
        """Initialize motor controllers"""
        self.motor_left = Motor(dirPin=7, PWMPin=6)   # Motor left is controlled from Motor Driv2 #2
        self.motor_right = Motor(dirPin=4, PWMPin=5)  # Motor right is controlled from Motor Driv2 #3
        self.linear_actuator = Actuator(dirPin=0, PWMPin=1)  

    def _init_state(self):
        """Initialize all state variables"""
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
        self.is_running = False  # Wait for button press to start
        self.last_button_time = 0
        self.list_of_bays = [ "GREEN", "RED", "BLUE", "YELLOW" ]
        
        self._actuator_initial_position()

    def _actuator_initial_position(self):
        self.linear_actuator.set("extend", 90)
        sleep(4)
        self.linear_actuator.set("retract", 90)
        sleep(2.5)
        self.linear_actuator.off()

    def _actuator_final_position(self):
        self.linear_actuator.set("extend", 90)
        sleep(4)
        self.linear_actuator.off()
    
    def motor_turn_right(self):
        """Configure motor speeds for right turn"""
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed = -1 * self.MIN_SPEED
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)
        sleep(1.2)
        while (self.signal_mid_left.value()==0):
            pass
        self.right_wheel_speed = self.MAX_SPEED
        self.left_wheel_speed = self.BASE_SPEED
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        sleep(0.2)
        while (self.signal_mid_right.value()==0):
            pass


    def motor_turn_left(self):
        """Configure motor speeds for left turn"""
        self.left_wheel_speed = -1 *self.MIN_SPEED
        self.right_wheel_speed = self.MAX_SPEED
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)
        sleep(1.2)
        while (self.signal_mid_right.value()==0):
            pass
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed = self.BASE_SPEED
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        sleep(0.2)
        while (self.signal_mid_left.value()==0):
            pass


    def motor_turn_right_bay(self):
        """Configure motor speeds for right turn"""
        # while (self.signal_mid_right.value()== 0 or self.signal_mid_left.value()==0):
        #     pass
            #Turn right
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed = -self.BASE_SPEED
        self.direction_flag = "forward"
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)
        sleep(1.5)
        while(self.signal_mid_left.value()==0):
            pass
         #Adjust to line
        self.right_wheel_speed = self.MAX_SPEED
        self.left_wheel_speed = self.BASE_SPEED
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)
        sleep(0.35)
        while (self.signal_mid_right.value() == 0):
            pass

    def motor_turn_left_bay(self):
        self.right_wheel_speed = self.MAX_SPEED
        self.left_wheel_speed = -self.BASE_SPEED
        self.direction_flag = "forward"
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)
        sleep(1.5)
        while (self.signal_mid_left.value()==0):
            pass
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed = self.BASE_SPEED
        self.motor_turn(self.left_wheel_speed, self.right_wheel_speed)


    def motor_turn_right_back(self):
        """Configure motor speeds for right turn"""
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed = 0
        self.direction_flag = "reverse"
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        sleep(1.5)
        while(self.signal_mid_left.value()==0):
            pass
        self.right_wheel_speed = self.MAX_SPEED
        self.left_wheel_speed = 0
        self.direction_flag = "forward"
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        #Wait until the mid right sensor detects the line
        while(self.signal_mid_right.value()==0):
            pass


    def motor_turn_left_back(self):
        """Configure motor speeds for left turn"""
        self.left_wheel_speed = 0
        self.right_wheel_speed = self.MAX_SPEED
        self.direction_flag = "reverse"
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        sleep(1.5) # Usually about this delay is used for a turn
        while(self.signal_mid_right.value()==0): #Wait until the mid right sensor detects the line
            pass
        self.right_wheel_speed = 0
        self.left_wheel_speed = self.MAX_SPEED
        self.direction_flag = "forward"
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        #Wait until the mid left sensor detects the line
        while (self.signal_mid_left.value() == 0):
            pass

    def _reverse_on_the_spot(self):
        """Reverse the robot on the spot until both mid sensors detect the line"""
        self.direction_flag = "reverse"
        left_wheel = -self.MAX_SPEED
        right_wheel = self.MAX_SPEED 
        self.motor_turn(left_wheel, right_wheel)
        # Wait until both mid sensors detect the line
        #TODO: Develop what time it takes to reverse like that
        sleep(2)
        self.motors_off()



    def motor_go_straight(self, speed_left, speed_right, delay=0):
        """
        Execute straight movement using the global direction_flag
        
        Args:
            speed_left: Speed for left motor (0-100)
            speed_right: Speed for right motor (0-100)
            delay: Optional delay in seconds after moving
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
        if speed_left > 0 and speed_right > 0:
            self.motor_go_straight(speed_left, speed_right)
        elif speed_left > 0 and speed_right < 0:
            speed_right = abs(speed_right)
            self.motor_left.Forward(speed_left)
            self.motor_right.Reverse(speed_right)
        elif speed_left < 0 and speed_right > 0:
            # Turn left: left motor reverse, right motor forward
            speed_left = abs(speed_left)
            self.motor_left.Reverse(speed_left)
            self.motor_right.Forward(speed_right)


    def motors_off(self):
        """Turn off both motors"""
        self.motor_left.off()
        self.motor_right.off()
    
    def destroy(self):
        """Clean up resources - stop sensors and motors"""
        try:
            self.vl53l0.stop()
        except Exception as e:
            print(f"Error stopping VL53L0X: {e}")
        finally:
            self.motors_off()
    
    def setup_interrupts(self):
        """Set up interrupt handlers for all sensors"""
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
        """Disable interrupt handlers for all sensors"""
        self.signal_mid_right.irq(handler=None)
        self.signal_mid_left.irq(handler=None)
        self.signal_far_left.irq(handler=None)
        self.signal_far_right.irq(handler=None)
        self.prev_sensor_left = 0
        self.prev_sensor_right = 0


    def _path_algorithm(self, current_position, desired_position):
        '''
        Args:  "GB", "GG", "G0", "GY", "GR"
        "B01", "B02", "B03", "B04", "B05", "B06", 
        "B16", "B15", "B14", "B13", "B12", "B11", 
        "A11", "A12", "A13", "A14", "A15", "A16",
        "A06", "A05", "A04", "A03", "A02", "A01"

        return a list of commands to go from current to desired position
        '''

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
        # do NOT disable interrupt for button

    def button_handler(self, p):
        """
        Interrupt handler for button press.
        Toggles robot state between running and stopped.

        Args:
            p: Pin that triggered the interrupt
        """
        current_time = ticks_ms()

        print("================= Button pressed ==================")
        
        # Debounce: ignore if button pressed too soon after last press
        if ticks_diff(current_time, self.last_button_time) < self.DEBOUNCE_MS:
            return
        
        self.last_button_time = current_time
        
        # Toggle running state
        if self.is_running:
            # Stop the robot
            print("Button pressed - Stopping robot")
            self.is_running = False
            self.destroy()
        else:
            # Start the robot
            print("Button pressed - Starting robot")
            self.is_running = True
            # Reset state for fresh start
            self.count_lines = 0
            self.turning_case = 0

    def line_counter(self, p):
        """
        Interrupt handler for line counting (both rising and falling edges).
        Detects when robot crosses perpendicular lines using far sensors.
        - Rising edge (0→1): Increments line count
        - Falling edge (1→0): Resets the on_perpendicular_line flag when both sensors are off
        
        Args:
            p: Pin that triggered the interrupt
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
        """
        Interrupt handler for line sensors.
        Adjusts motor speeds based on sensor readings to keep robot on line.
        
        Args:
            p: Pin that triggered the interrupt
        """

        # print("mid left:", self.signal_mid_left.value(), " mid right:", self.signal_mid_right.value())

        if not self.is_running:
            return

        # Read current sensor values (only mid sensors needed for line following)
        sensor_mid_left = self.signal_mid_left.value()
        sensor_mid_right = self.signal_mid_right.value()
        print("mid left:", sensor_mid_left, " mid right:", sensor_mid_right)
        # Proportional control for line following
        # Calculate error: -1 (left of line), 0 (on line), +1 (right of line)
        error = 0
        
        if sensor_mid_right == 1 and sensor_mid_left == 0:
            # Right sensor on line - car drifting right
            self.left_wheel_speed = max(self.BASE_SPEED + self.SPEED_ADJUSTMENT, self.MIN_SPEED)
            self.right_wheel_speed = min(self.BASE_SPEED - self.SPEED_ADJUSTMENT, self.MAX_SPEED)
            
        elif sensor_mid_left == 1 and sensor_mid_right == 0:
            # Left sensor on line - car drifting left
            self.left_wheel_speed = min(self.BASE_SPEED - self.SPEED_ADJUSTMENT, self.MAX_SPEED)
            self.right_wheel_speed = max(self.BASE_SPEED + self.SPEED_ADJUSTMENT, self.MIN_SPEED)
            
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
        """
        Helper method to execute a turn and update state
        
        Args:
            turn_function: Method to call for turning (motor_turn_left or motor_turn_right)
            next_case: Next turning_case value
            case_name: Name of current case for logging
        """
        print(f"start: turning_case {case_name} turning")
        #Allow the turn to be done
        self.disable_interrupts()
        turn_function()
        self.count_lines = 0
        #Restrat the interrupts after the turn and reset the speed
        self.direction_flag = "forward"
        self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED)
        self.setup_interrupts()

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
    
    def _calculate_distance_tof(self) -> int:
        if(self.tof.is_data_ready() == True):
            return self.tof.get_distance_mm()

    def _calcultaed_distance_URM09(self) -> int:
        adc_value = self.adc_pin.read_u16()
        distance = (adc_value / self.ADC_Resolution)*self.Max_range
        if distance is not None:
            return distance
        else:
            print ("URM09 sensor error")
            return 8173

    def _handle_identifying_cases(self, turns, lines):
        """
        Navigate through a sequence of turns based on line counts.
        
        Args:
            turns: List of turn directions ("right_forward", "left_forward", "right_backward", "left_backward","straight" and "backward")
            lines: List of line counts at which to execute each turn
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
        """
        Move the robot to the next bay in the search list.
        
        Args:
            current_bay: Current bay position as a string (e.g., "B01")
        """
        if (current_bay == "B0"):
            self._path_algorithm("B06", "B16") 
            return "B1"
        elif (current_bay == "B1"):
            self._path_algorithm("B11", "B0") 
            return "G0"
        elif (current_bay == "A1"):
            self._path_algorithm("A16", "G0") 
            return "G0"
        elif (current_bay == "A0"):
            self._path_algorithm("A06", "A16")
            return "A1"
        

    def pick_box(self, start = "G0", destination = "A01") -> (str, str):
        self._path_algorithm(start, destination)
        temp_line_counter = int(destination[2])
        self.found_box = False

        current_bay = destination[0:1] # It doesn`t matter the specific bay number, just the side
        self.direction_flag = "forward"
        self.motor_go_straight(self.MIN_SPEED, self.MIN_SPEED)
        while(not self.found_box):
            if self.signal_far_left.value() == 1 or self.signal_far_right.value() == 1:
                self.is_box(current_bay)
                temp_line_counter += 1
            if temp_line_counter > 6:
                current_bay = self._go_to_next_bay(current_bay)
                temp_line_counter = 1
                if current_bay == "G0":
                    print("No boxes found in any bay.")
                    return "G0", "G0"
            else:
                self.moving_between_lines(current_bay)
        
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
    
    def moving_between_lines(self,position):
        if position == "B0" or position == "A1":
            while(self.signal_far_left.value()==0):
                pass
        elif position == "B1" or position == "A0":
            while(self.signal_far_right.value()==0):
                pass

    def is_box(self,position) -> bool:
        if position == "B0" or position == "A1":
            while (self.signal_far_left.value()==1):
                if (self._calculate_distance_tof() < 300):
                    self.found_box = True
                sleep(0.05)
            return False
        elif position == "B1" or position == "A0":
            while (self.signal_far_right.value()==1):
                if (self._calculate_distance_URM09() <300):
                    self.found_box = True
                sleep(0.05)
            return False

    def _before_pick_box(self, current_position: str):
        """Here we want to help turn and then it will automatically readjust itself
        """
        if current_position =="B0" or current_position == "A1":
            self._execute_turn(self.motor_turn_left_bay, f"before_pick_box_{current_position}")
            self.motors_off()
            
        elif current_position  == "A0" or current_position == "B1":
            self._execute_turn(self.motor_turn_right_bay, f"before_pick_box_{current_position}")
            self.motors_off()
            # Wait until centered on line or timeout

        
    def _after_pick_box(self, current_position):
        """ First we want to reverse out and then go to the ground position"""
        if current_position[0:1] == "B0" or current_position[0:1] == "A1":
            self.direction_flag = "reverse"
            self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, delay = 0.5)
            self._execute_turn(self.motor_turn_right_back, "after_pick_box")
            self.direction_flag = "forward"
            self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
            while (self.signal_far_right.value()==1):
                pass
        elif current_position[0:1] == "B1" or current_position[0:1] == "A0":
            self.direction_flag = "reverse"
            self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, delay = 0.5)
            self._execute_turn(self.motor_turn_left_back, "after_pick_box")
            self.direction_flag = "forward"
            self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED)
            # Wait until the far left sensor is off the line 
            while self.signal_far_left.value() == 1:
                pass
        # self.motors_off()

    def _do_pick_box(self):
        colour_detected = False
        number_of_attempts = 0
        identified_colour = None
        
        #Identify the colour of the box before picking it up
        while not colour_detected and number_of_attempts < 3:
            self._approach_box()
            identified_colour = self.colour_sensor.detect_colour()
            print("Identified colour:", identified_colour)
            
            if identified_colour is not None:
                colour_detected = True
                self.linear_actuator.set("retract", 90)
                sleep(3)
                self.linear_actuator.off()
            else:
                print("Couldn't identify colour, redoing the pick up.")
                self.direction_flag = "reverse"
                self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED, delay=1)
                self.motors_off()
                number_of_attempts += 1
        
        if not colour_detected:
            print("Failed to identify colour after 3 attempts, aborting pick up.")
            self.motors_off()
            return None
        
        return identified_colour

    def _approach_box(self):
        self.direction_flag = "forward"
        self.motor_go_straight(self.MIN_SPEED, self.MIN_SPEED)
        while (self._calculate_distance() > 60):
            print("Waiting to reach box...")
            pass
        self.motors_off()
    
    def _place_box(self):
        self.direction_flag = "forward"
        self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED, delay = 3)
        self.motors_off()
        self._actuator_final_position()
        self.linear_actuator.off()
        self.direction_flag = "reverse"
        self.motor_go_straight(self.BASE_SPEED, self.BASE_SPEED, delay = 1.5)
        self.motors_off()
        self._actuator_initial_position()

    

    # def _detect_colour(self) -> str:
    #     """
    #     Detect the color of the box under the robot using the color sensor.

    #     Returns:
    #         str: The detected color as a string
    #     """
    #     # Placeholder implementation - replace with actual color detection logic
    #     return "R"

    def run(self):
        """
        Main control loop for the line follower.
        Continuously monitors sensors and adjusts motors until sequence is complete.
        """
        print("Robot ready. Press button to start...")
        while not self.is_running:
            sleep(0.1)

        print("Start line follower")
        #Start the flashing LED to indicate running
        self.yellow_led.value(0)
        # Start the algorithm to pick boxes
        current_position, position_to_go = self.pick_box(start="G0", destination="A01")
        print(f"New position: {position_to_go}")
        self._path_algorithm(current_position, position_to_go)
        if (position_to_go in self.list_of_bays):
            self._place_box()
        self._reverse_on_the_spot()
        #TODO: Create a sequence going through 
    
        #Last line of the run function
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




""" TODO

# from G0 to B01
# return ["right", 1, "left", 0]

for (command in commands):
    if command == "right":
        motor_turn_right()
    elif command == "left":
        motor_turn_left()
    else:
        go straight for %d lines and stop


"""