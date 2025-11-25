from machine import Pin, I2C
from utime import sleep, ticks_ms, ticks_diff
from sw.test_motor import Motor
from sw.path import Path
# from sw.libs.VL53L0X.VL53L0X import VL53L0X
# from sw.libs.DFRobot_URM09.DFRobot_URM09 import DFRobot_URM09
# from sw.libs.tcs3472_micropython.tcs3472 import tcs3472


class LineFollowerRobot:
    """
    Line-following robot with integrated sensor processing and motor control.
    Encapsulates all state and behavior for autonomous line following.
    """
    
    # Configuration constants
    BASE_SPEED = 50
    SPEED_ADJUSTMENT = 10
    MIN_SPEED = 35
    MAX_SPEED = 80

    SEARCH_LIST = [
        "B01", "B02", "B03", "B04", "B05", "B06", 
        "B16", "B15", "B14", "B13", "B12", "B11", 
        "A16", "A15", "A14", "A13", "A12", "A11",
        "A06", "A05", "A04", "A03", "A02", "A01"
    ]
    
    # Proportional control parameters
    KP = 4  # Proportional gain for steering correction

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
        # self.vl53l0.start()
    
    def _init_sensors(self):
        """Initialize all sensor pins"""
        self.signal_mid_right = Pin(self.MID_RIGHT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_mid_left = Pin(self.MID_LEFT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_far_right = Pin(self.FAR_RIGHT_PIN, Pin.IN, Pin.PULL_DOWN)
        self.signal_far_left = Pin(self.FAR_LEFT_PIN, Pin.IN, Pin.PULL_DOWN)
        
        # config I2C Bus
        # i2c_bus_vl5310 = I2C(0, sda=Pin(self.SDA_PIN), scl=Pin(self.SCL_PIN))  # I2C0 on GP8 & GP9
        
        # # Setup vl53l0 object
        # self.vl53l0 = VL53L0X(i2c_bus_vl5310)
        # self.vl53l0.set_Vcsel_pulse_period(self.vl53l0.vcsel_period_type[0], 18)
        # self.vl53l0.set_Vcsel_pulse_period(self.vl53l0.vcsel_period_type[1], 14)

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
        
        # Direction control
        self.direction_flag = "forward"  # Global flag: "forward" or "reverse"
        
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
        self.is_running = True  # Start immediately since button not connected
        # self.last_button_time = 0
    
    def motor_turn_right(self):
        """Configure motor speeds for right turn"""
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed = 0
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        while (self.signal_mid_right.value()==0):
            pass
        
    
    def motor_turn_left(self):
        """Configure motor speeds for left turn"""
        self.left_wheel_speed = 0
        self.right_wheel_speed = self.MAX_SPEED
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        while (self.signal_mid_left.value()==0):
            pass
    def motor_turn_right_back(self):
        """Configure motor speeds for right turn"""
        self.left_wheel_speed = self.MAX_SPEED
        self.right_wheel_speed = 0
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        while (self.signal_far_right.value()==0):
            pass
    def motor_turn_left_back(self):
        """Configure motor speeds for left turn"""
        self.left_wheel_speed = 0
        self.right_wheel_speed = self.MAX_SPEED
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        while (self.signal_far_left.value()==0):
            pass

    def motor_go_straight(self, speed_left, speed_right,delay = 0):
        """
        Execute straight movement using the global direction_flag
        
        Args:
            speed_left: Speed for left motor (0-100)
            speed_right: Speed for right motor (0-100)
        """
        if self.direction_flag == "forward":
            self.motor_left.Forward(speed_left)
            self.motor_right.Forward(speed_right)
        elif self.direction_flag == "reverse":
            self.motor_left.Reverse(speed_left)
            self.motor_right.Reverse(speed_right)
        if delay > 0:
            sleep(delay)

    
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
        
        def count_handler(p):
            self.line_counter(p)
        
        # Attach interrupt handlers to mid sensor pins for line following
        self.signal_mid_right.irq(handler=line_handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self.signal_mid_left.irq(handler=line_handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        
        # Attach interrupt handlers to far sensor pins for line counting (both rising and falling edges)
        self.signal_far_left.irq(handler=count_handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self.signal_far_right.irq(handler=count_handler, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
    
        # Enable interrupts for button (not connected yet)
        # self.button.irq(handler=self.button_handler, trigger=Pin.IRQ_FALLING) 

    def disable_interrupts(self):
        """Disable interrupt handlers for all sensors"""
        self.signal_mid_right.irq(handler=None)
        self.signal_mid_left.irq(handler=None)
        self.prev_sensor_left = 0
        self.prev_sensor_right = 0
        # self.signal_far_left.irq(handler=None)
        # self.signal_far_right.irq(handler=None)

    def _path_algorithm(self, current_position, desired_position):
        '''
        Args:  "GB", "GG", "G0", "GY", "GR"
        "B01", "B02", "B03", "B04", "B05", "B06", 
        "B16", "B15", "B14", "B13", "B12", "B11", 
        "A16", "A15", "A14", "A13", "A12", "A11",
        "A06", "A05", "A04", "A03", "A02", "A01"

        return a list of commands to go from current to desired position
        '''

        if (current_position.startswith("G") and desired_position.startswith("B0")):
            lines, turns = Path._path_G_to_B0(current_position, desired_position)
            self._handle_identifying_cases(lines, turns)
        if (current_position.startswith("G") and desired_position.startswith("B1")):
            return Path._path_G_to_B1(current_position, desired_position)
        if (current_position.startswith("G") and desired_position.startswith("A0")):
            return Path._path_G_to_A0(current_position, desired_position)
        if (current_position.startswith("G") and desired_position.startswith("A1")):
            return Path._path_G_to_A1(current_position, desired_position)
        if (current_position.startswith("B0") and desired_position.startswith("G")):
            lines, turns = Path._path_B0_to_G(current_position, desired_position)
            self._handle_identifying_cases(lines, turns)
        if (current_position.startswith("B1") and desired_position.startswith("G")):
            return Path._path_B1_to_G(current_position, desired_position)
        if (current_position.startswith("A0") and desired_position.startswith("G")):
            return Path._path_A0_to_G(current_position, desired_position)
        if (current_position.startswith("A1") and desired_position.startswith("G")):
            return Path._path_A1_to_G(current_position, desired_position)


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
            print("========================here========================")
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
                error = 2.5  # Larger error for stronger correction
            elif self.sensor_right_prev == 1 and self.sensor_left_prev == 0:
                # Last seen right on line - turn right aggressively
                error = -2.5  # Larger error for stronger correction
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
        self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        
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
        self.turning_case = next_case
        self.count_lines = 0
        self.setup_interrupts()
        print(f"end: turning_case {case_name} turning")

    # def _calculate_distance(self) -> int:
    #     """
    #     Read distance from VL53L0X sensor.
        
    #     Returns:
    #         Distance in millimeters, or 9999 if sensor fails
    #     """
    #     distance = self.vl53l0.read()
    #     print(">>> Dist: ", distance)
    #     sleep(0.1)  # Wait for sensor reading
    #     return distance if distance is not None else 9999  # Return large value on failure to avoid false triggers
    
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
                    print("==============here=================")
                    self.direction_flag = "forward"
                    self._execute_turn(self.motor_turn_right, 1.5, current_instruction + 1, str(current_instruction))
                elif turns[current_instruction] == "left_forward": 
                    self.direction_flag = "forward"
                    self._execute_turn(self.motor_turn_left, 1.5, current_instruction + 1, str(current_instruction))
                elif turns[current_instruction] == "straight":
                    # Continue straight - just increment instruction counter
                    self.direction_flag = "forward"
                    self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
                elif turns[current_instruction] == "right_backward":
                    self.direction_flag = "reverse"
                    self._execute_turn(self.motor_turn_right_back, 1.5, current_instruction + 1, str(current_instruction))
                elif turns[current_instruction] == "left_backward":
                    self.direction_flag = "reverse"
                    self._execute_turn(self.motor_turn_left_back, 1.5, current_instruction + 1, str(current_instruction))
                elif turns[current_instruction] == "reverse":
                    self.direction_flag = "reverse"
                    self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, delay = 1)
            
                # Move to next instruction
                current_instruction += 1
            
            # Small delay to avoid tight loop
            sleep(0.05)
        #Now it is at 90 degrees to the box
        sleep(1)
        
        # All instructions completed
        # print("Navigation sequence complete")
        # current_lines = self.count_lines
        # while True:
        #     if self.count_lines > current_lines:
        #         self.motors_off()
        #         break

    # def _handle_turning_cases(self):
    #     """
    #     Process turning logic based on current turning case.
    #     Returns True if final case is complete (should exit), False otherwise.
    #     """
    #     # Calculate distance once and cache it for all conditions
    #     # distance = self._calculate_distance()

    #     print("Current turning_case:", self.turning_case, " Lines counted:", self.count_lines)

    #     if self.count_lines == 2  and self.turning_case == 0:
    #         # if (sensor_far_right == 1):
    #         self._execute_turn(self.motor_turn_right, 1, 1, "0")
            

    #     elif self.count_lines ==2 and self.turning_case == 1:
    #         # if (sensor_far_left == 1) :
    #         self._execute_turn(self.motor_turn_left, 1, 2, "1")
            
            
    #     elif self.count_lines == 8 and self.turning_case == 2:
    #         # if (sensor_far_left == 1) :
    #         self._execute_turn(self.motor_turn_left, 1, 3, "2")
            
            
    #     elif self.count_lines == 2 and self.turning_case == 3:
    #         # if (sensor_far_left == 1) :
    #         self._execute_turn(self.motor_turn_left, 1, 4, "3")
            
            
    #     elif self.count_lines == 8 and self.turning_case == 4:
    #         # if (sensor_far_left == 1) :
    #         self._execute_turn(self.motor_turn_left, 1.5, 5, "4")
            
            
    #     elif self.count_lines == 2 and self.turning_case == 5:
    #         # Final approach sequence
    #         self.motor_turn_right()
    #         self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, direction="forward")
    #         sleep(1.5)
    #         self.motor_go_straight(70, 70, direction="forward")
    #         sleep(1.5)
    #         self.motors_off()
    #         return True  # Signal to exit main loop
        
    #     return False  # Continue running

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
    def pick_box(self, start = "G0", destination = "B01") -> (str, str):
        self._path_algorithm(start, destination)
        temp_line_counter = int(destination[2])

        while(not self.is_box() and temp_line_counter < 7):
            temp_line_counter += 1
            # go straight for 1 sec

            # go straight to next, until scan through all available boxes

        current_position = destination[0:2] + str(temp_line_counter)

        self._before_pick_box(destination)
        colour = self._detect_colour()
        position_to_go = "G" + colour[0]
        self._do_pick_box()        # pick the box
        self._after_pick_box(current_position) # reverse, do the turning
        #Depending on the colour we go to a different bay
        self._path_algorithm(current_position, position_to_go)
        return current_position, position_to_go

    def is_box(self) -> bool:
        """
        Check if the robot is currently over a box using color detection.
        
        Returns:
            bool: True if box detected, False otherwise
        """
        return False
    def _before_pick_box(self, current_position: str):
        if current_position[0:2] == "B0":
            self._execute_turn(self.motor_turn_left, 1.5, 0, "before_pick_box")
            if (self.signal_mid_left.value()==0 and self.signal_mid_right.value() == 0):
                self.motor_off()
        elif current_position[0:2] == "A0":
            self._execute_turn(self.motor_turn_right, 1.5, 0, "before_pick_box")
            if (self.signal_mid_left.value()==0 and self.signal_mid_right.value() == 0):
                self.motor_off()
        elif current_position[0:2] == "B1":
            self._execute_turn(self.motor_turn_right, 1.5, 0, "before_pick_box")
            if (self.signal_mid_left.value()==0 and self.signal_mid_right.value() == 0):
                self.motor_off()
        elif current_position[0:2] == "A1":
            self._execute_turn(self.motor_turn_left, 1.5, 0, "before_pick_box")
            if (self.signal_mid_left.value()==0 and self.signal_mid_right.value() == 0):
                self.motor_off()
        
    def after_pick_box(self, current_position):
        """ First we want to reverse out and then go to the ground position"""
        if current_position[0:2] == "B0" or current_position[0:2] == "A1":
            self.direction_flag = "reverse"
            self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, delay = 1)
            self._execute_turn(self.motor_turn_right_back, 1.5, 0, "after_pick_box")
            self.direction_flag = "forward"
            self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
            while (self.signal_far_right.value()==1):
                pass
        elif current_position[0:2] == "B1" or current_position[0:2] == "A0":
            self.direction_flag = "reverse"
            self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed, delay = 1)
            self._execute_turn(self.motor_turn_left_back, 1.5, 0, "after_pick_box")
            self.direction_flag = "forward"
            self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
            while (self.signal_far_left.value()==1):
                pass

    def _detect_colour(self) -> str:
        """
        Detect the color of the box under the robot using the color sensor.

        Returns:
            str: The detected color as a string
        """
        # Placeholder implementation - replace with actual color detection logic
        return "R"

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
        # self.motor_go_straight(self.left_wheel_speed, self.right_wheel_speed)
        # self._path_algorithm("G0", "B01")
        self._path_algorithm("B01", "G0")
        # while self.is_running:
        #     # Handle turning cases - exit if final case is complete
        #     if self._handle_turning_cases():
        #         self.is_running = False
        #         self.destroy()  # Clean up when finished
        #         break
        #     # Longer sleep since interrupts handle line following immediately
        #     sleep(0.05)  # Reduced from 100Hz to 20Hz polling

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