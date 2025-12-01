from machine import Pin, SoftI2C, I2C
from sw.libs.tcs3472_micropython.tcs3472 import tcs3472
from utime import sleep
import math

class COLOUR:
    RED = (140, 60, 60)
    BLUE = (20, 90, 160)
    GREEN = (40, 100, 120)
    YELLOW = (120, 120, 40)

class ColourSensor():
    def __init__(self):
        # i2c_bus = SoftI2C(sda=Pin(8), scl=Pin(9)) 
        i2c_bus_tcs = I2C(id=0, sda=Pin(8), scl=Pin(9), freq=100000)  # Reduced frequency for stability
        # print("I2C devices found:", i2c_bus_tcs.scan())
        self.colour_pin = Pin(10, Pin.OUT)
        self.colour_pin.value(0)  # Power off the sensor (inverse logic)
        # Add delay for sensor initialization
        sleep(0.1)
        
        self.tcs = tcs3472(i2c_bus_tcs)  # TCS3472 is always at address 41 (0x29)
        sleep(0.1)  # Give sensor time to initialize
        

    def get_rgb_value(self) -> (int, int, int):
        # create an output pin at 18 
        self.colour_pin.value(0) # ON

        '''
        In the actual setup, we set the Pin.OUT to high (1)
        and Pin.OUT to 0 to turn on the circuit 
        and Pin.OUT to 1 to turn off the circuit after detection. 
        '''
        
        # Wait for sensor to stabilize
        sleep(0.2)
        
        try:
            result = self.tcs.rgb()
            print("Light:", self.tcs.light())
            print("RGB:", result)
        except OSError as e:
            print(f"I2C Error: {e}")
            print("Check wiring and connections!")
            return None
        
        sleep(1)
        self.colour_pin.value(1) # OFF
        sleep(1)
        return result

    def detect_colour(self, warmup_time=5):
        """
        Detect the color of the surface under the robot using the color sensor.
        Waits for warmup_time seconds to allow sensor to stabilize before reading.

        Parameters:
            warmup_time (int): Time in seconds to wait before taking readings (default: 5)

        Returns:
            str: The detected color name or None if no match
        """
        # Turn on sensor
        self.colour_pin.value(0)  # ON
        
        # Wait for sensor to stabilize - first few seconds are inaccurate
        print(f"Waiting {warmup_time} seconds for colour sensor to stabilize...")
        sleep(warmup_time)
        
        # Now take the reading
        try:
            rgb = self.tcs.rgb()
            print("Light:", self.tcs.light())
            print("RGB:", rgb)
        except OSError as e:
            print(f"I2C Error: {e}")
            self.colour_pin.value(1)  # OFF
            return None
        
        self.colour_pin.value(1)  # OFF
        
        if rgb is None:
            return None
        
        r, g, b = rgb

        # Determine the closest color based on RGB values
        min_distance = float('inf')
        closest_color = None

        # Iterate through all color attributes in COLOUR class
        for color_name in ['RED', 'BLUE', 'GREEN', 'YELLOW']:
            color_value = getattr(COLOUR, color_name)
            
            # Calculate Euclidean distance between sensor reading and color values
            distance = math.sqrt((r - color_value[0]) ** 2 + (g - color_value[1]) ** 2 + (b - color_value[2]) ** 2)

            # Update closest color if current distance is smaller
            if distance < min_distance:
                min_distance = distance
                closest_color = color_name

        print(f"Closest color: {closest_color} (distance: {min_distance:.2f})")
        return closest_color

if __name__ == "__main__":
    # pin = Pin(16, Pin.OUT)
    # Pin(16, Pin.OUT).value(0)  
    robot = ColourSensor()
    while True:
        rgb = robot.get_rgb_value()
        if rgb:
            color = robot.detect_colour()
            print(f"Detected color: {color}")
        sleep(2)
    
