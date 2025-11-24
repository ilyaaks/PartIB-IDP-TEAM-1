from machine import Pin, ADC
from utime import sleep

def test():
    # URM09 analog output on pin 26
    Max_range = 500  # Maximum range in cm
    ADC_Resolution = 65535  # 16-bit ADC resolution
    
    input_pin = ADC(Pin(26))
    
    print("URM09 Ultrasonic Sensor Test (Analog Mode)")
    print("Press Ctrl+C to stop")
    print("-" * 40)
    
    while True:
        try:
            # Read ADC value
            adc_value = input_pin.read_u16()
            
            # Convert ADC value to distance in cm
            # Distance = (ADC_value / ADC_Resolution) * Max_range
            distance = (adc_value / ADC_Resolution) * Max_range
            
            print(f"ADC: {adc_value} | Distance: {distance:.2f} cm")
            
            sleep(0.5)
            
        except KeyboardInterrupt:
            print("\nTest stopped by user")
            break
        except Exception as e:
            print(f"Error: {e}")
            sleep(1)

if __name__ == "__main__":
    test()
