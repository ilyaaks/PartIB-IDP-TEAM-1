from machine import Pin
from utime import sleep
led_pin = 14  # Pin 14 = GP14
led = Pin(led_pin, Pin.OUT)
def test_led():

   led.value(1)  
   sleep(5)
   led.value(0)
   sleep(4)
   led.value(1)

if __name__ == "__main__":
    try:
        test_led()
    except KeyboardInterrupt:
        led.value(0)
        print("LED test interrupted by user")