from machine import Pin

def test_pin(pin_number):
    """
    Test a GPIO pin by toggling its state on and off.
    Args:
        pin_number (int): The GPIO pin number to test.
    """
    test = Pin(pin_number, Pin.OUT)
    state = 0
    
    print(f"Testing PIN {pin_number}")
    print("Press Enter to toggle, Ctrl+C to exit\n")
    
    try:
        while True:
            state = 1 - state  # Toggle between 0 and 1
            test.value(state)
            status = "ON" if state else "OFF"
            print(f"PIN {pin_number}: {status}")
            input("Press any key + Enter to toggle...")
            
    except KeyboardInterrupt:
        test.value(0)
        print(f"\nPIN {pin_number} set to OFF")

if __name__ == "__main__": 
    test_pin(4)
