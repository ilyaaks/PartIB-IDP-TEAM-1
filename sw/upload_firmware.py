"""
Manual firmware upload script
Run this on your Pico to upload the firmware file
"""

def upload_firmware_from_computer():
    """
    This script reads the hex file from your computer and writes it to the Pico
    You need to run this from your computer, not the Pico
    """
    import os
    
    # Read hex file from computer
    computer_path = r"c:\Users\Illia\PartIB-IDP-TEAM-1\sw\libs\DFRobot_TMF8x01\fw\TMF8701\main_app_3v3_k2.hex"
    pico_path = "sw/libs/DFRobot_TMF8x01/fw/TMF8701/main_app_3v3_k2.hex"
    
    print(f"Reading from: {computer_path}")
    
    try:
        with open(computer_path, 'rb') as f:
            firmware_data = f.read()
        
        print(f"Read {len(firmware_data)} bytes")
        
        # Create directory structure on Pico
        dirs = ["sw", "sw/libs", "sw/libs/DFRobot_TMF8x01", 
                "sw/libs/DFRobot_TMF8x01/fw", "sw/libs/DFRobot_TMF8x01/fw/TMF8701"]
        
        for dir_path in dirs:
            try:
                os.mkdir(dir_path)
                print(f"Created directory: {dir_path}")
            except:
                print(f"Directory exists: {dir_path}")
        
        # Write to Pico
        print(f"Writing to Pico: {pico_path}")
        with open(pico_path, 'wb') as f:
            f.write(firmware_data)
        
        print("✓ Firmware uploaded successfully!")
        
        # Verify
        stat = os.stat(pico_path)
        print(f"Verified: {stat[6]} bytes written")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    upload_firmware_from_computer()
