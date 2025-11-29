import os

# Check if the file exists
try:
    f = open("/main_app_3v3_k2.hex", "r")
    print("File opened successfully!")
    f.close()
except Exception as e:
    print("Error:", e)

# List root
print(os.listdir("/"))