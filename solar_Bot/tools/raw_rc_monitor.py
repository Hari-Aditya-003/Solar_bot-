import serial

# On Raspberry Pi 5, /dev/ttyAMA0 is the default hardware UART
try:
    ser = serial.Serial('/dev/ttyAMA0', 420000, timeout=1)
    print("Searching for ELRS signals on /dev/ttyAMA0...")
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

while True:
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        # This will print the raw HEX data coming from your RadioMaster
        print(f"Receiving Signal: {data.hex()}")
