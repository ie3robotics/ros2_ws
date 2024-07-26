import serial

ports = [
    '/dev/ttyS0',
    '/dev/ttyS1',
    '/dev/ttyS2',
    '/dev/ttyS3',
    '/dev/ttyAMA0',
    '/dev/ttyGS0',
    '/dev/ttyTHS1',
    '/dev/ttyTHS2',
]

for port in ports:
    try:
        ser = serial.Serial(port, 9600)  # You may need to adjust the baud rate
        print(f"Serial port {port} opened successfully")
        ser.close()
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
