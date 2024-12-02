import time
import socket
from dynamixel_sdk import *  # Import the Dynamixel SDK
from gpiozero import LED
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


# Motor and Serial Communication Parameters
PORT_NAME = '/dev/ttyUSB0'  # Replace with the serial port where your Dynamixel motor is connected
BAUDRATE = 57600  # Baud rate for communication (make sure it matches your motor's baud rate)
DXL_ID = 1  # Dynamixel motor ID
PROTOCOL_VERSION = 2.0  # Dynamixel protocol version (1.0 or 2.0)
ADDR_PRESENT_POSITION = 132  # Address for reading current position (for Dynamixel Pro series, use 132)
LENGTH = 4  # Length of the data (4 bytes for position data)

# Setup UDP communication (PC IP address and port)
UDP_IP = "192.168.1.44"  # Replace with your PC's IP address
UDP_PORT = 5005  # Port on which the data will be received

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Setup Dynamixel motor connection
port_handler = PortHandler(PORT_NAME)
packet_handler = PacketHandler(PROTOCOL_VERSION)

# Open port
if not port_handler.openPort():
    print("Failed to open port")
    quit()

# Set baudrate
if not port_handler.setBaudRate(BAUDRATE):
    print("Failed to set baud rate")
    quit()

def get_motor_position():
    """Get the current position of the Dynamixel motor"""
    dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, DXL_ID, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Error: {packet_handler.getTxRxResult(dxl_comm_result)}")
        return None
    return dxl_present_position


led = LED(27)       # Turn LED on
led.on()

try:
    while True:
        print(f'Sending motor position to', UDP_IP, '\n', 'Press any key to contionue (or press ESC to quit!)')
        if getch() ==chr(0x1b):
            break
        else:
            print(f'Position sending is active')

        while 1:    
            position = get_motor_position()
            # Send motor position data via UDP
            message = str(position).encode('utf-8')
            sock.sendto(message, (UDP_IP, UDP_PORT))
            time.sleep(0.1)

except KeyboardInterrupt:
    print("Program interrupted")
    sock.sendto("Remote program interrupted".encode('utf-8'), (UDP_IP, UDP_PORT))
    led.off()

finally:
    port_handler.closePort()
    sock.close()
