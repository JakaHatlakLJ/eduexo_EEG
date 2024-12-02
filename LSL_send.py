#import socket
from dynamixel_sdk import *  # Import the Dynamixel SDK
from gpiozero import LED
import os
from pylsl import StreamInfo, StreamOutlet
import numpy as np


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
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
LENGTH = 4  # Length of the data (4 bytes for position data)

# Setup UDP communication (PC IP address and port)
#UDP_IP = "192.168.1.44"  # Replace with your PC's IP address
#UDP_PORT = 5005  # Port on which the data will be received

# Unit conversion
pos_unit = 360/4095         # [deg] = [dxl_unit] * [pos_unit]
vel_unit = 2 * np.pi/60     # [rad/s]
cur_unit = 2.69             # [mA]



# Create a UDP socket
#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Create a new stream info
info = StreamInfo('Stream_EXO', 'EEG', 4, 100, 'float32', 'test_LSL')
# Create an outlet
outlet = StreamOutlet(info)


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
    b1 = dxl_present_position.to_bytes(4, byteorder=sys.byteorder, signed = False) 
    dxl_present_position = int.from_bytes(b1, byteorder=sys.byteorder, signed = True)
    return dxl_present_position

def get_motor_current():
    """Get the current current of the Dynamixel motor"""
    dxl_present_current, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(port_handler, DXL_ID, ADDR_PRESENT_CURRENT)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Error: {packet_handler.getTxRxResult(dxl_comm_result)}")
        return None
    b1 = dxl_present_current.to_bytes(2, byteorder=sys.byteorder, signed = False) 
    dxl_present_current = int.from_bytes(b1, byteorder=sys.byteorder, signed = True)    
    return dxl_present_current

def get_motor_velocity():
    """Get the current velocity of the Dynamixel motor"""
    dxl_present_velocity, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, DXL_ID, ADDR_PRESENT_VELOCITY)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Error: {packet_handler.getTxRxResult(dxl_comm_result)}")
        return None
    b1 = dxl_present_velocity.to_bytes(4, byteorder=sys.byteorder, signed = False) 
    dxl_present_velocity = int.from_bytes(b1, byteorder=sys.byteorder, signed = True)    
    return dxl_present_velocity


led = LED(27)       # Turn LED on
led.on()

try:
    while True:
        print(f'Sending motor position through stream: EEG', '\n', 'Press any key to contionue (or press ESC to quit!)')
        if getch() ==chr(0x1b):
            break
        else:
            print(f'Position sending is active')

        while 1:    
            position = get_motor_position()
            velocity = get_motor_velocity()
            current = get_motor_current()
            torque = 0.082598 * (1 - (1 - current * cur_unit / 8.247191)**2) #[mNm]
            
            DATA = [position * pos_unit, velocity * vel_unit, current * cur_unit, torque]
            # Send motor position data via stream
            outlet.push_sample(DATA)
            #message = str(position).encode('utf-8')
            #sock.sendto(message, (UDP_IP, UDP_PORT)) 

except KeyboardInterrupt:
    print("Program interrupted")
    #sock.sendto("Remote program interrupted".encode('utf-8'), (UDP_IP, UDP_PORT))
    led.off()

finally:
    port_handler.closePort()
    #sock.close()