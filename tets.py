#!/usr/bin/env python
# -*- coding: utf-8 -*-

#*******************************************************************************
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#*******************************************************************************


#*******************************************************************************
#***********************     Read and Write Example      ***********************
#  Required Environment to run this example :
#    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
#    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
#  How to use the example :
#    - Select the DYNAMIXEL in use at the MY_DXL in the example code. 
#    - Build and Run from proper architecture subdirectory.
#    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
#    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
#  Author: Ryu Woon Jung (Leon)
#  Maintainer : Zerom, Will Son
# *******************************************************************************

import os
import sys

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


from time import perf_counter
import numpy as np
from gpiozero import LED, Button
from ahrs.filters import Tilt
import threading

# Uses Dynamixel SDK library
from dynamixel_sdk import * 

# DFRobot_LIS library to read LIS331HH Triple Axis Accelerometer data
sys.path.append('/home/eduexo/Desktop/DFRobot_LIS/python/raspberrypi') # set system path to top
from DFRobot_LIS import *

# RPi GPIO Settings
led = LED(27)       
sync_pin = LED(26)

sync_pin.off() # Set initial state of trigger pin

# DYNAMIXEL SETTINGS

# Control table address
ADDR_TORQUE_ENABLE          = 64
ADDR_OP_MODE                = 11
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_PRESENT_VELOCITY       = 128
ADDR_GOAL_CURRENT           = 102
ADDR_PRESENT_CURRENT        = 126
ADDR_CURRENT_LIMIT          = 38
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
DXL_CURRENT_LIMIT_VALUE     = 2047   	# Refer to the Current Limit of product eManual
BAUDRATE                    = 57600
ADDR_WATCHDOG               = 98

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID                      = 1

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 1     # Dynamixel moving status threshold

POSITION_CONTROL            = 3    # Value for switching to position control mode
CURRENT_CONTROL             = 0    # Value for switching to current control mode

# Bus Watchdog value
watchdog_time = 5          # [1 = 20 ms]
watchdog_clear = 0

# Position offset and limits values
dxl_position_offset = 90   # [deg] Torque/current is 0 at this position
min_pos = 45               # [deg] Torque/current is set to 0 if limit is exceeded
max_pos = 170              # [deg] Torque/current is set to 0 if limit is exceeded

# Unit conversion
pos_unit = 0.088           # [deg] = [dxl_unit] * [pos_unit]
vel_unit = 0.229           # [rpm] = [dxl_unit] * [vel_unit]
cur_unit = 2.69            # [mA] = [dxl_unit] * [cur_unit]

# Current limit
max_torque = 8                                                           # [Nm]
max_current = 8.247191-8.247191*np.sqrt(1-0.082598*max_torque)           # [A]
max_current = max_current * 1000                                         # [mA]
max_current = round(max_current / cur_unit)                              # [dxl units]

# Initial settings (later replaced by user input values)
K_s = 0.00                   # [Nm/deg] Initial spring coefficient
K_d = 0.00

ard_line = 0

# GRAVITY COMPENSATION PARAMETERS

#m_body = 56               # total body mass in [kg]
#L_arm = 0.225/2.81 # 0.08 # elbow axis to ulnar styloid lenght [m]

m_arm = 0.803 +0.809 +0.809 #0.022 * m_body    # mass of forearm + hand in [kg]
m_mech = 0.0105  #0.2175           # mass of mechanical parts in [kg]
m = m_arm + m_mech
g = 9.807                 # gravity acceleration [m/s^2]

x_arm = 0                 # distance from motor axis to arm CoG in x direction [m]
y_arm = -0.039 #-0.682*L_arm
z_arm = 0                 
x_mech = 0                # distance from motor axis to mechanical parts CoG in x direction [m]
y_mech = -0.0126 #-0.100
z_mech = 0

cog_x = (x_arm*m_arm+x_mech*m_mech)/m
cog_y = (y_arm*m_arm+y_mech*m_mech)/m
cog_z = (z_arm*m_arm+z_mech*m_mech)/m

# RESULTS PRINTING TIME
t_print = 0.1             # [s] after this time results are printed

# ACCELEROMETER INITIALIZATION

#Connect the accelerometer module via I2C port.
I2C_BUS         = 0x01            #default use I2C1
#ADDRESS_0       = 0x18            #Sensor address0
ADDRESS_1       = 0x19            #Sensor address1
acce = DFRobot_LIS331HH_I2C(I2C_BUS ,ADDRESS_1)

#Chip initialization
acce.begin()
#Get chip id
print('chip id :%x'%acce.get_id())

'''
set range:Range(g)
          LIS331HH_6G  = 6   #±6G
          LIS331HH_12G = 12  #±12G
          LIS331HH_24G = 24  #±24G
'''
acce.set_range(acce.LIS331HH_6G)

'''
Set data measurement rate
         POWERDOWN_0HZ 
         LOWPOWER_HALFHZ 
         LOWPOWER_1HZ 
         LOWPOWER_2HZ 
         LOWPOWER_5HZ 
         LOWPOWER_10HZ 
         NORMAL_50HZ 
         NORMAL_100HZ 
         NORMAL_400HZ 
         NORMAL_1000HZ 
'''
acce.set_acquire_rate(acce.NORMAL_1000HZ)
time.sleep(0.1)

# SERIAL COMMUNICATION WITH ARDUINO INITIALIZATION

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1.0)      # open serial # timeout is the amount of time the command waits for data
time.sleep(3)                                               # gives Arduino time to start serial
ser.reset_input_buffer()                                    # fresh buffer
print('Serial OK')

# DYNAMIXEL INITIALIZATION

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# LOCKER INITIALIZATION
the_lock = threading.Lock()
position_lock = threading.Lock()
dxl_lock = threading.Lock()


while 1:
    #Get the acceleration in the three directions of xyz
    #The measurement range can be ±6g, ±12g or ±24g, set by set_range() function
    x,y,z = acce.read_acce_xyz()
    #Change coordinates to fit the model used for calculation
    ac_x = -(z)
    ac_y = -(y)
    ac_z = -(-x)
    #print(ac_x, ac_y, ac_z)

    # Orientation calculation using AHRS library
    acc_data = np.array([[ac_x, ac_y, ac_z]])
    #acc_data = np.ndarray(shape=(1,3), dtype=float, buffer=np.array([ac_x, ac_y, ac_z]))
    tilt = Tilt(acc_data, representation='angles')

    # First and second rotation: roll_1, pitch
    roll_1_deg = tilt.Q[0,0]                      # [°]
    pitch_deg = tilt.Q[0,1]   
    print(roll_1_deg, pitch_deg)
    
    
    with the_lock:
        global roll_1
        roll_1 = roll_1_deg*np.pi/180               # [rad]
        global pitch
        pitch = pitch_deg*np.pi/180

    print(roll_1, pitch)
    time.sleep(10)