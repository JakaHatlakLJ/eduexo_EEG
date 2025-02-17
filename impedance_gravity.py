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

def orientation_calc():
    while 1:
        #Get the acceleration in the three directions of xyz
        #The measurement range can be ±6g, ±12g or ±24g, set by set_range() function
        x,y,z = acce.read_acce_xyz()
        #Change coordinates to fit the model used for calculation
        ac_x = -(z)
        ac_y = -(y)
        ac_z = -(-x)

        # Orientation calculation using AHRS library
        acc_data = np.array([[ac_x, ac_y, ac_z]])
        tilt = Tilt(acc_data, representation='angles')

        # First and second rotation: roll_1, pitch
        roll_1_deg = tilt.Q[0,0]                      # [°]
        pitch_deg = tilt.Q[0,1]   

        with the_lock:
            global roll_1
            roll_1 = roll_1_deg*np.pi/180               # [rad]
            global pitch
            pitch = pitch_deg*np.pi/180

def position_read():
    while 1:
        # Read present position
        with dxl_lock:
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
        b1 = dxl_present_position.to_bytes(4, byteorder=sys.byteorder, signed = False) 
        dxl_present_position = int.from_bytes(b1, byteorder=sys.byteorder, signed = True)
        with position_lock:
            global dxl_present_position_deg
            dxl_present_position_deg = dxl_present_position*pos_unit            #[deg]
            global  dxl_present_position_rad
            dxl_present_position_rad = (dxl_present_position_deg*np.pi)/180     # [rad]

# Set operating mode to current control mode
OP_MODE = CURRENT_CONTROL
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OP_MODE, OP_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Operating mode has been switched to current control.")

# Set the current limit (used to limit the maximum value of goal current in Torque control mode) 
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_CURRENT_LIMIT, max_current)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Current limit has been set.")

# LED LIGHT
led.on()    # Turn LED on


while 1:
    # Start of new loop
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() ==chr(0x1b):
        break

    # Open the file in which results will be saved
    f_input = input("Enter name of the file to save results to (or press Enter to skip).\n")
    if f_input!="":
        f_end = ".csv"
        f_name = f_input + f_end
        print('file name = '+f_name)
        results_file = open(f_name, "x") # Create new file ("x") with name ("")
    
    # Get user input for spring and damper coefficients and enable/disable gravity compensation
    K_s = input('Enter spring coefficient value in Nm/deg and press Enter (e.g. 0.1)\n')
    if K_s == "": # if there is no input set a default
        K_s = "0.2"
    print('Spring coefficient = '+K_s+"Nm/deg")    
    K_s = float(K_s)

    K_d = input('Enter damping coefficient value in Nm*s/deg and press Enter (e.g. 0.01)\n')
    if K_d == "": # if there is no input set a default
        K_d = "0.01"
    print('Damping coefficient = '+K_d+"Nm*s/deg")    
    K_d = float(K_d)

    G_mech = input('Enter 1 for gravity compensation of the exoskeleton and press Enter (or just press Enter for no gravity compensation)\n')
    if G_mech == "1":
        G_mech = 1
        print("Gravity compensation of the exoskeleton is on.")    
    else:
        G_mech = 0
        print("Gravity compensation of the exoskeleton is off.")

    G_arm = input('Enter 1 for gravity compensation of the arm and press Enter (or just press Enter for no gravity compensation)\n')
    if G_arm == "1":
        G_arm = 1
        print("Gravity compensation of the arm is on.")    
    else:
        G_arm = 0
        print("Gravity compensation of the arm is off.")


    # GRAVITY COMPENSATION PARAMETERS

    m_body = 81               # total body mass in [kg]
    L_arm = 0.27/2.81 # 0.08 # elbow axis to ulnar styloid lenght [m]

    m_arm = G_arm*0.022 * m_body    # mass of forearm + hand in [kg]
    m_mech = G_mech*0.2175           # mass of mechanical parts in [kg]
    m = m_arm + m_mech
    g = 9.807                 # gravity acceleration [m/s^2]

    x_arm = 0                 # distance from motor axis to arm CoG in x direction [m]
    y_arm = -0.682*L_arm
    z_arm = 0                 
    x_mech = 0                # distance from motor axis to mechanical parts CoG in x direction [m]
    y_mech = -0.100
    z_mech = 0

    if G_arm == 0 and G_mech ==0:
        cog_x = 0
        cog_y = 0
        cog_z = 0
    else:
        cog_x = (x_arm*m_arm+x_mech*m_mech)/m
        cog_y = (y_arm*m_arm+y_mech*m_mech)/m
        cog_z = (z_arm*m_arm+z_mech*m_mech)/m

    # Clear Bus Watchdog
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_WATCHDOG, watchdog_clear)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque is enabled.")
    
    roll_1 = 0
    pitch = 0

    dxl_present_position_deg = 0
    dxl_present_position_rad = 0

    # Initialize threads
    orientation_thread = threading.Thread(target= orientation_calc, daemon = True)
    orientation_thread.start()

    position_thread = threading.Thread(target= position_read, daemon = True)
    position_thread.start()


    ser.reset_input_buffer()# fresh buffer

    t0 = t1 = t5 =perf_counter() # Used for results printing timing

    # Sync pin set to high
    sync_pin.on()

    try:
        
        # Enable Bus Watchdog
        with dxl_lock:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_WATCHDOG, watchdog_time)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"Watchdog is set to {watchdog_time*20} ms.")

        # Used for initial velocity calculation
        with position_lock:
            dxl_present_position_main_deg = dxl_present_position_deg
            dxl_present_position_main_rad = dxl_present_position_rad
        
        # START of the control loop
        while 1:
            # Loop timing
            t4 = perf_counter()-t5
            t5 = perf_counter()

            # Save previous position for velocity calculation
            dxl_present_position_old = dxl_present_position_main_deg

            # Read present position     
            with position_lock:
                dxl_present_position_main_deg = dxl_present_position_deg
                dxl_present_position_main_rad = dxl_present_position_rad
            
            # Calculate present velocity
            dxl_present_velocity = (dxl_present_position_main_deg-dxl_present_position_old)/t4    # [deg/s]

            # First and second rotation: roll_1, pitch
            with the_lock:
                roll_1_main = roll_1       # [rad]
                pitch_main = pitch  

            # Third rotation: roll_2
            roll_2 = np.pi - dxl_present_position_main_rad   # [rad]
            roll_2_deg = roll_2 *180/np.pi # [°]

            # Calculate torque for gravity compensation

            R_roll_1 = np.array([[1, 0, 0], [0, np.cos(roll_1_main), np.sin(roll_1_main)], [0, -np.sin(roll_1_main), np.cos(roll_1_main)]])    
            R_pitch = np.array([[np.cos(pitch_main), 0, -np.sin(pitch_main)], [0, 1, 0], [np.sin(pitch_main), 0, np.cos(pitch_main)]])
            R_roll_2 = np.array([[1, 0, 0], [0, np.cos(roll_2), np.sin(roll_2)], [0, -np.sin(roll_2), np.cos(roll_2)]]) 
            P = np.matmul(R_roll_1, R_pitch)
            R = np.matmul(P, R_roll_2)

            R = np.transpose(R)              # Rotation matrix that changes any vector in earth frame to lower arm segment frame

            f = np.transpose(np.array([0,0,m*g])) #[N]
            f = np.matmul(R, f)              # Gravitation vector in lower arm segment frame

            r = np.transpose(np.array([cog_x, cog_y, cog_z])) #[m]
            M = np.cross(r, f)               # 3D Moment caused by gravitation vector #[Nm]    

            gr_comp_torque = M[0]
               
            # Calculate goal current
            if (dxl_present_position_main_deg<min_pos or dxl_present_position_main_deg>max_pos):           # Check if position is within position limits 
                dxl_goal_current = 0
                dxl_goal_torque = 0
            else:
                dxl_goal_torque = -K_s*(dxl_present_position_main_deg-dxl_position_offset)-K_d*(dxl_present_velocity)+gr_comp_torque # [Nm] Torque should point in opposite direction as displacement
                dxl_goal_current = 8.247191-8.247191*np.sqrt(1-0.082598*dxl_goal_torque)     # [A]
                dxl_goal_current = dxl_goal_current * 1000                                   # [mA]
                dxl_goal_current = round(dxl_goal_current / cur_unit)                        # [dxl units]
            
            # Write goal current
            if -max_current<dxl_goal_current<max_current:   # Check if goal current is within current limits
                with dxl_lock:
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, dxl_goal_current)
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
                dxl_goal_current = dxl_goal_current * cur_unit  # [mA]
            else:
                print("Goal current is too high.")
                break
            t2 = perf_counter() -t1  #Used for results printing timing
            

            if (t2>t_print):
                t1 = perf_counter()
                # Read present current 
                with dxl_lock:   
                    dxl_present_current, dxl_comm_result2, dxl_error2 = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_CURRENT)
                    if dxl_comm_result2 != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error2 != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
                b3 = dxl_present_current.to_bytes(2, byteorder=sys.byteorder, signed = False) 
                dxl_present_current = int.from_bytes(b3, byteorder=sys.byteorder, signed = True) # [dxl units]
                dxl_present_current = dxl_present_current * cur_unit # [mA]
                dxl_present_current = dxl_present_current/1000       # [A]
                dxl_present_torque =  2.936*dxl_present_current - 0.178*dxl_present_current**2 #[Nm]
                
                # Get force sensor data from Arduino
                if ser.in_waiting>0:                                    # number of bytes received
                    ard_line = ser.readline().decode('utf-8').rstrip()  # reads up to new line symbol # removes newline symbol
                ser.reset_input_buffer()                                # fresh buffer

                # Elapsed time from the start of the program
                t3 = -(t0-perf_counter())
                # Print results
                print("Gravity Comp - Exo: %.4f Gravity Comp - Arm: %.4f SpringCoeff: %.4f Nm/deg  DampingCoeff: %.4f Nm*s/deg  PresPos: %.4f deg  PresVel: %.4f deg/s  GoalTorque: %.4f Nm  PresTorque: %.4f Nm Time: %.4f s Loop Time: %.4f s " % (G_mech, G_arm, K_s, K_d, dxl_present_position_main_deg, dxl_present_velocity, dxl_goal_torque, dxl_present_torque, t3, t4))
                print(ard_line)
                if f_input!="":
                    results_file.write(f"{G_mech}, {G_arm}, {K_s:.4f}, {K_d:.4f}, {dxl_present_position_main_deg:.4f}, {dxl_present_velocity:.4f}, {dxl_goal_torque:.4f}, {dxl_present_torque:.4f}, {roll_1_main:.4f}, {pitch_main:.4f}, {t3:.4f}, {t4:.4f}, "+ard_line+ '\n') # Write results to file
                
    except KeyboardInterrupt:
        print("Loop ended.")

    finally:
        t_wd = perf_counter()

        # Close port
        portHandler.closePort()

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        portHandler = PortHandler(DEVICENAME)

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

        # Set goal current to 0
        dxl_goal_current = 0
        with dxl_lock:
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, dxl_goal_current)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print("Goal current is set to 0.")
        
        # Disable Dynamixel Torque
        with dxl_lock:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print("Torque is disabled.")
        
        # Close serial
        print('Close Serial Communication')
        ser.close()
    
        # Close results file
        if f_input!="":
            results_file.close()
        
        # Turn LED on
        led.off()
        sync_pin.off()






