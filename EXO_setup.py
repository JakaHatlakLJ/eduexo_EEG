import os
import sys
from time import perf_counter
import numpy as np
from dynamixel_sdk import * 
from pylsl import StreamInlet, resolve_streams, StreamInfo, StreamOutlet
from gpiozero import LED
import threading


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

class SetupEXO:
    """
    Class for initializing and seting up the comunication of Dynamixel-XM540-W270-T/R motor with RasPi
    """

    # BAUDRATE dictionary
    baud_dict = {9600 : 0, 57600 : 1, 115200 : 2, 1000000 : 3, 2000000 : 4, 3000000 : 5, 4000000 : 6, 4500000 : 7}

    # DYNAMIXEL SETTINGS
    ADDR_TORQUE_ENABLE          = 64
    ADDR_OP_MODE                = 11
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    ADDR_PRESENT_CURRENT        = 126
    ADDR_PRESENT_VELOCITY       = 128
    ADDR_GOAL_CURRENT           = 102
    ADDR_BAUDRATE               = 8
    DEFAULT_BAUDRATE            = 57600
    PROTOCOL_VERSION            = 2.0
    DXL_ID                      = 1
    ADDR_CURRENT_LIMIT          = 38
    ADDR_WATCHDOG               = 98

    TORQUE_ENABLE               = 1
    TORQUE_DISABLE              = 0
    DXL_MOVING_STATUS_THRESHOLD = 10

    # Unit conversion
    pos_unit = 0.088            # [deg] = [dxl_unit] * [pos_unit]
    vel_unit = 0.229            # [rpm] = [dxl_unit] * [vel_unit]
    cur_unit = 2.69             # [mA]  = [dxl_unit] * [cur_unit]

    # Initial settings (later replaced by user input values)
    K_s = 0.07                  # [Nm/deg] Initial spring coefficient
    K_d = 0.006

    #setup LED
    led = LED(27)

    def __init__(
            self,
            max_torque=8,
            min_pos=55,
            max_pos=180,
            dxl_lock = threading.Lock(),
            CONTROL_MODE=0,
            BAUDRATE=1000000,
            loop_frequency=200,
            watchdog_time=5,
            DEVICENAME='/dev/ttyUSB0'
            ):
        """
        Define Dynamixel parameters and limitations:\n
        max_torque - Maximum torque allowed on motor [Nm]\n
        min_pos - torque is set to 0 before this motor position [deg]\n
        max_pos - torque is set to 0 after this motor position [deg]\n
        dxl_lock - threading.Lock() used for writing goal current on motor
        CONTROL_MODE - operating mode of Dynamixel motor (https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/):\n
        - 0: Current control\n
        - 1: Velocity control\n
        - 3: Position control\n
        - 4: Extended position control (Multi-turn)\n
        - 5: Current-based Position control\n
        - 16: PWM control\n
        BAUDRATE - 9600/57600/115200/1000000/2000000/3000000/4000000/4500000\n
        loop_frequency - limit of control frequency for improved consistency
        watchdog_time - bus watchdog checking time [1 unit = 20 msec]
        DEVICENAME - device name on RasPi

        """

        #### Define control mode and BAUDRATE
        self.CONTROL_MODE             = CONTROL_MODE                                                            # Value for switching to current control mode
        self.BAUDRATE                    = BAUDRATE                             # [bps]                         # set BAUDRATE: 9600/57600/115200/1M/2M/3M/4M/4.5M
        self.BAUDRATE_VALUE              = SetupEXO.baud_dict[BAUDRATE]
        self.loop_frequency              = loop_frequency                       # [Hz]                          # set maximum frequency of the program loop

        # Bus Watchdog value
        self.watchdog_time = watchdog_time           # [1 = 20 ms]
        self.watchdog_clear = 0

        # Current limit
        self.max_torque  = max_torque
        max_current = 8.247191-8.247191*np.sqrt(1-0.082598*max_torque)          # [A]
        max_current = max_current * 1000.0                                      # [mA]
        self.max_current = round(max_current / SetupEXO.cur_unit)               # [dxl units]

        # Position offset and limits values
        self.min_pos = min_pos                  # [deg] Torque/current is set to 0 if limit is exceeded
        self.max_pos = max_pos                  # [deg] Torque/current is set to 0 if limit is exceeded

        # Initialize Dynamixel and ports
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(SetupEXO.PROTOCOL_VERSION)

        # LOCKER INTIALIZATION
        self.dxl_lock = dxl_lock

    # Function to find current baudrate
    def find_current_baudrate(self):
        print(f"Trying to find current baudrate")
        for baudrate in SetupEXO.baud_dict.keys():
            if self.portHandler.setBaudRate(baudrate):
                # Try to ping the motor
                dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, SetupEXO.DXL_ID)
                if dxl_comm_result == COMM_SUCCESS:
                    print(f"Successfully connected at baudrate: {baudrate}")
                    return baudrate
            else:
                print(f"Failed to set baudrate: {baudrate}")
        return None

    # function for setting BAUDRATE of Dynamixel
    def set_baudrate(self,NEW_BAUDRATE_VALUE, NEW_BAUDRATE, CURRENT_BAUDRATE = None, start_system = True):
        if CURRENT_BAUDRATE == None:
            CURRENT_BAUDRATE = self.find_current_baudrate()

        # Set NEW BAUDRATE on Dynamixel
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_BAUDRATE, NEW_BAUDRATE_VALUE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Baudrate has been switched to {NEW_BAUDRATE} on Dynamixel.")

        if start_system:
            # Set BAUDRATE to NEW BAUDRATE on RasPi
            if self.portHandler.setBaudRate(NEW_BAUDRATE):
                print(f"Succeeded to change the baudrate to {NEW_BAUDRATE} on RasPi")
            else:
                print("Failed to change the baudrate")
                sys.exit()

            return CURRENT_BAUDRATE

    #Function to initalize Dynamixel Motor
    def start_system(self):
        '''Function for seting up motor:
            - Open port
            - Set Baudrate
            - Set operating mode to current control
            - Set current limit
            - Clear Bus Watchdog'''

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            sys.exit()

        # Set NEW BAUDRATE
        self.set_baudrate(self.BAUDRATE_VALUE, self.BAUDRATE)

        with self.dxl_lock:
            # Set operating mode to current control
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_OP_MODE, self.CONTROL_MODE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Operating mode has been switched to current control.")

            # Set the current limit
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_CURRENT_LIMIT, self.max_current)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Current limit has been set.")

            # Clear Bus Watchdog
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_WATCHDOG, self.watchdog_clear)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"Watchdog is cleared.")

    def torque_watchdog(self):
        '''Function for enabling Torque and setting Bus Watchdog'''       
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_TORQUE_ENABLE, SetupEXO.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Torque is enabled.")
            SetupEXO.led.on() #Turn LED ON

        # Enable Bus Watchdog
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_WATCHDOG, self.watchdog_time)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Watchdog is set to {self.watchdog_time*20} ms.")
            
    def stop_system(self, CURRENT_BAUDRATE):
        '''Function for stoping motor safely:
            - Disable Bus Watchdog
            - Set goal Current to 0
            - Disable torque'''    
        # Clear Bus Watchdog
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_WATCHDOG, self.watchdog_clear)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Watchdog is disabled.")

        # Set goal current to 0
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_GOAL_CURRENT, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Goal current is set to 0.")

        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_TORQUE_ENABLE, SetupEXO.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Torque is disabled.")
            SetupEXO.led.off()# Turn LED off

        # reset BAUDRATE back to default value
        self.set_baudrate(1, DEFAULT_BAUDRATE, CURRENT_BAUDRATE, start_system=False)

    def write_current(self, goal_cur):
        '''Functiong for writing goal current to motor'''
        with self.dxl_lock:
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_GOAL_CURRENT, goal_cur)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))