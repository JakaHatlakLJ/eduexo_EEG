import os
import sys
from time import perf_counter
import numpy as np
from dynamixel_sdk import * 
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
            torque_limit = 8,
            min_pos = 55,
            max_pos = 180,
            CONTROL_MODE = 0,
            BAUDRATE = 1000000,
            loop_frequency = 200,
            DEVICENAME = '/dev/ttyUSB0'
            ):
        """
        max_torque - maximum torque allowed on motor [Nm]\n
        min_pos - torque is set to 0 before this motor position [deg]\n
        max_pos - torque is set to 0 after this motor position [deg]\n
        Threading Locks:\n
        - dxl_lock - for accessing motor's API\n
        - position_lock - for updating global "current_position" variable\n
        - velocity_lock - for updating global "current_velocity" variable\n
        - torque_lock - for updating global "current_torque" variable\n
        stop_event - Threading.Event(), for gracefully stoping threads
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
        self.CONTROL_MODE                   = CONTROL_MODE                                                          # Value for switching to current control mode
        self.BAUDRATE                       = BAUDRATE                          # [bps]                             # set BAUDRATE: 9600/57600/115200/1M/2M/3M/4M/4.5M
        self.BAUDRATE_VALUE                 = SetupEXO.baud_dict[BAUDRATE]
        self.loop_frequency                 = loop_frequency                    # [Hz]                              # set maximum frequency of the program loop

        # Bus Watchdog value
        self.watchdog_time = 5           # [1 unit = 20 ms]
        self.watchdog_clear = 0

        # Current limit
        self.torque_limit  = torque_limit
        current_limit = 8.247191-8.247191*np.sqrt(1-0.082598*torque_limit)          # [A]
        current_limit = current_limit * 1000.0                                      # [mA]
        self.current_limit = round(current_limit / SetupEXO.cur_unit)               # [dxl units]

        # Position offset and limits values
        self.min_pos = min_pos                          # [deg] Torque/current is set to 0 if limit is exceeded
        self.max_pos = max_pos                          # [deg] Torque/current is set to 0 if limit is exceeded
        self.mid_pos = (self.max_pos + self.min_pos)/2  # [deg] Midpoint, where trials start

        # Initialize Dynamixel and ports
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(SetupEXO.PROTOCOL_VERSION)

        # LOCKER INTIALIZATION
        self.dxl_lock = threading.Lock()
        self.stop_event = threading.Event()

        self.present_position_deg = 0
        self.present_velocity_deg = 0
        self.present_torque = 0

    def find_current_baudrate(self):
        """Function for finding current baudrate"""

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

    def set_baudrate(self, NEW_BAUDRATE, CURRENT_BAUDRATE = None, stop_system = False):
        """Function for setting BAUDRATE of Dynamixel"""

        NEW_BAUDRATE_VALUE = SetupEXO.baud_dict[NEW_BAUDRATE]

        if CURRENT_BAUDRATE != None and not stop_system:
            if self.portHandler.setBaudRate(CURRENT_BAUDRATE):
                dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, SetupEXO.DXL_ID)
                if dxl_comm_result == COMM_SUCCESS:
                    self.CURRENT_BAUDRATE = CURRENT_BAUDRATE
                    return self.CURRENT_BAUDRATE
                else:
                    print(f"Current baudrate is not {CURRENT_BAUDRATE}")
                    self.CURRENT_BAUDRATE = None
        else:
            self.CURRENT_BAUDRATE = None

        if self.CURRENT_BAUDRATE == None and not stop_system:
            print(f"Trying to find current baudrate")
            for baudrate in SetupEXO.baud_dict.keys():
                if self.portHandler.setBaudRate(baudrate):
                    # Try to ping the motor
                    dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, SetupEXO.DXL_ID)
                    if dxl_comm_result == COMM_SUCCESS:
                        print(f"Successfully connected at baudrate: {baudrate}")
                        self.CURRENT_BAUDRATE = baudrate
                        break
                else:
                    print(f"Failed to find current baudrate")
                    sys.exit()

        if NEW_BAUDRATE == self.CURRENT_BAUDRATE:
            return self.CURRENT_BAUDRATE
        else:
            # Set NEW BAUDRATE on Dynamixel
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_BAUDRATE, NEW_BAUDRATE_VALUE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                self.CURRENT_BAUDRATE = NEW_BAUDRATE
                print(f"Baudrate has been switched to {NEW_BAUDRATE} on Dynamixel.")

            # Set BAUDRATE to NEW BAUDRATE on RasPi
            if self.portHandler.setBaudRate(NEW_BAUDRATE):
                print(f"Succeeded to change the baudrate to {NEW_BAUDRATE} on RasPi")
            else:
                print("Failed to change the baudrate")
                sys.exit()

        return self.CURRENT_BAUDRATE

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
        self.set_baudrate(self.BAUDRATE)

        with self.dxl_lock:
            # Set desired control mode
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_OP_MODE, self.CONTROL_MODE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Operating mode has been switched to current control.")

            if self.CONTROL_MODE in (0,5):
                # Set the current limit
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_CURRENT_LIMIT, self.current_limit)
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
            
    def stop_system(self):
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
        self.set_baudrate(57600, self.CURRENT_BAUDRATE, stop_system=True)

        self.portHandler.closePort()

    def write_current(self, goal_cur):
        '''Functiong for writing goal current to motor'''
        if self.CONTROL_MODE not in (0,5):
            raise Exception(f"Can only write goal current in Current Control or Current-based Position Control Mode")        
        else:
            with self.dxl_lock:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_GOAL_CURRENT, goal_cur)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def write_position(self, goal_pos):
        '''Functiong for writing goal position to motor'''

        if self.CONTROL_MODE not in (3,5):
            raise Exception(f"Can only write desired position in Position Control or Current-based Position Control Mode")        
        else:
            with self.dxl_lock:
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_GOAL_POSITION, goal_pos)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def motor_data(self, position=False, velocity=False, torque=False):
        '''Function for reading current Position, Velocity and Torque of motor'''

        self.present_position_deg = 0
        self.present_velocity_deg = 0
        self.present_torque = 0

        previous_t = perf_counter()
        while not self.stop_event.is_set():
            try:
                current_t = perf_counter()

                if current_t - previous_t >= 1 / (1.1 * self.loop_frequency):
                    if position:
                        # Read present Position
                        with self.dxl_lock:
                            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_PRESENT_POSITION)
                            if dxl_comm_result != COMM_SUCCESS:
                                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                            elif dxl_error != 0:
                                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                        b1 = dxl_present_position.to_bytes(4, byteorder=sys.byteorder, signed = False) 
                        dxl_present_position = int.from_bytes(b1, byteorder=sys.byteorder, signed = True)
                        self.present_position_deg = float(dxl_present_position*SetupEXO.pos_unit)  

                    if velocity:
                        # Read present Velocity
                        with self.dxl_lock:
                            dxl_present_velocity, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_PRESENT_VELOCITY)
                            if dxl_comm_result != COMM_SUCCESS:
                                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                            elif dxl_error != 0:
                                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                        b2 = dxl_present_velocity.to_bytes(4, byteorder=sys.byteorder, signed = False) 
                        dxl_present_velocity = int.from_bytes(b2, byteorder=sys.byteorder, signed = True)
                        self.present_velocity_deg = float(dxl_present_velocity) * SetupEXO.vel_unit * 6.0     # [deg/s]

                    if torque:
                        # Read present Current
                        with self.dxl_lock:
                            dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, SetupEXO.DXL_ID, SetupEXO.ADDR_PRESENT_CURRENT)
                            if dxl_comm_result != COMM_SUCCESS:
                                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                            elif dxl_error != 0:
                                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                        b3 = dxl_present_current.to_bytes(2, byteorder=sys.byteorder, signed = False) 
                        dxl_present_current = int.from_bytes(b3, byteorder=sys.byteorder, signed = True)
                        self.present_torque = 0.082598 * (1 - (1 - dxl_present_current * SetupEXO.cur_unit / 8.247191)**2)    # [mNm]

                    previous_t = current_t

            except Exception as e:
                print(f'Error in motor_data: {e}')
                break

if __name__ == "__main__":
    print(f'reseting EXO')
    EXO = SetupEXO()
    _ = EXO.start_system()
    EXO.stop_system()