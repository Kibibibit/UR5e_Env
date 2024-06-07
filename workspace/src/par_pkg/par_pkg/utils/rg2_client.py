from pymodbus.client.sync import ModbusTcpClient as ModbusClient
import rclpy.logging as logger
from .colour_string import *
import math

MAX_WIDTH = 110.0
MAX_FORCE = 40.0
DEFAULT_FORCE = 10.0


TARGET_FORCE_REGISTER         = 0x0000
TARGET_WIDTH_REGISTER         = 0x0001
CONTROL_REGISTER              = 0x0002
FINGERTIP_OFFSET_REGISTER     = 0x0102
DEPTH_REGISTER                = 0x0107
RELATIVE_DEPTH_REGISTER       = 0x0108
WIDTH_REGISTER                = 0x010B
STATUS_REGISTER               = 0x010C
WIDTH_WITH_OFFSET_REGISTER    = 0x0113
SET_FINGERTIP_OFFSET_REGISTER = 0x0407


class RG2ControlMode:
    CONTROL_MODE_GRIP:int = 0x0001
    CONTROL_MODE_STOP:int = 0x0008
    CONTROL_MODE_GRIP_W_OFFSET:int = 0x0010


class RG2Client():

    

    def __init__(
            self, 
            ip: str, 
            port: str|int=502, 
            stopbits:int=1, 
            bytesize:int=8, 
            parity:str='E', 
            baudrate:int=115200, 
            timeout:int=1
        ):

        self.ip = ip
        self.port = port
        self.connected = False

        self.__logger_name = f"RG2@{ip}:{port}"

        self.__client: ModbusClient = ModbusClient(
            str(ip),
            port=int(port),
            stopbits=stopbits,
            bytesize=bytesize,
            parity=parity,
            baudrate=baudrate,
            timeout=timeout
        )
        self.max_width = MAX_WIDTH
        """The maximum width the gripper can open to in milimetres. For the RG2, this is 110mm"""
        self.max_force = MAX_FORCE
        """The maximum force the gripper can exert, in newtons. For the RG2, this is 40N"""
    
    def __log(self, message:str) -> None:
        logger.get_logger(self.__logger_name).info(message)
    
    def __warn(self, message:str) -> None:
        logger.get_logger(self.__logger_name).warn(colour_string(ANSIColour.YELLOW, message))

    def __error(self, message:str) -> None:
        logger.get_logger(self.__logger_name).error(colour_string(ANSIColour.RED, message))

    def open_connection(self)->bool:
        """Opens a connection to the gripper. Returns True on a succesful connection, or if a gripper
        is already connected."""
        if (self.connected):
            self.__warn("Already connected to gripper@{self.ip}:{self.port}!")
            return True
        try:
            self.__log(colour_string(ANSIColour.BLUE,f"Attempting to connect to gripper@{self.ip}:{self.port}..."))
            self.__client.connect()
            self.__log(colour_string(ANSIColour.GREEN,"Gripper connected successfully!"))
            self.connected = True
            return True
        except Exception as e:
            self.__error("Failed to connect to gripper with exception: " +e)
            return False
    
    def close_connection(self):
        """Closes the modbus connection to the gripper"""
        if (self.connected):
            self.__log(colour_string(ANSIColour.BLUE,f"Disconnecting from gripper@{self.ip}:{self.port}..."))
            self.__client.close()
        else:
            self.__warn("No gripper connected!")
    
    def __valid_force(self, force:float|int):
        if (force < 0.0 or force > MAX_FORCE):
            self.__error(f"Force value of {force} is outside valid range [0.0, {MAX_FORCE}]")
            return False
        return True

    def __valid_width(self, width:float|int):
        if (width < 0.0 or width > MAX_WIDTH):
            self.__error(f"Width value of {width} is outside valid range [0.0, {MAX_WIDTH}]")
            return False
        return True

    def __read_int_register(self, address:int) -> float:
        return self.__read_register(address)/10.0
    
    def __read_register(self, address:int) -> int:
        if (self.connected):
            result = self.__client.read_holding_registers(
                address=address, count=1, unit=65)
            return result.registers[0]
        else:
            self.__warn("Tried to read when gripper not connected!")
            return None

    def __set_int_register(self, address:int, value:float|int) -> None:
        self.__set_register(address, math.floor(value*10.0))

    def __set_register(self, address:int, value):
        if (self.connected):
            self.__client.write_register(
                address=address, value=value, unit=65)
        else:
            self.__warn("Tried to set when gripper not connected!")

    def set_target_force(self, force:float|int):
        if (not self.__valid_force(force)):
            return
        self.__set_int_register(TARGET_FORCE_REGISTER, force)
    
    def set_target_width(self, width:float|int):
        if (not self.__valid_width(width)):
            return
        self.__set_int_register(TARGET_WIDTH_REGISTER, width)

    def set_control_mode(self, control_mode:int):
        if (not control_mode in [RG2ControlMode.CONTROL_MODE_GRIP, RG2ControlMode.CONTROL_MODE_GRIP_W_OFFSET, RG2ControlMode.CONTROL_MODE_STOP]):
            self.__error("Invalid control mode! Must be one of CONTROL_MODE_GRIP, CONTROL_MODE_GRIP_W_OFFSET or CONTROL_MODE_STOP!")
            return
        self.__set_register(CONTROL_REGISTER, control_mode)

    def get_fingertip_offset(self) -> float:
        return self.__read_int_register(FINGERTIP_OFFSET_REGISTER)
    
    def set_fingertip_offset(self, offset: float|int) -> None:
        self.__set_int_register(SET_FINGERTIP_OFFSET_REGISTER, offset)
    
    def get_actual_depth(self) -> float:
        return self.__read_int_register(DEPTH_REGISTER)

    def get_relative_depth(self) -> float:
        return self.__read_int_register(RELATIVE_DEPTH_REGISTER)

    def get_width(self) -> float:
        return self.__read_int_register(WIDTH_REGISTER)

    def get_width_with_offset(self) -> float:
        return self.__read_int_register(WIDTH_WITH_OFFSET_REGISTER)

    def get_busy(self) -> bool:
        return self.get_status()[0] != 0

    def get_status(self) -> list[int]:
        result = self.__read_register(STATUS_REGISTER)
        status = format(result, '016b')
        status_list = [0] * 7
        if int(status[-1]):
            #print("A motion is ongoing so new commands are not accepted.")
            status_list[0] = 1
        if int(status[-2]):
            #print("An internal- or external grip is detected.")
            status_list[1] = 1
        if int(status[-3]):
            #print("Safety switch 1 is pushed.")
            status_list[2] = 1
        if int(status[-4]):
            #print("Safety circuit 1 is activated so it will not move.")
            status_list[3] = 1
        if int(status[-5]):
            #print("Safety switch 2 is pushed.")
            status_list[4] = 1
        if int(status[-6]):
            #print("Safety circuit 2 is activated so it will not move.")
            status_list[5] = 1
        if int(status[-7]):
            #print("Any of the safety switch is pushed.")
            status_list[6] = 1

        return status_list

    def move_gripper(self, target_width, target_force=DEFAULT_FORCE, control_mode = RG2ControlMode.CONTROL_MODE_GRIP) -> None:
        if (self.get_busy()):
            self.__warn("Tried to move the gripper while it was busy!")
            return
        self.set_target_force(target_force)
        self.set_target_width(target_width)
        self.set_control_mode(control_mode)
        self.__log(f"Setting gripper@{self.ip}:{self.port} to width of {target_width} with force {target_force}")
    
    def move_gripper_with_offset(self, target_width, target_force=DEFAULT_FORCE) -> None:
        self.move_gripper(target_width, target_force, RG2ControlMode.CONTROL_MODE_GRIP_W_OFFSET)
    
    def open_gripper(self, target_force=DEFAULT_FORCE) -> None:
        self.move_gripper(self.max_width, target_force)
    
    def close_gripper(self, target_force=DEFAULT_FORCE) -> None:
        self.move_gripper(0.0, target_force)

    def stop_gripper(self) -> None:
        self.set_control_mode(RG2ControlMode.CONTROL_MODE_STOP)
