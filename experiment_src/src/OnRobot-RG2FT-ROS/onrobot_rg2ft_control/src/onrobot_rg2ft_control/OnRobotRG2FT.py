#!/usr/bin/env python3

import logging
from onrobot_rg2ft_control.OnRobotTcpClient import OnRobotTcpClient
from onrobot_rg2ft_msgs.msg import RG2FTCommand, RG2FTState
import struct

RG2FT_MIN_WIDTH = 0
RG2FT_MAX_WIDTH = 1000
RG2FT_MIN_FORCE = 0
RG2FT_MAX_FORCE = 400
RG2FT_DEVICE_ADDRESS = 65

def s16(val):
    return struct.unpack('h', struct.pack('H', val))[0]

def u16(val):
    return struct.unpack('H', struct.pack('h', val))[0]

class OnRobotRG2FT:

    def __init__(self, ip, port):
        self.client = OnRobotTcpClient()
        self.client.connect(ip, port, RG2FT_DEVICE_ADDRESS)
        self._logger = logging.getLogger(__name__)

    def write_command(self, cmd: RG2FTCommand):
        if cmd.target_width < RG2FT_MIN_WIDTH or cmd.target_width > RG2FT_MAX_WIDTH:
            self._logger.error("[OnRobotRG2FT] target_width out of range")
            return

        if cmd.target_force < RG2FT_MIN_FORCE or cmd.target_force > RG2FT_MAX_FORCE:
            self._logger.error("[OnRobotRG2FT] target_force out of range")
            return

        if cmd.control not in [0, 1]:
            self._logger.error("[OnRobotRG2FT] control is not 0 or 1")
            return

        message = [0] * 3
        message[0] = int(cmd.target_force)
        message[1] = int(cmd.target_width)
        message[2] = int(cmd.control)

        self.client.write(address=2, message=message)

    def read_state(self) -> RG2FTState:
        resp = self.client.read(address=257, count=26)

        msg = RG2FTState()

        msg.status_l = resp[0]
        msg.fx_l = s16(resp[2])
        msg.fy_l = s16(resp[3])
        msg.fz_l = s16(resp[4])
        msg.tx_l = s16(resp[5])
        msg.ty_l = s16(resp[6])
        msg.tz_l = s16(resp[7])
        msg.status_r = resp[9]
        msg.fx_r = s16(resp[11])
        msg.fy_r = s16(resp[12])
        msg.fz_r = s16(resp[13])
        msg.tx_r = s16(resp[14])
        msg.ty_r = s16(resp[15])
        msg.tz_r = s16(resp[16])
        msg.proximity_status_l = resp[17]
        msg.proximity_value_l = s16(resp[18])
        msg.proximity_status_r = resp[20]
        msg.proximity_value_r = s16(resp[21])
        msg.actual_gripper_width = s16(resp[23])
        msg.gripper_busy = resp[24]
        msg.grip_detected = resp[25]

        return msg

    def set_proximity_offsets(self, left_offset, right_offset):
        message = [0] * 2
        message[0] = int(left_offset)
        message[1] = int(right_offset)
        self.client.write(address=5, message=message)

    def zero_force_torque(self, val):
        message = [int(bool(val))]
        self.client.write(address=0, message=message)

    def restart_power_cycle(self):
        self.client.restart_power_cycle()

    # Backward-compatible aliases (ROS 1 names)
    def writeCommand(self, cmd: RG2FTCommand):
        return self.write_command(cmd)

    def readState(self) -> RG2FTState:
        return self.read_state()

    def setProximityOffsets(self, left_offset, right_offset):
        return self.set_proximity_offsets(left_offset, right_offset)

    def zeroForceTorque(self, val):
        return self.zero_force_torque(val)

    def restartPowerCycle(self):
        return self.restart_power_cycle()
