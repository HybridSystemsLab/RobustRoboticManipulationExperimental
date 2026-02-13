#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .OnRobotRG2FT import OnRobotRG2FT
from std_srvs.srv import Trigger, SetBool
from onrobot_rg2ft_msgs.srv import SetProximityOffsets
from onrobot_rg2ft_msgs.msg import RG2FTCommand, RG2FTState
from geometry_msgs.msg import Wrench


class OnRobotRG2FTDriver(Node):

    def __init__(self):
        super().__init__('onrobot_rg2ft_driver')

        ip = self.declare_parameter('ip', '192.168.1.1').get_parameter_value().string_value
        port = self.declare_parameter('port', '502').get_parameter_value().string_value

        self.gripper = OnRobotRG2FT(ip, port)

        # Topics align with action server defaults
        self.state_pub = self.create_publisher(RG2FTState, 'gripper/states', 10)
        self.left_wrench_pub = self.create_publisher(Wrench, 'left_wrench', 10)
        self.right_wrench_pub = self.create_publisher(Wrench, 'right_wrench', 10)

        self.cmd_sub = self.create_subscription(RG2FTCommand, 'gripper/ctrl', self.gripper.write_command, 10)

        self.restart_srv = self.create_service(Trigger, 'restart', self.restart_cb)
        self.zero_srv = self.create_service(SetBool, 'zero_force_torque', self.zero_force_torque_cb)
        self.set_prox_offset_srv = self.create_service(SetProximityOffsets, 'set_proximity_offsets', self.prox_offsets_cb)

        self.timer = self.create_timer(0.01, self.spin_once)

    def spin_once(self):
        state = self.gripper.read_state()
        self.state_pub.publish(state)
        self.publish_wrenches(state)

    def restart_cb(self, req, resp):
        self.get_logger().info('Restarting the power cycle of all grippers connected.')
        self.gripper.restart_power_cycle()
        resp.success = True
        resp.message = 'Power cycle restarted.'
        return resp

    def zero_force_torque_cb(self, req, resp):
        self.get_logger().info('Zeroing force and torque values to cancel any offset')
        self.gripper.zero_force_torque(req.data)
        resp.success = True
        resp.message = 'Zero command sent.'
        return resp

    def prox_offsets_cb(self, req, resp):
        self.get_logger().info('Setting proximity offsets.')
        self.gripper.set_proximity_offsets(req.proximity_offset_l, req.proximity_offset_r)
        resp.success = True
        resp.message = 'Proximity offsets set.'
        return resp

    def publish_wrenches(self, state: RG2FTState):
        left_wrench = Wrench()
        left_wrench.force.x = state.fx_l / 10.0
        left_wrench.force.y = state.fy_l / 10.0
        left_wrench.force.z = state.fz_l / 10.0
        left_wrench.torque.x = state.tx_l / 100.0
        left_wrench.torque.y = state.ty_l / 100.0
        left_wrench.torque.z = state.tz_l / 100.0

        right_wrench = Wrench()
        right_wrench.force.x = state.fx_r / 10.0
        right_wrench.force.y = state.fy_r / 10.0
        right_wrench.force.z = state.fz_r / 10.0
        right_wrench.torque.x = state.tx_r / 100.0
        right_wrench.torque.y = state.ty_r / 100.0
        right_wrench.torque.z = state.tz_r / 100.0

        self.left_wrench_pub.publish(left_wrench)
        self.right_wrench_pub.publish(right_wrench)


def main(args=None):
    rclpy.init(args=args)
    node = OnRobotRG2FTDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
