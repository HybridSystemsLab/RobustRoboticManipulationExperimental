#!/usr/bin/env python3

import math
from statistics import mean
import time
import os
import sys
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import rclpy.node
from onrobot_rg2ft_control.OnRobotRG2FT import OnRobotRG2FT
from onrobot_rg2ft_msgs.msg import RG2FTCommand, RG2FTState
import threading
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState
import requests

OUTSOURCE_IP = "127.0.0.1" 

class PNS_Driver:
    def __init__(self, node, ip, port):
        self.node = node
        self.gripper = OnRobotRG2FT(ip, port)
        self.state_pub = node.create_publisher(RG2FTState, "state", 1)
        self.cmd_sub = node.create_subscription(RG2FTCommand, "command", self.gripper.writeCommand, 1)
        self.fd = 0
        self.done = True
        self.shutdown = False

        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def callback(self, data):
        self.node.get_logger().info(f"I heard {data.data}")

    def set_fd(self, force):
        self.fd = force
        self.done = False

    def get_done(self):
        return self.done

    def stop(self):
        self.shutdown = True
        self.thread.join()

    def loop(self):
        LOOP_FREQ = 100 # Hz
        PERIOD = 1.0 / LOOP_FREQ
        force_avg = 0
        # movement = 200 # for berry
        movement = 300 # for ball
        q = 0 # lambda state variable for gripper control
        desired_force = 0 # ONLY for beginning the loop
        last_prox = 1000

        MIN_HOLD_TIME = 300 # hold for 30 minutes -> testing temperature sensor drift
        MAX_GRIP_TIME = 300  # seconds -> adjust for altering data collection amount

        # OPEN_WIDTH = 1000 # mm; max opening width -> if smaller than 300 mm diameter, change open width to 400 mm

        OPEN_WIDTH = 670 # only for water bottle experiment

        # states for gripper control
        TIGHTEN_FAST = 3
        TIGHTEN = 2
        TIGHTEN_SLOW = 1
        HOLD = 0
        LOOSEN_SLOW = -1
        LOOSEN = -2

        FAR = 500
        CLOSE = 300 

        MOVING_AVG_LEN_FORCE = 100
        MOVING_AVG_LEN_PROX = 100
        MOVING_AVG_LEN_K = 100

        SLOW_FORCE_BOUND = 0.5

        # hysteresis thresholds
        # DELTA2 = 0.3
        # DELTA1 = 0.1

        # no hysteresis
        DELTA2 = 0.0
        DELTA1 = 0.0

        # speeds are optional, they just determine how fast the gripper moves when grasping. not necessary for experiment
        SPEED_FAST = 1.0
        SPEED_NORMAL = 0.05 * 2
        SPEED_SLOW = 0.005

        OPEN_HOLD_TOLERANCE = 5

        prox_l_range = []
        prox_r_range = []
        width_range = []
        force_range = []

        last_target = 0

        reached_hold_time = float("inf")
        start_time = float("inf")

        k_filter = []
        time_data = []
        hold_time_data = []
        force_data = []
        fd_data = []
        state_data = []
        width_data = []
        cmd_width_data = []
        prox_data = []
        raw_fz_l_data = []
        raw_fz_r_data = []

        CALIBRATION_TIME = 5 # seconds

        calibrated = False
        l_force_bias = 0.0
        r_force_bias = 0.0
        l_force_drift = 0.0   # per loop increment
        r_force_drift = 0.0   # per loop increment
        # baseline_loop_counter = 0  # loop index at last calibration/rebaseline

        tmp_width = 0

        loop_counter = 0
        while rclpy.ok() and not self.shutdown:
            loop_start_time = time.time()
            state = self.gripper.readState()

            if self.fd != desired_force:
                desired_force = self.fd
                reached_hold_time = float("inf")
                start_time = time.time()

            cmd = RG2FTCommand()
            tmp_width = last_target # mm
            cmd.target_force = movement # target force is how aggressive the gripper moves, NOT the actual force applied
            cmd.control = 1

            prox_l_range.append(state.proximity_value_l)
            prox_r_range.append(state.proximity_value_r)
            width_range.append(state.actual_gripper_width)

            if len(prox_l_range) > MOVING_AVG_LEN_PROX:
                prox_l_range.pop(0)
                prox_r_range.pop(0)
                width_range.pop(0)

            ProxL = mean(prox_l_range)
            ProxR = mean(prox_r_range)
            width = mean(width_range)

            ProxAvg = (ProxL + ProxR) / 2 # divide by two due to width between two fingers (find midpoint)

            if mean([state.proximity_value_l, state.proximity_value_r]) - last_prox > CLOSE:
                desired_force = 0
                self.fd = 0
                self.done = False

            last_prox = ProxAvg

            # Raw force readings
            l_force_raw = abs(state.fz_l)
            r_force_raw = abs(state.fz_r)

            l_force = l_force_raw - l_force_bias - l_force_drift * loop_counter
            r_force = r_force_raw - r_force_bias - r_force_drift * loop_counter

            force = (l_force_raw + r_force_raw) / 2 / 10
            
            force_range.append(force)
            if len(force_range) > MOVING_AVG_LEN_FORCE:
                force_range.pop(0)
            force_avg = mean(force_range)

            force_error = desired_force - force_avg

            if desired_force == 0:
                tmp_width = OPEN_WIDTH
                q = HOLD
                if abs(width - tmp_width) < OPEN_HOLD_TOLERANCE:
                    reached_hold_time = min(reached_hold_time, time.time())
            else:
                if ProxAvg > FAR and force_avg <= desired_force - desired_force * SLOW_FORCE_BOUND - DELTA2:
                    q = TIGHTEN_FAST
                elif ProxAvg < FAR and ProxAvg > CLOSE and force_avg <= desired_force - desired_force * SLOW_FORCE_BOUND - DELTA2:
                    q = TIGHTEN_FAST
                else:
                    if (q == TIGHTEN or q == TIGHTEN_SLOW or q == TIGHTEN_FAST) and (force_error <= (DELTA1)):
                        reached_hold_time = time.time()
                        q = HOLD
                    elif (q == HOLD) and (force_error >= (DELTA2)):
                        q = TIGHTEN
                    elif (q == HOLD) and (force_error <= (-1 * DELTA2)):
                        q = LOOSEN
                    elif (q == LOOSEN or q == LOOSEN_SLOW) and (force_error >= (-1 * DELTA1)):
                        reached_hold_time = time.time()
                        q = HOLD

                if q != HOLD:
                    reached_hold_time = float("inf")
        
                if q == HOLD:
                    pass
                elif q == TIGHTEN:
                    tmp_width -= SPEED_NORMAL
                elif q == LOOSEN:
                    tmp_width += SPEED_NORMAL
                elif q == TIGHTEN_FAST:
                    tmp_width -= SPEED_FAST
                elif q == LOOSEN_SLOW:
                    tmp_width += SPEED_SLOW
                elif q == TIGHTEN_SLOW:
                    tmp_width -= SPEED_SLOW
                
                if tmp_width < 0:
                    tmp_width = 0
                elif tmp_width > 1000:
                    tmp_width = 1000

                self.node.get_logger().info(f"prox: {ProxAvg:.2f}, target width: {tmp_width:.2f}, current force: {force_avg:.2f}, force: {force:.2f}, q state: {q}, raw fz_l: {l_force_raw:.2f}, raw fz_r: {r_force_raw:.2f}")

            if calibrated:
                time_data.append(time.time())
                hold_time_data.append(0 if reached_hold_time > time.time() else reached_hold_time)
                force_data.append(force_avg)
                fd_data.append(desired_force)
                state_data.append(q)
                width_data.append(width)
                cmd_width_data.append(last_target)
                prox_data.append(ProxAvg)
                raw_fz_l_data.append(l_force_raw)
                raw_fz_r_data.append(r_force_raw)

            cmd.target_width = int(round(tmp_width))
            cmd.target_force = int(round(cmd.target_force))

            if (desired_force == 0 and not self.done) or cmd.target_width != int(round(last_target)):
                self.gripper.writeCommand(cmd)
                if cmd.target_width == OPEN_WIDTH and not calibrated:
                    time.sleep(1)
                    self.node.get_logger().info("Starting gripper force calibration...")
                    f_l_arr = []
                    f_r_arr = []
                    for _ in range(MOVING_AVG_LEN_FORCE):
                        state = self.gripper.readState()
                        f_l_arr.append(abs(state.fz_l))
                        f_r_arr.append(abs(state.fz_r))
                        time.sleep(PERIOD)

                    l_force_bias = mean(f_l_arr)
                    r_force_bias = mean(f_r_arr)
                    
                    time.sleep(CALIBRATION_TIME)
                    
                    f_l_arr = []
                    f_r_arr = []
                    for _ in range(MOVING_AVG_LEN_FORCE):
                        state = self.gripper.readState()
                        f_l_arr.append(abs(state.fz_l))
                        f_r_arr.append(abs(state.fz_r))
                        time.sleep(PERIOD)

                    l_force_drift = (mean(f_l_arr) - l_force_bias) / ((CALIBRATION_TIME + PERIOD * MOVING_AVG_LEN_FORCE) * LOOP_FREQ)
                    r_force_drift = (mean(f_r_arr) - r_force_bias) / ((CALIBRATION_TIME + PERIOD * MOVING_AVG_LEN_FORCE) * LOOP_FREQ)
                    self.node.get_logger().info(f"Force calibration complete! l_force_drift: {l_force_drift:.6f} per loop, r_force_drift: {r_force_drift:.6f} per loop\n Bias left: {l_force_bias:.6f}, Bias right: {r_force_bias:.6f}\n")
                    calibrated = True

            last_target = tmp_width

            if not self.done and desired_force == self.fd and (time.time() - reached_hold_time > MIN_HOLD_TIME or time.time() - start_time > MAX_GRIP_TIME or desired_force == 0):
                self.done = True

            duration = time.time() - loop_start_time
            if duration < PERIOD:
                time.sleep(PERIOD - duration)
            
            loop_counter += 1

        if not os.path.exists("data"):
            os.makedirs("data")
        f_idx = 0
        while os.path.exists(f"data/hybrid_gripper_{f_idx}.csv"):
            f_idx += 1
        with open(f"data/hybrid_gripper_{f_idx}.csv", "w") as f:
            f.write("Time,ReachedHoldTime,Force,DesiredForce,State,ActualWidth,CommandWidth,Proximity,RawFzL,RawFzR\n")
            for i in range(len(time_data)):
                f.write(f"{time_data[i]},{hold_time_data[i]},{force_data[i]},{fd_data[i]},{state_data[i]},{width_data[i]},{cmd_width_data[i]},{prox_data[i]},{raw_fz_l_data[i]},{raw_fz_r_data[i]}\n")
        self.node.get_logger().info(f"Data saved to data/hybrid_gripper_{f_idx}.csv")

class CmdMove(object):
    def __init__(self, node):
        self.node = node
        self.joints_sub = node.create_subscription(JointState, "/elfin_arm_controller/controller_state", self.joints_callback, 1)
        self.joints_pub = node.create_publisher(JointState, "joint_goal", 1)
        self.ompl_planning_service = node.create_client(GetMotionPlan, "/ompl_planning_service")

        self.joint_min_limits = [-3.14, -2.04, -2.61, -3.14, -2.56, -3.14]
        self.joint_max_limits = [3.14, 2.04, 2.61, 3.14, 2.56, 3.14]
        self.joint_state = None

    def joints_callback(self, data):
        self.joint_state = data.position
        self.node.get_logger().info(f"Received joint state: {self.joint_state}")
        self.node.get_logger().info(f"Received joint state: {self.joint_state}")

    def dance(self, joints):
        self.node.get_logger().info(f"Attempted joint state: {joints}")
        joints = np.mod(joints, 2 * np.pi)
        joints = np.array([(j - 2 * np.pi) if j > np.pi else j for j in joints])
        joints = np.clip(joints, self.joint_min_limits, self.joint_max_limits)
        self.node.get_logger().info(f"Final joint state: {joints}")
        js = JointState()
        js.name = [
            "elfin_joint1",
            "elfin_joint2",
            "elfin_joint3",
            "elfin_joint4",
            "elfin_joint5",
            "elfin_joint6",
        ]
        js.position = [float(j) for j in joints]
        js.header.stamp = self.node.get_clock().now().to_msg()
        time.sleep(0.5)
        self.joints_pub.publish(js)
        count = 0
        while count < 100:
            if self.joint_state is not None:
                joint_state_np = np.array(self.joint_state)
                if np.allclose(joint_state_np, joints, atol=1e-1):
                    self.node.get_logger().info("Goal reached")
                    return True
            else:
                if count % 10 == 0:
                    self.node.get_logger().info("Waiting for previous goal to be reached...")
                count += 1
                time.sleep(0.1)
        return False

    def move(self, target_pose, target_orientation, previous_joint_state=None):
        if target_pose is None:
            return [0, 0, 0, 0, 0, 0]
        try:
            while self.joint_state is None and previous_joint_state is None:
                self.node.get_logger().info("Waiting for joint state...")
                time.sleep(1)
            
            if previous_joint_state is not None:
                theta_vals = np.array(previous_joint_state)
            else:
                theta_vals = np.array(self.joint_state)
            response = requests.get(
                    f"http://{OUTSOURCE_IP}:5001/move",
                    json={
                        "position": target_pose,
                        "orientation": target_orientation,
                        "starting_joint_state": theta_vals.tolist(),
                    },
                )
            if response.status_code == 200:
                self.node.get_logger().info(f"{response.text}")
                joints = response.json()["joints"]
                return joints
            else:
                self.node.get_logger().warn(
                    f"Computation failed with status code: {response.status_code}"
                )
        except requests.exceptions.RequestException as e:
            self.node.get_logger().warn(f"Computation failed: {e}")
        return []

if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("hybrid_node")
    ip = node.declare_parameter("ip", "192.168.1.1").get_parameter_value().string_value
    port = node.declare_parameter("port", "502").get_parameter_value().string_value

    cm = CmdMove(node)
    driver = PNS_Driver(node, ip, port)
    time.sleep(1)

    MOVE = 0
    GRIP = 1

    actions = [
        (GRIP, 0),
        # (MOVE, None, None),
        # (MOVE, [-0.100, 0.520, 0.400], [-90, -112, 0]),
        # (MOVE, [-0.100, 0.620, 0.400], [-90, -112, 0]),
        (GRIP, 1.5), # berry
        # (GRIP, 3), # ball
        # (MOVE, [-0.100, 0.620, 0.300], [-90, -112, 0]),
        # (GRIP, 1),
        # (MOVE, [-0.100, 0.520, 0.300], [-90, -112, 0]),
        # (MOVE, [-0.300, 0, 0.350], [-179.5, 0, -179.5]),
        # (MOVE, [-0.300, 0, 0.200], [-179.5, 0, -179.5]),
        # (GRIP, 0),
        # (MOVE, [-0.300, 0, 0.350], [-179.5, 0, -179.5]),
        # (MOVE, None, None)
    ]

    joint_actions = dict()

    def compute_thread():
        last_idx = -1
        for i, action in enumerate(actions):
            if action[0] == MOVE:
                node.get_logger().info(f"Requesting joints for {i}")
                joint_actions[i] = cm.move(action[1], action[2], None if last_idx == -1 else joint_actions[last_idx])
                node.get_logger().info(f"Received joints for {i}")
                last_idx = i

    thread = threading.Thread(target=compute_thread)
    thread.start()

    if not os.path.exists("data"):
        os.makedirs("data")
    f_idx = 0
    while os.path.exists(f"data/hybrid_state_{f_idx}.csv"):
        f_idx += 1
    log_file = open(f"data/hybrid_state_{f_idx}.csv", "w")
    log_file.write("Time,State,X,Y,Z,RX,RY,RZ,F\n")

    for i, action in enumerate(actions):
        if action[0] == MOVE:
            _, p, r = action
            node.get_logger().info(f"Moving to {i}: {p} with orientation {r}")
            if p is None:
                log_file.write(f"{time.time()},{MOVE},{','.join(['0']*6)},0\n")
            else:
                log_file.write(f"{time.time()},{MOVE},{','.join([str(x) for x in p])},{','.join([str(x) for x in r])},0\n")
            while i not in joint_actions:
                time.sleep(0.1)
            if joint_actions[i]:
                cm.dance(joint_actions[i])
        elif action[0] == GRIP:
            _, f = action
            node.get_logger().info(f"Setting desired force to {f}")
            log_file.write(f"{time.time()},{GRIP},{','.join(['0']*6)},{f}\n")
            driver.set_fd(f)
            count = 0
            while not driver.get_done():
                if count % 10 == 0:
                    node.get_logger().info("Waiting for gripper to reach desired force")
                count += 1
                time.sleep(0.5)

    log_file.close()
    node.get_logger().info(f"Data saved to data/hybrid_state_{f_idx}.csv")
    driver.stop()
    node.get_logger().info("It worked!")
