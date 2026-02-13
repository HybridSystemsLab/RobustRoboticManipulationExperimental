#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# import rospy
import os.path
import matplotlib.pyplot as plt
import numpy as np
import sympy as sp
from flask import Flask, jsonify, request
from scipy.spatial.transform import Rotation


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


JOINT_MIN_LIMITS = [-3.14, -2.04, -2.61, -3.14, -2.56, -3.14]
JOINT_MAX_LIMITS = [3.14, 2.04, 2.61, 3.14, 2.56, 3.14]

END_EFFECTOR_ATTACHED = True
ITER_COUNT = 100

a = [0, -0.266, 0, 0, 0, 0]  # link lengths in meters
alpha = [90, 180, 90, -90, 90, 0]  # link twists in degrees
d = [
    0.1925,
    0,
    0,
    0.324,
    0,
    0.155 + (0.219 / 2 if END_EFFECTOR_ATTACHED else 0),
]  # link offsets in meters (half the last link length)

# Joint angles (theta_i) - all are variables
theta1, theta2, theta3, theta4, theta5, theta6 = sp.symbols(
    "theta1 theta2 theta3 theta4 theta5 theta6"
)
theta = [theta1, theta2 - sp.pi / 2, theta3 - sp.pi / 2, theta4, theta5, theta6]

A = []
for i in range(6):
    A.append(
        sp.Matrix(
            [
                [
                    sp.cos(theta[i]),
                    -sp.sin(theta[i]) * np.cos(np.deg2rad(alpha[i])),
                    sp.sin(theta[i]) * np.sin(np.deg2rad(alpha[i])),
                    a[i] * sp.cos(theta[i]),
                ],
                [
                    sp.sin(theta[i]),
                    sp.cos(theta[i]) * np.cos(np.deg2rad(alpha[i])),
                    -sp.cos(theta[i]) * np.sin(np.deg2rad(alpha[i])),
                    a[i] * sp.sin(theta[i]),
                ],
                [
                    0,
                    np.sin(np.deg2rad(alpha[i])),
                    np.cos(np.deg2rad(alpha[i])),
                    d[i],
                ],
                [0, 0, 0, 1],
            ]
        )
    )

T = []
T06 = A[0]
T.append(T06)
for i in range(1, 6):
    T06 = T06 * A[i]
    T.append(T06)


R = T06[:3, :3]
p = T06[:3, 3]

Jv = sp.zeros(3, 6)
Jw = sp.zeros(3, 6)

T_prev = sp.eye(4)
z_prev = sp.Matrix([0, 0, 1])
p_prev = sp.Matrix([0, 0, 0])
for i in range(6):
    T_prev = T_prev * A[i]
    z_curr = T_prev[:3, 2]
    p_curr = T_prev[:3, 3]
    Jv[:, i] = z_prev.cross(p - p_prev)
    Jw[:, i] = z_prev
    z_prev = z_curr
    p_prev = p_curr

# Combine Jacobian components
J = sp.Matrix.vstack(Jv, Jw)

Kp_pos = 0.4
Kd_pos = 0.1

Kp_rot = np.deg2rad(10.0)
Kd_rot = np.deg2rad(0.05)

pos_tolerance = 1e-6
rot_tolerance = 1e-3

app = Flask(__name__)


def check_pose(drag_joints, target_pose, target_orientation, retry=False):
    print(f"Checking pose: {drag_joints}")
    drag_joints = np.mod(drag_joints, 2 * np.pi)
    drag_joints = np.array(
        [(j - 2 * np.pi) if j > np.pi else j for j in drag_joints], dtype=float
    )
    drag_joints = np.round(drag_joints, 2)
    print(
        f"Pose re-mapped to [-pi, pi] range: {'' if retry else bcolors.OKGREEN} {drag_joints} {'' if retry else bcolors.ENDC}"
    )
    for i in range(6):
        transform = np.array(
            T[i].subs(
                [
                    (theta1, drag_joints[0]),
                    (theta2, drag_joints[1]),
                    (theta3, drag_joints[2]),
                    (theta4, drag_joints[3]),
                    (theta5, drag_joints[4]),
                    (theta6, drag_joints[5]),
                ]
            ),
            dtype=float,
        )
        final_p = np.round(transform[:3, 3], 4)
        print(f"Position {i}: {final_p}")
        if final_p[0] > 0.48 or final_p[2] < 0.1:
            print(f"Position {i} is invalid")
            if i >= 3:
                print("Target position/orientation is invalid")
                return False, drag_joints
            elif i >= 1 and not retry:
                print("Trying to fix via the other triangle solution")
                drag_joints[1] -= drag_joints[2]
                drag_joints[4] += drag_joints[2]
                drag_joints[2] *= -1

                drag_joints = compute(
                    target_pose, target_orientation, drag_joints, retry=True
                )
                return check_pose(
                    drag_joints, target_pose, target_orientation, retry=True
                )
            else:
                print("Unknown error")
                return False, drag_joints
        elif drag_joints[i] != np.clip(
            drag_joints[i], JOINT_MIN_LIMITS[i], JOINT_MAX_LIMITS[i]
        ):
            print(f"Joint {i} is invalid")
            if not retry:
                print("Trying to fix via the other triangle solution")
                drag_joints[1] -= drag_joints[2]
                drag_joints[4] += drag_joints[2]
                drag_joints[2] *= -1

                drag_joints = compute(
                    target_pose, target_orientation, drag_joints, retry=True
                )
                return check_pose(
                    drag_joints, target_pose, target_orientation, retry=True
                )
            else:
                print("Unknown error")
                return False, drag_joints

    return True, drag_joints

    # position 1/2 == joint 3
    # position 3/4 == joint 5
    # position 5 == target position


def compute(target_pose, target_orientation, starting_joint_state, retry=False):
    theta_vals = np.array(starting_joint_state, dtype=float)
    target_pose = np.array(target_pose, dtype=float)
    target_orientation = np.array(target_orientation, dtype=float)
    target_orientation_matrix = Rotation.from_euler(
        "xyz", target_orientation, degrees=True
    ).as_matrix()

    position_errors = []
    orientation_errors = []

    for iter in range(ITER_COUNT):
        if iter % 10 == 0 or iter == ITER_COUNT - 1:
            print("Iteration: {}".format(iter))

        # Current end-effector position and orientation
        current_position = np.array(
            p.subs(
                [
                    (theta1, theta_vals[0]),
                    (theta2, theta_vals[1]),
                    (theta3, theta_vals[2]),
                    (theta4, theta_vals[3]),
                    (theta5, theta_vals[4]),
                    (theta6, theta_vals[5]),
                ]
            ),
            dtype=float,
        ).flatten()
        current_orientation = np.array(
            R.subs(
                [
                    (theta1, theta_vals[0]),
                    (theta2, theta_vals[1]),
                    (theta3, theta_vals[2]),
                    (theta4, theta_vals[3]),
                    (theta5, theta_vals[4]),
                    (theta6, theta_vals[5]),
                ]
            ),
            dtype=float,
        )
        # Position error
        position_error = target_pose - current_position
        R_error = target_orientation_matrix @ current_orientation.T

        theta_error = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))

        # Store the position error
        position_errors.append(np.linalg.norm(position_error))
        orientation_errors.append(theta_error)

        if abs(theta_error) > rot_tolerance:
            u_error = (1 / (2 * np.sin(theta_error))) * np.array(
                [
                    R_error[2][1] - R_error[1][2],
                    R_error[0][2] - R_error[2][0],
                    R_error[1][0] - R_error[0][1],
                ]
            )

            if np.linalg.norm(u_error) > rot_tolerance:
                u_error = u_error / np.linalg.norm(u_error)
            orientation_error = theta_error * u_error
        else:
            orientation_error = np.zeros(3)

        control_outputs = np.zeros(6)
        for i in range(3):
            control_outputs[i] = Kp_pos * position_error[i]
        for i in range(3):
            control_outputs[i + 3] = Kp_rot * orientation_error[i]

        J_val = np.array(
            J.subs(
                [
                    (theta1, theta_vals[0]),
                    (theta2, theta_vals[1]),
                    (theta3, theta_vals[2]),
                    (theta4, theta_vals[3]),
                    (theta5, theta_vals[4]),
                    (theta6, theta_vals[5]),
                ]
            ),
            dtype=float,
        )

        # Compute change in joint variables using pseudo-inverse of Jacobian
        delta_theta = (
            np.linalg.pinv(J_val, rcond=1e-4) @ control_outputs
        )  # TODO: this might fail bc different dimensions of J_val and control_outputs --> control_outputs used to be 3x1 instead of 6x1

        # Update joint variables
        theta_vals += delta_theta

        # print(
        #     f"Errors: {np.linalg.norm(position_error)} {np.linalg.norm(orientation_error)}"
        # )

        # Check convergence to the final desired position
        if (
            np.linalg.norm(position_error) < pos_tolerance
            and np.linalg.norm(orientation_error) < rot_tolerance
        ):
            break

    if not os.path.exists("../../../../data"):
        os.makedirs("../../../../data")
    f_idx = 0
    while os.path.exists(f"../../../../data/hybrid_arm_{f_idx}.csv"):
        f_idx += 1
    if retry:
        f = open(f"../../../../data/hybrid_arm_{f_idx-1}_retry.csv", "w")
    else:
        f = open(f"../../../../data/hybrid_arm_{f_idx}.csv", "w")
    f.write("PositionError,OrientationError\n")
    for i in range(len(position_errors)):
        f.write(f"{position_errors[i]},{orientation_errors[i]}\n")
    f.close()

    if retry:
        return theta_vals.tolist()

    valid, theta_vals = check_pose(theta_vals, target_pose, target_orientation)
    print(f"{'Valid' if valid else 'Invalid'} pose: {theta_vals}")
    if request:
        return jsonify({"joints": theta_vals.tolist()}), 200 if valid else 400
    else:
        return theta_vals.tolist()


@app.route("/move", methods=["GET"])
def move():
    if not request.json or any(
        [
            key not in request.json
            for key in ["position", "orientation", "starting_joint_state"]
        ]
    ):
        return jsonify({"error": "No JSON data provided"}), 400

    target_pose = np.array(request.json["position"])
    target_orientation = np.array(request.json["orientation"])
    starting_joint_state = np.array(request.json["starting_joint_state"])

    print(
        f"Received request: {target_pose}, {target_orientation}, {starting_joint_state}"
    )

    return compute(target_pose, target_orientation, starting_joint_state)


def plot(joints, target_position, target_orientation):
    vals = [
        np.array(
            T[i].subs(
                [
                    (theta1, joints[0]),
                    (theta2, joints[1]),
                    (theta3, joints[2]),
                    (theta4, joints[3]),
                    (theta5, joints[4]),
                    (theta6, joints[5]),
                ]
            ),
            dtype=float,
        )
        for i in range(6)
    ]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Plot links with different colors
    ax.plot(
        [0, vals[0][0, 3]], [0, vals[0][1, 3]], [0, vals[0][2, 3]], "b-", linewidth=2
    )
    ax.plot(
        [vals[0][0, 3], vals[1][0, 3]],
        [vals[0][1, 3], vals[1][1, 3]],
        [vals[0][2, 3], vals[1][2, 3]],
        "r-",
        linewidth=2,
    )
    ax.plot(
        [vals[1][0, 3], vals[2][0, 3]],
        [vals[1][1, 3], vals[2][1, 3]],
        [vals[1][2, 3], vals[2][2, 3]],
        "g-",
        linewidth=2,
    )
    ax.plot(
        [vals[2][0, 3], vals[3][0, 3]],
        [vals[2][1, 3], vals[3][1, 3]],
        [vals[2][2, 3], vals[3][2, 3]],
        "c-",
        linewidth=2,
    )
    ax.plot(
        [vals[3][0, 3], vals[4][0, 3]],
        [vals[3][1, 3], vals[4][1, 3]],
        [vals[3][2, 3], vals[4][2, 3]],
        "m-",
        linewidth=2,
    )
    ax.plot(
        [vals[4][0, 3], vals[5][0, 3]],
        [vals[4][1, 3], vals[5][1, 3]],
        [vals[4][2, 3], vals[5][2, 3]],
        "y-",
        linewidth=2,
    )

    ax.plot(
        target_position[0],
        target_position[1],
        target_position[2],
        "rx",
        markersize=12,
        linewidth=2,
    )

    target_orientation = Rotation.from_euler(
        "xyz", target_orientation, degrees=True
    ).as_matrix()
    v = np.dot(target_orientation, np.array([[0, 0, 1]]).T)
    ax.quiver(
        target_position[0],
        target_position[1],
        target_position[2],
        v[0],
        v[1],
        v[2],
        color="r",
        length=0.3,
    )

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")  # type: ignore
    ax.set_xlim(-0.75, 0.75)
    ax.set_ylim(-0.75, 0.75)
    ax.set_zlim(0, 1.5)  # type: ignore
    plt.show()


if __name__ == "__main__":
    # target_pose = np.array([0, 0.25, 0.3])
    # target_orientation = np.array([-90, -22, 0])
    # starting_joint_state = np.array([-1, 2, 1, 0, 0, 1])

    # result = compute(target_pose, target_orientation, starting_joint_state)
    # print(result)
    # plot(result, target_pose, target_orientation)

    app.run(host="0.0.0.0", port=5001)