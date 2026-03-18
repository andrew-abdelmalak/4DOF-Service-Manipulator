#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped
import numpy as np

from kinematics_ms03 import (
    jacobian_matrix,
    inverse_kinematics_func,      # uses your MS02 FK internally
    forward_velocity_kinematics,  # not used yet, but ready
    inverse_velocity_kinematics
)

ee_pos = None  # latest end-effector position from Gazebo


def ee_cb(msg):
    """Callback for /end_effector_pose."""
    global ee_pos
    ee_pos = np.array(
        [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
        dtype=float
    )


def publish_joint_commands(pubs, q):
    """
    Publish joint angles to Gazebo.
    q = [q1, q2, q3, q4_math]  where q4_math corresponds to Joint_5.
    """
    pubs[0].publish(Float64(float(q[0])))
    pubs[1].publish(Float64(float(q[1])))
    pubs[2].publish(Float64(float(q[2])))
    pubs[3].publish(Float64(float(q[3])))


def make_layout(rows, cols):
    """Create MultiArray layout for a (rows x cols) matrix."""
    arr = Float64MultiArray()
    layout = arr.layout
    layout.dim = [
        MultiArrayDimension(label="rows", size=rows, stride=rows * cols),
        MultiArrayDimension(label="cols", size=cols, stride=cols),
    ]
    layout.data_offset = 0
    return layout


def main():
    rospy.init_node("milestone3_ms3_jacobian")

    # Publishers for your 4 active joints (q1, q2, q3, q4_math->Joint_5)
    pubs = [
        rospy.Publisher('/Joint_1/command', Float64, queue_size=10),
        rospy.Publisher('/Joint_2/command', Float64, queue_size=10),
        rospy.Publisher('/Joint_3/command', Float64, queue_size=10),
        rospy.Publisher('/Joint_5/command', Float64, queue_size=10),
    ]

    # Jacobian publisher: 6x4 matrix, row-major
    jac_pub = rospy.Publisher('/jacobian', Float64MultiArray, queue_size=10)

    # Subscribe to end-effector pose
    rospy.Subscriber('/end_effector_pose', PoseStamped, ee_cb, queue_size=1)
    rospy.loginfo("Waiting for /end_effector_pose...")
    rospy.wait_for_message('/end_effector_pose', PoseStamped, timeout=10.0)
    rospy.loginfo("/end_effector_pose is active.")

    # -------- Inverse Position Kinematics validation --------
    X_des = np.array([0.0000, 0.0000, 0.4255], dtype=float)   # desired EE position
    q0    = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)       # initial guess

    rospy.loginfo("X_des: %s", X_des.tolist())
    rospy.loginfo("q0: %s", q0.tolist())

    q_des = inverse_kinematics_func(q0, X_des)
    rospy.loginfo("q_des: %s", np.array(q_des).round(4).tolist())

    # Send joints to Gazebo
    publish_joint_commands(pubs, q_des)
    rospy.sleep(2.5)  # let Gazebo update

    # Compare with actual Gazebo EE pose if available
    if ee_pos is not None:
        X_meas = ee_pos.copy()
        err = X_des - X_meas
        rospy.loginfo("Gazebo EE pose: X=%.4f, Y=%.4f, Z=%.4f",
                      X_meas[0], X_meas[1], X_meas[2])
        rospy.loginfo("||X_des - X_meas|| = %.6f m", np.linalg.norm(err))
    else:
        rospy.logwarn("No /end_effector_pose received after sending joints.")

    # -------- Continuous Jacobian publishing at q_des --------
    rospy.loginfo("Starting continuous /jacobian publishing at 10 Hz...")
    layout = make_layout(6, 4)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        J = jacobian_matrix(q_des)  # 6x4
        msg = Float64MultiArray()
        msg.layout = layout
        msg.data = J.flatten().tolist()  # row-major
        jac_pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as e:
        rospy.logerr(e)

