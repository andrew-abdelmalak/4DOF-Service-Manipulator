#!/usr/bin/env python3

import rospy
import numpy as np
import math
# Import your MS02 functions AND your link lengths
from forward_kinematics import transformation_func, forward_kinematics_func, l1, l2, l3, l4

# Map link lengths to DH-style constants used in the Jacobian derivation
d1 = l1                      # base offset
a2, a3, a4 = l2, l3, l4      # link lengths of joints 2, 3, and wrist

def get_full_kinematics(q):
    """
    Helper function based on YOUR forward_kinematics.py file.
    q = [q1, q2, q3, q4]  (here q4 == Joint_5 in Gazebo)
    """
    q1, q2, q3, q4 = q[0], q[1], q[2], q[3]

    # We only really need the EE position for IK, so call your MS2 FK:
    x, y, z = forward_kinematics_func(q1, q2, q3, q4)
    return np.array([x, y, z])

# -----------------------------
#   Geometric Jacobian (6x4)
#   (q4 here == Joint_5 in Gazebo)
# -----------------------------

def jacobian_geometric(q1, q2, q3, q4):
    """
    Returns the 6x4 geometric Jacobian for the 4-DOF robot.
    Inputs: q1, q2, q3, q4 (joint angles, where q4 is your Joint_5).

    This is exactly your closed-form Jacobian.
    """

    # Precompute
    Δ = q2 - q3

    c1, s1 = np.cos(q1), np.sin(q1)
    c2, s2 = np.cos(q2), np.sin(q2)
    cΔ, sΔ = np.cos(Δ), np.sin(Δ)
    cD4, sD4 = np.cos(Δ + q4), np.sin(Δ + q4)

    # Useful repeated expressions
    R = a2 * c2 + a3 * cΔ + a4 * cD4           # radial horizontal projection
    Z = a2 * s2 + a3 * sΔ + a4 * sD4           # vertical extension

    # --- Jv (3x4) ---
    Jv = np.zeros((3, 4))

    # Column 1 (joint 1)
    Jv[:, 0] = np.array([
        -R * s1,
         R * c1,
         0.0
    ])

    # Column 2 (joint 2)
    Jv[:, 1] = np.array([
         Z * c1,
         Z * s1,
         R
    ])

    # Column 3 (joint 3)
    Jv[:, 2] = np.array([
        -(a3 * sΔ + a4 * sD4) * c1,
        -(a3 * sΔ + a4 * sD4) * s1,
        -(a3 * cΔ + a4 * cD4)
    ])

    # Column 4 (joint 4 = Joint_5)
    Jv[:, 3] = np.array([
         a4 * sD4 * c1,
         a4 * sD4 * s1,
         a4 * cD4
    ])

    # --- Jw (3x4) ---
    Jw = np.array([
        [0.0,  c1,   -c1,    c1],
        [0.0,  s1,   -s1,    s1],
        [1.0,  0.0,   0.0,   0.0]
    ])

    # Full 6x4 Jacobian
    J = np.vstack((Jv, Jw))
    return J

def jacobian_matrix(q):
    """
    Wrapper used by other code:
    q = [q1, q2, q3, q4]  ->  J (6x4)

    NOTE: in Gazebo, this q4 corresponds to /Joint_5/command.
    """
    q = np.asarray(q, dtype=float).ravel()
    assert q.size == 4, "q must be [q1, q2, q3, q4]"
    return jacobian_geometric(q[0], q[1], q[2], q[3])

def inverse_jacobian_matrix(q):
    """
    Calculates the pseudo-inverse of the Jacobian (4x6).
    """
    J = jacobian_matrix(q)
    # Use pseudo-inverse since J is not square (6x4)
    J_inv = np.linalg.pinv(J)  # [4x6]
    return J_inv

# -----------------------------------
#   Inverse Position Kinematics (IK)
# -----------------------------------

def inverse_kinematics_func(q0, X_des, max_iter=100, tolerance=1e-4):
    """
    Solves Inverse Position Kinematics using Newton-Raphson.

    Uses your MS2 forward_kinematics_func and the Jacobian above.
    q0: initial guess [q1, q2, q3, q4]
    X_des: desired EE position [x, y, z]
    """
    q_current = np.array(q0, dtype=float).ravel()

    for i in range(max_iter):
        # Forward kinematics using your MS2 function
        x, y, z = forward_kinematics_func(
            q_current[0], q_current[1], q_current[2], q_current[3]
        )
        X_current = np.array([x, y, z])

        # Position error
        e = X_des - X_current
        err_norm = np.linalg.norm(e)

        if err_norm < tolerance:
            rospy.loginfo("IK converged in %d iterations, error = %.6e", i, err_norm)
            return q_current

        # Build a 6x1 task error: position error + zero desired angular error
        V_des = np.hstack((e, np.zeros(3)))  # [vx, vy, vz, wx, wy, wz]

        # Jacobian pseudo-inverse
        J_inv = inverse_jacobian_matrix(q_current)  # [4x6]

        # Joint update (Newton-Raphson in velocity form)
        q_delta = J_inv.dot(V_des)  # [4x1]

        # Update joint angles
        q_current += q_delta

    rospy.logwarn(
        "IK did not converge after %d iterations. Final error = %.6e",
        max_iter, err_norm
    )
    return q_current

# -----------------------------------
#   Inverse Velocity Kinematics
# -----------------------------------

def inverse_velocity_kinematics(q, V_des):
    """
    Calculates joint velocities from desired EE velocity.
    q: current joint angles [4x1]
    V_des: desired EE velocity [6x1] (vx, vy, vz, wx, wy, wz)
    """
    J_inv = inverse_jacobian_matrix(q)  # [4x6]
    q_dot = J_inv.dot(V_des)  # [4x1]
    return q_dot


def forward_velocity_kinematics(q, q_dot):
    """
    Calculates end-effector spatial velocity from joint velocities.

    q:     current joint angles [4x1] (q1, q2, q3, q4 where q4 == Joint_5)
    q_dot: joint velocities [4x1]

    Returns:
        V = [vx, vy, vz, wx, wy, wz]^T  (6x1)
    """
    q = np.asarray(q, dtype=float).ravel()
    q_dot = np.asarray(q_dot, dtype=float).ravel()
    J = jacobian_matrix(q)          # [6x4]
    V = J.dot(q_dot)                # [6x1]
    return V

