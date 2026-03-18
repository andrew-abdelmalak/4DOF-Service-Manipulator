#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from trajectory_planner import get_task_space_trajectory, get_joint_space_trajectory, get_circular_trajectory
from kinematics_ms03 import inverse_kinematics_func

def normalize_angle(angle):
    """Wraps an angle to [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))

def publish_joints(publishers, q_values):
    """Publishes angles to Gazebo after normalizing them."""
    for i, pub in enumerate(publishers):
        norm_q = normalize_angle(q_values[i])
        pub.publish(Float64(norm_q))

def main():
    rospy.init_node('milestone4_trajectory_node')

    # Publishers [Joint1, Joint2, Joint3, Joint5]
    pub1 = rospy.Publisher('/Joint_1/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/Joint_2/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/Joint_3/command', Float64, queue_size=10)
    pub5 = rospy.Publisher('/Joint_5/command', Float64, queue_size=10)
    
    pubs = [pub1, pub2, pub3, pub5]

    # --- TRAJECTORY CONFIGURATION ---
    # 1. Circular Config (for reference)
    circle_center = [0.15, 0.0, 0.15]
    circle_radius = 0.03
    
    # 2. Square/Box Config (REFINED SAFE COORDINATES)
    # Previous X=0.25 was too far. 
    # New range: X=0.18 (front) to X=0.12 (back)
    pA = [0.15, 0.10, 0.15]
    pB = [0.15, 0.10, 0.20]
    pC = [0.15, 0.0, 0.20]
    pD = [0.15, 0.0, 0.15]
    
    segment_duration = 1.0 
    dt = 0.1       

    # --- CHOOSE MODE HERE ---
    MODE = 'circle' 

    rospy.loginfo(f"--- Starting Milestone 4: {MODE.upper()} Trajectory ---")
    rospy.sleep(1.0)

    # 1. Move to Home Position
    q_home = [0, 0, 0, 0]
    publish_joints(pubs, q_home)
    rospy.sleep(2.0)

    if MODE == 'square':
        waypoints = [pA, pB, pC, pD]
        
        # -- Step 1: Move to Start Point A --
        rospy.loginfo(f"Moving to Start Point A: {pA}...")
        # Solve IK for A using home as guess
        q_start = inverse_kinematics_func(q_home, pA)
        q_start = [normalize_angle(q) for q in q_start]
        
        # Publish and wait
        publish_joints(pubs, q_start)
        rospy.sleep(4.0) 
        
        q_prev = q_start
        rate = rospy.Rate(1/dt)

        # -- Step 2: Loop through segments (A->B, B->C, C->D) --
        for i in range(len(waypoints) - 1):
            start_pt = waypoints[i]
            end_pt = waypoints[i+1]
            
            rospy.loginfo(f"--- Starting Segment {i+1}: {start_pt} -> {end_pt} ---")
            
            # Generate Linear Trajectory for this segment
            times, points = get_task_space_trajectory(start_pt, end_pt, segment_duration, dt)
            
            for t, point in zip(times, points):
                if rospy.is_shutdown(): break
                
                # Solve IK
                q_sol = inverse_kinematics_func(q_prev, point)
                
                # Safety Check
                if np.any(np.abs(q_sol) > 100.0):
                    rospy.logerr("Unsafe joint values detected! Stopping.")
                    break

                # Publish
                publish_joints(pubs, q_sol)
                
                rospy.loginfo(f"Seg {i+1} | T: {t:.1f} | Tgt: {np.round(point,2)} | q: {np.round(q_sol, 2)}")
                
                q_prev = q_sol 
                rate.sleep()
            
            # Short pause at each corner to make the motion distinct
            rospy.sleep(0.5)

    elif MODE == 'circle':
        # Calculate Circle Start
        circle_start_pos = [circle_center[0] + circle_radius, circle_center[1], circle_center[2]]

        rospy.loginfo(f"Moving to Circle Start: {circle_start_pos}...")
        q_circle_start = inverse_kinematics_func(q_home, circle_start_pos)
        publish_joints(pubs, q_circle_start)
        rospy.sleep(4.0) 

        rospy.loginfo("Starting Circular Motion...")
        times, points = get_circular_trajectory(circle_center, circle_radius, 10.0, dt)
        
        q_prev = q_circle_start 
        rate = rospy.Rate(1/dt)

        for t, point in zip(times, points):
            if rospy.is_shutdown(): break
            q_sol = inverse_kinematics_func(q_prev, point)
            publish_joints(pubs, q_sol)
            rospy.loginfo(f"T: {t:.1f} | Tgt: {np.round(point,3)} | q: {np.round(q_sol, 2)}")
            q_prev = q_sol 
            rate.sleep()

    rospy.loginfo("Trajectory Completed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
