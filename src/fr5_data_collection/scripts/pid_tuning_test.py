#!/usr/bin/env python
import rospy
import moveit_commander
import sys
import time

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pid_tuning_test', anonymous=True)
    
    group = moveit_commander.MoveGroupCommander("fr5_arm")
    group.set_max_velocity_scaling_factor(0.5)
    group.set_max_acceleration_scaling_factor(0.5)
    
    rospy.loginfo("PID Tuning Test Script Started")
    rospy.loginfo("Please open 'rqt_reconfigure' in another terminal to tune gains.")
    
    # 1. Move to Zero Pose (Usually vertical or folded)
    rospy.loginfo("1. Moving to 'zero' pose...")
    group.set_named_target("zero")
    group.go(wait=True)
    rospy.sleep(2.0)
    
    # 2. Move to a pose that exerts gravity torque (Horizontal stretch)
    # Adjust these values based on your robot's kinematics
    # Assuming J2 controls the shoulder lift
    stretch_joints = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0] 
    
    rospy.loginfo(f"2. Moving to Stretch Pose: {stretch_joints}")
    group.go(stretch_joints, wait=True)
    
    rospy.loginfo("Holding pose for tuning... (Press Ctrl+C to stop)")
    
    # Keep sending the command to hold position
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # In a real controller, just staying idle holds the position,
        # but here we just keep the script alive.
        # You can also gently wiggle joints here if you want to test dynamic response.
        rate.sleep()

if __name__ == "__main__":
    main()
