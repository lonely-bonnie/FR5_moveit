#!/usr/bin/env python
import rospy
import moveit_commander
import sys
import csv
import time
import os
from sensor_msgs.msg import JointState
from math import pi

class DataCollector:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('fr5_data_collector', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "fr5_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.data_buffer = []
        self.is_recording = False
        self.last_vel = None
        self.last_time = None

        # Subscribe to joint states
        self.sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        
        # Output file setup
        self.output_dir = os.path.join(os.path.dirname(__file__), "../data")
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        self.csv_file = os.path.join(self.output_dir, "fr5_pinn_data.csv")
        
        rospy.loginfo(f"Data will be saved to {self.csv_file}")

    def joint_state_callback(self, msg):
        if not self.is_recording:
            return

        # Sort by name to ensure consistency across different messages
        sorted_indices = sorted(range(len(msg.name)), key=lambda k: msg.name[k])
        
        names = [msg.name[i] for i in sorted_indices]
        positions = [msg.position[i] for i in sorted_indices]
        velocities = [msg.velocity[i] if msg.velocity else 0.0 for i in sorted_indices]
        efforts = [msg.effort[i] if msg.effort else 0.0 for i in sorted_indices]
        
        current_time = msg.header.stamp.to_sec()
        
        # Calculate acceleration using finite difference
        accelerations = [0.0] * len(names)
        if self.last_vel is not None and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0.0001: # Avoid division by zero or extremely small dt
                for i in range(len(names)):
                    accelerations[i] = (velocities[i] - self.last_vel[i]) / dt
        
        self.last_vel = velocities
        self.last_time = current_time

        record = {
            'timestamp': current_time,
            'names': names,
            'positions': positions,
            'velocities': velocities,
            'accelerations': accelerations,
            'efforts': efforts
        }
        self.data_buffer.append(record)

    def start_recording(self):
        self.data_buffer = []
        self.is_recording = True
        self.last_vel = None
        self.last_time = None
        rospy.loginfo("Started recording data...")

    def stop_recording(self):
        self.is_recording = False
        rospy.loginfo(f"Stopped recording. Captured {len(self.data_buffer)} samples.")
        self.save_data()

    def save_data(self):
        if not self.data_buffer:
            return

        # Get joint names from first record to create header
        joint_names = self.data_buffer[0]['names']
        
        header = ['timestamp']
        for name in joint_names:
            header.extend([f'{name}_pos', f'{name}_vel', f'{name}_acc', f'{name}_eff'])
            
        file_exists = os.path.isfile(self.csv_file)
        
        with open(self.csv_file, 'a') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(header)
                
            for record in self.data_buffer:
                row = [record['timestamp']]
                for i in range(len(joint_names)):
                    row.extend([
                        record['positions'][i],
                        record['velocities'][i],
                        record['accelerations'][i],
                        record['efforts'][i]
                    ])
                writer.writerow(row)
        
        rospy.loginfo(f"Data appended to {self.csv_file}")

    def move_to_pose(self, pose_name):
        rospy.loginfo(f"Moving to named target: {pose_name}")
        self.move_group.set_named_target(pose_name)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def move_joint(self, joint_values):
        rospy.loginfo(f"Moving to joint values: {joint_values}")
        self.move_group.go(joint_values, wait=True)
        self.move_group.stop()

    def run_trajectories(self):
        # ---------------------------------------------------------
        # Trajectories from main.pdf (extracted from SRDF)
        # ---------------------------------------------------------
        
        trajectory_groups = {
            "traj0": ["traj0_pose1", "traj0_pose2"],
            "traj1": ["traj1_mid_pose", "traj1_inside", "traj1_up", "traj1_change_pose", "traj1_send_inside", "traj1_down", "traj1_back"],
            "traj2": ["traj2_pose1", "traj2_inside", "traj2_up", "traj2_outside", "traj2_mid_pose", "traj2_inside2", "traj2_down", "traj2_outside2"],
            "traj3": ["traj3_mid_pose", "traj3_inside", "traj3_down", "traj3_outside"],
            "traj4": ["traj4_mid_pose", "traj4_inside", "traj4_down", "traj4_outside1", "traj4_outside2"]
        }

        # Move to initial position first
        self.move_to_pose("zero")
        time.sleep(1.0)

        for traj_name, poses in trajectory_groups.items():
            rospy.loginfo(f"--- Executing Trajectory Group: {traj_name} ---")
            
            # Move to the first pose of the trajectory without recording to get in position
            self.move_to_pose(poses[0])
            time.sleep(0.5)
            
            # Now record the sequence
            for pose in poses:
                self.start_recording()
                success = self.move_to_pose(pose)
                if success:
                    # Optional: Record a bit of static data at the pose or just the movement
                    time.sleep(0.5) 
                self.stop_recording()
                time.sleep(0.5)
                
        rospy.loginfo("All PDF trajectories completed.")

        # Example 2: Moving to random poses (good for PINN data diversity)
        rospy.loginfo("Executing random movements for data diversity...")
        for _ in range(5):
            self.start_recording()
            self.move_group.set_random_target()
            self.move_group.go(wait=True)
            self.stop_recording()
            time.sleep(0.5)

if __name__ == '__main__':
    try:
        collector = DataCollector()
        # Wait for subscribers and publishers to connect
        time.sleep(1.0)
        collector.run_trajectories()
    except rospy.ROSInterruptException:
        pass
