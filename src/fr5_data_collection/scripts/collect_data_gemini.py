#!/usr/bin/env python
import rospy
import moveit_commander
import sys
import csv
import time
import os
import numpy as np
import random
from math import pi, sin, cos
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory

class TrajectoryGenerator:
    """
    负责生成 main.pdf 中描述的数学轨迹
    """
    def __init__(self, num_joints=6):
        self.num_joints = num_joints
        
        #  论文推荐的互质频率集合 (Hz)
        self.frequencies = [
            [0.17, 0.53, 0.89], # J1
            [0.13, 0.47, 0.79], # J2
            [0.19, 0.59, 0.97], # J3
            [0.23, 0.43, 0.83], # J4
            [0.11, 0.41, 0.73], # J5
            [0.29, 0.61, 1.07]  # J6
        ]
        
        # 随机相位 phi ~ U(0, 2pi) [cite: 18]
        self.phases = np.random.uniform(0, 2*pi, (6, 3))
        
        # 幅值设置 (根据实际仿真安全范围需微调，此处设为保守值)
        self.amplitudes = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15]

    def randomize_phases(self):
        """重新生成随机相位，用于多条不同的轨迹"""
        self.phases = np.random.uniform(0, 2*pi, (6, 3))

    def get_multi_sine_point(self, t, joint_idx, q0):
        """
        计算单关节在 t 时刻的多正弦位置, 速度, 加速度
        公式参考 main.pdf Eq (7), (8), (9) [cite: 13, 14, 15]
        """
        pos = q0
        vel = 0.0
        acc = 0.0
        
        # 累加 K=3 个正弦分量
        for k in range(3):
            f = self.frequencies[joint_idx][k]
            phi = self.phases[joint_idx][k]
            A = self.amplitudes[joint_idx] / 3.0 # 分配总幅值
            
            omega = 2 * pi * f
            angle = omega * t + phi
            
            pos += A * sin(angle)
            vel += A * omega * cos(angle)
            acc -= A * (omega**2) * sin(angle)
            
        return pos, vel, acc

    def get_chirp_point(self, t, duration, q0):
        """
        生成 Chirp 信号
        公式参考 main.pdf Eq (6) 
        f0 = 0.1, f1 = 2.0
        """
        f0 = 0.1
        f1 = 2.0
        alpha = (f1 - f0) / duration
        A = 0.05 # Chirp 幅值通常较小 [cite: 38]
        
        angle = 2 * pi * (f0 * t + 0.5 * alpha * t**2)
        
        pos = q0 + A * sin(angle)
        vel = A * cos(angle) * (2 * pi * (f0 + alpha * t))
        # acc 近似计算忽略高阶小量，或完整推导，此处简化用于轨迹生成
        acc = -A * sin(angle) * (2 * pi * (f0 + alpha * t))**2 
        
        return pos, vel, acc

class DataCollector:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('fr5_data_collector', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.group_name = "fr5_arm" # 请确认这与你的SRDF中的组名一致
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        # 增加轨迹规划容差，防止密集点执行失败
        self.move_group.set_goal_joint_tolerance(0.001)
        self.move_group.set_goal_position_tolerance(0.001)

        self.data_buffer = []
        self.is_recording = False
        self.last_vel = None
        self.last_time = None

        self.sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        
        # 数据保存路径
        self.output_dir = os.path.join(os.path.dirname(__file__), "../data")
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        self.generator = TrajectoryGenerator()

    def joint_state_callback(self, msg):
        if not self.is_recording:
            return

        # 确保关节顺序一致
        if len(msg.name) < 6: return
        sorted_indices = sorted(range(len(msg.name)), key=lambda k: msg.name[k])
        
        names = [msg.name[i] for i in sorted_indices]
        positions = [msg.position[i] for i in sorted_indices]
        velocities = [msg.velocity[i] if msg.velocity else 0.0 for i in sorted_indices]
        efforts = [msg.effort[i] if msg.effort else 0.0 for i in sorted_indices]
        
        current_time = msg.header.stamp.to_sec()
        
        accelerations = [0.0] * len(names)
        if self.last_vel is not None and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0.0001:
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
        rospy.loginfo(">>> Start Recording")

    def stop_recording(self, filename_suffix):
        self.is_recording = False
        rospy.loginfo(f"<<< Stop Recording. Samples: {len(self.data_buffer)}")
        self.save_data(filename_suffix)

    def save_data(self, suffix):
        if not self.data_buffer:
            return

        # 提取组名 (例如 "GroupA_Static_1" -> "GroupA")
        group_name = suffix.split('_')[0]
        csv_file = os.path.join(self.output_dir, f"fr5_data_{group_name}.csv")
        
        # 强制关节顺序
        target_order = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        
        # 构建表头 (仅在文件不存在时写入)
        header = ['timestamp']
        for name in target_order:
            header.append(f'{name}_pos')
        for name in target_order:
            header.append(f'{name}_vel')
        for name in target_order:
            header.append(f'{name}_acc')
        for name in target_order:
            header.append(f'{name}_eff')
            
        file_exists = os.path.isfile(csv_file)
        
        with open(csv_file, 'a') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(header)
                
            for record in self.data_buffer:
                # 建立当前记录中关节名到索引的映射
                name_to_idx = {name: i for i, name in enumerate(record['names'])}
                
                row = [record['timestamp']]
                
                # 1-6: Positions
                for target_name in target_order:
                    if target_name in name_to_idx:
                        idx = name_to_idx[target_name]
                        row.append(record['positions'][idx])
                    else:
                        row.append(0.0) # Should not happen if setup is correct

                # 7-12: Velocities
                for target_name in target_order:
                    if target_name in name_to_idx:
                        idx = name_to_idx[target_name]
                        row.append(record['velocities'][idx])
                    else:
                        row.append(0.0)

                # 13-18: Accelerations
                for target_name in target_order:
                    if target_name in name_to_idx:
                        idx = name_to_idx[target_name]
                        row.append(record['accelerations'][idx])
                    else:
                        row.append(0.0)

                # 19-24: Efforts
                for target_name in target_order:
                    if target_name in name_to_idx:
                        idx = name_to_idx[target_name]
                        row.append(record['efforts'][idx])
                    else:
                        row.append(0.0)
                        
                writer.writerow(row)
        
        rospy.loginfo(f"Data appended to {csv_file}")

    def execute_dense_trajectory(self, duration, sample_rate, calc_func):
        """
        生成并执行密集点轨迹 (用于正弦和Chirp)
        """
        # 强制指定关节名称顺序，确保与 SRDF/URDF 一致
        joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        
        # 获取当前关节值作为"中心位置" (q0)
        current_state = self.robot.get_current_state()
        center_joints = []
        current_joint_map = dict(zip(current_state.joint_state.name, current_state.joint_state.position))
        
        for name in joint_names:
            if name in current_joint_map:
                center_joints.append(current_joint_map[name])
            else:
                rospy.logerr(f"Joint {name} not found in current state!")
                return

        # 1. 预先移动到轨迹的起始点 (t=0)
        # 因为正弦轨迹在 t=0 时可能不等于 q0 (取决于相位)，直接执行会导致跳变
        start_joints = []
        for i in range(len(joint_names)):
            p, _, _ = calc_func(0.0, i, center_joints[i])
            start_joints.append(p)
            
        # 检查起始点偏差，如果较大则先移动过去
        max_diff = max([abs(s - c) for s, c in zip(start_joints, center_joints)])
        if max_diff > 0.001:
            rospy.loginfo(f"Moving to trajectory start point (max diff: {max_diff:.4f})...")
            self.move_group.go(start_joints, wait=True)
            time.sleep(0.5)

        # 2. 生成完整轨迹
        traj_msg = RobotTrajectory()
        traj_msg.joint_trajectory.header.frame_id = self.robot.get_planning_frame()
        traj_msg.joint_trajectory.joint_names = joint_names
        
        times = np.arange(0, duration, 1.0/sample_rate)
        
        for t in times:
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(t)
            
            pos_list = []
            vel_list = []
            acc_list = []
            
            # 对每个关节计算 t 时刻状态
            for i in range(len(joint_names)):
                # 注意：这里始终使用 center_joints 作为 q0，保证震荡中心不变
                p, v, a = calc_func(t, i, center_joints[i])
                pos_list.append(p)
                vel_list.append(v)
                acc_list.append(a)
                
            point.positions = pos_list
            point.velocities = vel_list
            point.accelerations = acc_list
            traj_msg.joint_trajectory.points.append(point)
            
        rospy.loginfo(f"Executing dense trajectory ({duration}s, {len(times)} points)...")
        self.move_group.execute(traj_msg, wait=True)

    def run_trajectories(self):
        # 回到零位
        rospy.loginfo("Moving to Home...")
        self.move_group.set_named_target("zero") # 假设SRDF里有 zero 或 home
        self.move_group.go(wait=True)
        time.sleep(1.0)
        
        # ==========================================
        # 1. 轨迹组 A: 静态姿态扫描 [cite: 27]
        # ==========================================
        rospy.loginfo("--- Starting Group A: Static Pose Scanning ---")
        M = 20 # 论文建议20
        for i in range(M):
            rospy.loginfo(f"Group A: Pose {i+1}/{M}")
            # 生成随机有效姿态
            self.move_group.set_random_target()
            
            self.start_recording()
            success = self.move_group.go(wait=True)
            if success:
                # 静态保持 1-2s [cite: 28]
                time.sleep(2.0)
            self.stop_recording(f"GroupA_Static_{i+1}")
            time.sleep(0.5)

        # ==========================================
        # 2. 轨迹组 B: 单关节多正弦 [cite: 10]
        # ==========================================
        rospy.loginfo("--- Starting Group B: Single Joint Multi-Sine ---")
        # 回到中心姿态
        self.move_group.set_named_target("zero")
        self.move_group.go(wait=True)
        
        for joint_idx in range(6): # 遍历6个关节
            rospy.loginfo(f"Group B: Exciting Joint {joint_idx+1}/6")
            
            # 每次重新随机相位，增加多样性
            self.generator.randomize_phases()

            # 定义该特定运动的计算逻辑
            def calc_single_joint(t, j_idx, q0):
                if j_idx == joint_idx:
                    return self.generator.get_multi_sine_point(t, j_idx, q0)
                else:
                    return q0, 0.0, 0.0 # 其他关节保持静止
            
            self.start_recording()
            # 执行 60秒 (论文建议 60-90s)
            self.execute_dense_trajectory(duration=60.0, sample_rate=50.0, calc_func=calc_single_joint)
            self.stop_recording(f"GroupB_MultiSine_J{joint_idx+1}")
            
            # 回归中心，准备下一次
            self.move_group.set_named_target("zero")
            self.move_group.go(wait=True)
            time.sleep(1.0)

        # ==========================================
        # 3. 轨迹组 C: 六关节耦合多正弦 
        # ==========================================
        rospy.loginfo("--- Starting Group C: Coupled Multi-Sine ---")
        
        # 执行 20 条 (论文建议 20 条)
        for i in range(20):
            rospy.loginfo(f"Group C: Trajectory {i+1}/20")
            self.move_group.set_named_target("zero")
            self.move_group.go(wait=True)
            
            # 随机化相位
            self.generator.randomize_phases()
            
            def calc_coupled(t, j_idx, q0):
                return self.generator.get_multi_sine_point(t, j_idx, q0)
                
            self.start_recording()
            # 执行 90秒 (论文建议 90-120s)
            self.execute_dense_trajectory(duration=90.0, sample_rate=50.0, calc_func=calc_coupled)
            self.stop_recording(f"GroupC_Coupled_{i+1}")

        # ==========================================
        # 4. 轨迹组 D: 扫频 Chirp 
        # ==========================================
        rospy.loginfo("--- Starting Group D: Chirp Sweep ---")
        
        # 执行 6 条 (假设为单关节分别扫频，或者6次耦合扫频)
        # 这里实现为单关节分别扫频，覆盖所有关节
        for joint_idx in range(6):
            rospy.loginfo(f"Group D: Chirp Joint {joint_idx+1}/6")
            self.move_group.set_named_target("zero")
            self.move_group.go(wait=True)
            
            def calc_chirp(t, j_idx, q0):
                if j_idx == joint_idx:
                    return self.generator.get_chirp_point(t, 60.0, q0)
                else:
                    return q0, 0.0, 0.0
                
            self.start_recording()
            # 执行 60秒 (论文建议 60s)
            self.execute_dense_trajectory(duration=60.0, sample_rate=50.0, calc_func=calc_chirp)
            self.stop_recording(f"GroupD_Chirp_J{joint_idx+1}")

        # ==========================================
        # 5. 轨迹组 E: 随机点到点 S-curve [cite: 40]
        # ==========================================
        rospy.loginfo("--- Starting Group E: Random PTP Sequence ---")
        # 连续执行 10 条序列
        M_ptp = 10 
        
        self.start_recording()
        for i in range(M_ptp):
            rospy.loginfo(f"Group E: Move {i+1}/{M_ptp}")
            self.move_group.set_random_target()
            # MoveIt 默认规划器通常生成五次多项式或梯形速度 (接近 S-curve)
            self.move_group.go(wait=True)
            # 段间停顿 [cite: 41]
            time.sleep(1.0) 
            
        self.stop_recording("GroupE_PTP")

        rospy.loginfo("All Data Collection Completed!")

if __name__ == '__main__':
    try:
        collector = DataCollector()
        time.sleep(1.0)
        collector.run_trajectories()
    except rospy.ROSInterruptException:
        pass