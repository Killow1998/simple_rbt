#!/usr/bin/env python3
import threading
import tkinter as tk
from tkinter import ttk

import rospy
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointPublisher:
    def __init__(self):
        self.trajectory_pub = rospy.Publisher('/joint_group_effort_controller/command', JointTrajectory, queue_size=10)
        self.velocity_pub = rospy.Publisher('/joint_group_velocity_controller/command', Float64MultiArray, queue_size=10)

        self.trajectory = [
            0.0, 1.0143535137176514, -2.0287070274353027,
            0.0, 1.0143535137176514, -2.0287070274353027,
            0.0, 1.0143535137176514, -2.0287070274353027,
            0.0, 1.0143535137176514, -2.0287070274353027,
        ]
        self.trajectory_joint_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]

        self.velocity = [0.0, 0.0, 0.0, 0.0]
        self.velocity_joint_names = [
            'FL_foot_joint',
            'FR_foot_joint',
            'RL_foot_joint',
            'RR_foot_joint'
        ]

    def publish_trajectory(self, positions):
        msg = JointTrajectory()
        msg.joint_names = self.trajectory_joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration.from_sec(0.016666666)
        msg.points.append(point)
        self.trajectory_pub.publish(msg)

    def publish_velocity(self, velocities):
        msg = Float64MultiArray()
        msg.data = velocities
        self.velocity_pub.publish(msg)


def main():
    rospy.init_node('go2w_joint_publisher')
    node = JointPublisher()

    positions = node.trajectory.copy()
    velocities = node.velocity.copy()

    def on_slider_change(idx, var):
        if idx < len(positions):
            positions[idx] = var.get()
            node.publish_trajectory(positions)
        else:
            velocities[idx - len(positions)] = var.get()
            node.publish_velocity(velocities)

    def ros_spin():
        rospy.spin()

    spin_thread = threading.Thread(target=ros_spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title('Go2W Low Level Controller')

    for i, name in enumerate(node.trajectory_joint_names + node.velocity_joint_names):
        frame = ttk.Frame(root)
        frame.pack(fill='x', padx=5, pady=2)
        if i == 0:
            label = ttk.Label(frame, text='\tTrajectory Joints')
            label.pack(side='left')
            frame = ttk.Frame(root)
            frame.pack(fill='x', padx=5, pady=2)
        elif i == len(node.trajectory_joint_names):
            label = ttk.Label(frame, text='\tVelocity Joints')
            label.pack(side='left')
            frame = ttk.Frame(root)
            frame.pack(fill='x', padx=5, pady=2)

        label = ttk.Label(frame, text=f"{i+1}. {name}")
        label.pack(side='left')
        var = tk.DoubleVar(value=(positions + velocities)[i])

        if i in [0, 3, 6, 9]:
            slider = ttk.Scale(frame, from_=-1.0472, to=1.0472, orient='horizontal', variable=var,
                               command=lambda _val, idx=i, v=var: on_slider_change(idx, v))
        elif i in [1, 4, 7, 10]:
            slider = ttk.Scale(frame, from_=-1.5708, to=3.4907, orient='horizontal', variable=var,
                               command=lambda _val, idx=i, v=var: on_slider_change(idx, v))
        elif i in [2, 5, 8, 11]:
            slider = ttk.Scale(frame, from_=-2.7227, to=-0.83776, orient='horizontal', variable=var,
                               command=lambda _val, idx=i, v=var: on_slider_change(idx, v))
        else:
            slider = ttk.Scale(frame, from_=-5.0, to=5.0, orient='horizontal', variable=var,
                               command=lambda _val, idx=i, v=var: on_slider_change(idx, v))

        slider.pack(side='right', fill='none', expand=False, padx=5)
        slider.configure(length=100)
        frame.pack_propagate(False)
        frame.configure(width=230, height=30)

    def on_close():
        rospy.signal_shutdown('GUI closed')
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
