#!/usr/bin/env python3
import threading
import tkinter as tk
from tkinter import ttk

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class CmdVelPublisher:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.velocity_pub = rospy.Publisher('/joint_group_velocity_controller/command', Float64MultiArray, queue_size=10)
        self.velocity_joint_names = [
            'FL_foot_joint',
            'FR_foot_joint',
            'RL_foot_joint',
            'RR_foot_joint'
        ]

    def publish_cmd_vel(self, linear_x, linear_y, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def publish_velocity(self, velocities):
        msg = Float64MultiArray()
        msg.data = velocities
        self.velocity_pub.publish(msg)


def main():
    rospy.init_node('go2w_high_level_publisher')
    node = CmdVelPublisher()

    root = tk.Tk()
    root.title('Go2W High Level Controller')

    linear_x = tk.DoubleVar(value=0.0)
    linear_y = tk.DoubleVar(value=0.0)
    angular_z = tk.DoubleVar(value=0.0)
    velocity_vars = [tk.DoubleVar(value=0.0) for _ in node.velocity_joint_names]

    def on_slider_change(_=None):
        node.publish_cmd_vel(linear_x.get(), linear_y.get(), angular_z.get())
        velocities = [v.get() for v in velocity_vars]
        node.publish_velocity(velocities)

    def on_button_click(var):
        var.set(0.0)
        on_slider_change()

    def ros_spin():
        rospy.spin()

    spin_thread = threading.Thread(target=ros_spin, daemon=True)
    spin_thread.start()

    sliders_info = [
        ("Linear  X", linear_x, -0.4, 0.4),
        ("Linear  Y", linear_y, -0.3, 0.3),
        ("Angular Z", angular_z, -0.6, 0.6)
    ]

    for label_text, var, minval, maxval in sliders_info:
        frame = ttk.Frame(root)
        frame.pack(fill='x', padx=5, pady=2)
        label = ttk.Label(frame, text=label_text)
        label.pack(side='left')
        label.configure(width=8)
        button = ttk.Button(frame, text='Reset', command=lambda v=var: on_button_click(v))
        button.pack(side='right', padx=5)
        slider = ttk.Scale(frame, from_=minval, to=maxval, orient='horizontal', variable=var, command=on_slider_change)
        slider.pack(side='right', fill='x', expand=True, padx=5)
        slider.configure(length=150)
        frame.pack_propagate(False)
        frame.configure(width=400, height=40)

    velocity_label = ttk.Label(root, text='Velocity Joints')
    velocity_label.pack(pady=(10, 0))
    for i, joint_name in enumerate(node.velocity_joint_names):
        frame = ttk.Frame(root)
        frame.pack(fill='x', padx=5, pady=2)
        label = ttk.Label(frame, text=joint_name)
        label.pack(side='left')
        button = ttk.Button(frame, text='Reset', command=lambda v=velocity_vars[i]: on_button_click(v))
        button.pack(side='right', padx=5)
        slider = ttk.Scale(frame, from_=-5.0, to=5.0, orient='horizontal', variable=velocity_vars[i], command=on_slider_change)
        slider.pack(side='right', fill='x', expand=True, padx=5)
        slider.configure(length=150)
        frame.pack_propagate(False)
        frame.configure(width=400, height=40)

    def on_close():
        rospy.signal_shutdown('GUI closed')
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
