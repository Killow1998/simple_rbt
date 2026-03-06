#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class OmniCommander:
    def __init__(self):
        rospy.init_node('omni_commander', anonymous=True)
        self.pub1 = rospy.Publisher('/first_wheel_controller/command', Float64, queue_size=10)
        self.pub2 = rospy.Publisher('/second_wheel_controller/command', Float64, queue_size=10)
        self.pub3 = rospy.Publisher('/third_wheel_controller/command', Float64, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb)
        
        self.alpha = math.pi / 2.0
        self.L = 0.088
        self.multiplier = 50.0 

    def cmd_cb(self, msg):
        vel_x = msg.linear.x
        vel_y = msg.linear.y
        vel_theta = msg.angular.z

        # Kinematics of 3-omniwheel robot
        vel1 = self.multiplier * (-math.sin(self.alpha)*vel_x + math.cos(self.alpha)*vel_y + self.L*vel_theta)
        vel2 = self.multiplier * (-math.sin(self.alpha + 2*math.pi/3)*vel_x + math.cos(self.alpha + 2*math.pi/3)*vel_y + self.L*vel_theta)
        vel3 = self.multiplier * (-math.sin(self.alpha - 2*math.pi/3)*vel_x + math.cos(self.alpha - 2*math.pi/3)*vel_y + self.L*vel_theta)

        self.pub1.publish(vel1)
        self.pub2.publish(vel2)
        self.pub3.publish(vel3)

if __name__ == '__main__':
    try:
        OmniCommander()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
