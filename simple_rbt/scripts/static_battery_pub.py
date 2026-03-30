#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import BatteryState


class StaticBatteryPublisher:
    def __init__(self):
        self.topic = rospy.get_param("~topic", "battery")
        self.publish_rate = max(float(rospy.get_param("~publish_rate", 1.0)), 1e-3)
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.voltage = float(rospy.get_param("~voltage", 24.0))
        self.percentage = min(max(float(rospy.get_param("~percentage", 1.0)), 0.0), 1.0)
        self.present = bool(rospy.get_param("~present", True))
        self.status = int(rospy.get_param(
            "~power_supply_status", BatteryState.POWER_SUPPLY_STATUS_FULL
        ))
        self.technology = int(rospy.get_param(
            "~power_supply_technology", BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        ))
        self.publisher = rospy.Publisher(self.topic, BatteryState, queue_size=1, latch=True)

    def make_message(self):
        msg = BatteryState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.voltage = self.voltage
        nan = float("nan")
        msg.temperature = nan
        msg.current = nan
        msg.charge = nan
        msg.capacity = nan
        msg.design_capacity = nan
        msg.percentage = self.percentage
        msg.power_supply_status = self.status
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = self.technology
        msg.present = self.present
        return msg

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.publisher.publish(self.make_message())
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("battery_state_publisher")
    StaticBatteryPublisher().run()
