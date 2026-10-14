"""Independent Humble receiver for byte-exact ROS 2/Noetic image comparisons."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Node("simple_rbt_noetic_image_oracle")
    frames = {}
    subscriptions = []
    for prefix in ["/camera", "/camera/panorama"]:
        for suffix in ["/image_raw", "/depth/image_raw"]:
            topic = prefix + suffix
            frames[topic] = {}

            def receive(message, key=topic):
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                frames[key][str(stamp)] = hashlib.sha256(message.data).hexdigest()
                while len(frames[key]) > 32:
                    frames[key].pop(next(iter(frames[key])))

            subscriptions.append(node.create_subscription(Image, topic, receive, QoSProfile(depth=10)))
    (args.output / "ros2-ready").touch()
    deadline = time.monotonic() + 50
    drained = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        if (args.output / "stop").exists():
            if drained is None:
                drained = time.monotonic() + 1.
            if time.monotonic() >= drained:
                break
    (args.output / "ros2-images.json").write_text(json.dumps(frames, indent=2) + "\n")
    for subscription in subscriptions:
        node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
