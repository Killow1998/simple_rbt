"""Real Noetic publishers/subscribers for the optional Docker E2E test."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage


def sha(data):
    return hashlib.sha256(data).hexdigest()


def payload(message):
    if isinstance(message, Clock):
        return {"stamp_ns": message.clock.to_nsec()}
    if isinstance(message, TFMessage):
        return [{"parent": t.header.frame_id, "child": t.child_frame_id,
                 "position": [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z],
                 "xyzw": [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]}
                for t in message.transforms]
    result = {"stamp_ns": message.header.stamp.to_nsec(), "frame_id": message.header.frame_id}
    if isinstance(message, Image):
        result.update(width=message.width, height=message.height, encoding=message.encoding,
                      step=message.step, bytes=len(message.data), sha256=sha(message.data))
    elif isinstance(message, CustomMsg):
        digest = hashlib.sha256()
        digest.update(b"".join(struct.pack("<fff", p.x, p.y, p.z) for p in message.points))
        digest.update(b"".join(struct.pack("<I", p.offset_time) for p in message.points))
        digest.update(bytes(p.line for p in message.points))
        result.update(timebase=message.timebase, point_num=message.point_num,
                      actual_points=len(message.points), sha256=digest.hexdigest(),
                      max_offset_ns=max((p.offset_time for p in message.points), default=0),
                      attributes_match=all(p.tag == 0 and p.reflectivity == 100 for p in message.points),
                      lidar_id=message.lidar_id, reserved=list(message.rsvd))
    elif isinstance(message, PointCloud2):
        result.update(width=message.width, point_step=message.point_step, row_step=message.row_step,
                      sha256=sha(message.data), fields=[f.name for f in message.fields])
    elif isinstance(message, Imu):
        result.update(gyro=[message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z],
                      acceleration=[message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z],
                      orientation_covariance=list(message.orientation_covariance))
    elif isinstance(message, CameraInfo):
        result.update(width=message.width, height=message.height, K=list(message.K))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("robot")
    parser.add_argument("output", type=Path)
    parser.add_argument("--settle", type=float, default=2.)
    args = parser.parse_args()
    rospy.init_node("simple_rbt_noetic_probe", anonymous=True, disable_signals=True)
    latest, counts, velocities, first_stamps = {}, {}, [], {}
    lock = threading.Lock()
    types = {"/clock": Clock, "/livox/lidar": CustomMsg, "/livox/points": PointCloud2,
             "/livox/imu": Imu, "/livox/imu_si": Imu, "/odom": Odometry,
             "/tf": TFMessage, "/tf_static": TFMessage,
             "/camera/image_raw": Image, "/camera/depth/image_raw": Image,
             "/camera/panorama/image_raw": Image, "/camera/panorama/depth/image_raw": Image,
             "/camera/camera_info": CameraInfo}

    def receive(message, topic):
        with lock:
            latest[topic] = message
            counts[topic] = counts.get(topic, 0) + 1
            if hasattr(message, "header"):
                first_stamps.setdefault(topic, message.header.stamp.to_nsec())
            if topic == "/odom":
                velocities.append((time.monotonic(), message.twist.twist.linear.x))

    subscriptions = [rospy.Subscriber(topic, kind, receive, callback_args=topic,
                                     queue_size=64 if kind in (Imu, Clock) else 5,
                                     buff_size=4 * 1024 * 1024)
                     for topic, kind in types.items()]
    own = rospy.Publisher("/" + args.robot + "/cmd_vel", Twist, queue_size=5)
    wrong = rospy.Publisher("/unrelated_robot/cmd_vel", Twist, queue_size=5)
    global_command = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    checks = {}
    deadline = time.monotonic() + 16
    while time.monotonic() < deadline:
        with lock:
            ready = (all(topic in latest for topic in types)
                     and latest["/clock"].clock.to_sec() >= args.settle)
        if ready and own.get_num_connections():
            break
        time.sleep(.05)
    else:
        raise RuntimeError("Noetic discovery incomplete: " + str(sorted(latest)))

    phases = []
    for name, speed, duration in [("forward", .3, 2.), ("reverse", -.25, 2.), ("deadman", None, 1.8)]:
        began = time.monotonic()
        while time.monotonic() - began < duration:
            if speed is not None:
                command = Twist(); command.linear.x = speed; own.publish(command)
            foreign = Twist(); foreign.linear.x = -.8
            wrong.publish(foreign); global_command.publish(foreign)
            time.sleep(.05)
        with lock:
            samples = [v for stamp, v in velocities if stamp >= began + duration * .6]
        mean = sum(samples) / len(samples) if samples else None
        phases.append({"name": name, "mean_vx_m_s": mean, "samples": len(samples)})
        checks[name] = bool(samples) and (mean > .03 if name == "forward" else
                                        mean < -.03 if name == "reverse" else abs(mean) < .15)

    deadline = time.monotonic() + 35
    while not (args.output / "stop").exists() and time.monotonic() < deadline:
        time.sleep(.05)
    if not (args.output / "stop").exists():
        raise RuntimeError("Host runtime did not finish within the Noetic deadline")
    time.sleep(1.)  # Drain ROS 1 TCPROS before comparing the final runtime frame.
    with lock:
        messages = dict(latest)
        received = dict(counts)
    for topic, message in messages.items():
        if isinstance(message, Image):
            name = topic.strip("/").replace("/", "-")
            (args.output / (name + ".bin")).write_bytes(bytes(message.data))
    checks["all_topics_received"] = all(received.get(topic, 0) > 0 for topic in types)
    delivery = {}
    for topic, first in first_stamps.items():
        period = 5_000_000 if topic.startswith("/livox/imu") else 100_000_000
        expected = (messages[topic].header.stamp.to_nsec() - first) // period + 1
        delivery[topic] = {"received": received[topic], "expected_in_received_window": expected}
        checks["complete_delivery_" + topic] = received[topic] == expected
    report = {"delivery": delivery, "checks": checks, "passed": all(checks.values()), "robot": args.robot,
              "command_topic": own.resolved_name, "phases": phases, "received": received,
              "latest": {topic: payload(message) for topic, message in messages.items()}}
    (args.output / "ros1.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    for subscription in subscriptions:
        subscription.unregister()
    rospy.signal_shutdown("Noetic E2E complete")
    print(json.dumps({"robot": args.robot, "passed": report["passed"], "checks": checks}))


if __name__ == "__main__":
    main()
