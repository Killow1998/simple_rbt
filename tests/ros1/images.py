"""Independent Noetic image receiver for the direct roscpp wrapper."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import rospy
from sensor_msgs.msg import Image

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rospy.init_node("simple_rbt_direct_image_oracle", anonymous=True)
    frames, subscriptions = {}, []
    for prefix in ["/camera", "/camera/panorama"]:
        for suffix in ["/image_raw", "/depth/image_raw"]:
            topic = prefix + suffix
            frames[topic] = {}
            def receive(message, key=topic):
                frames[key][str(message.header.stamp.to_nsec())] = hashlib.sha256(message.data).hexdigest()
                while len(frames[key]) > 32:
                    frames[key].pop(next(iter(frames[key])))
            subscriptions.append(rospy.Subscriber(topic, Image, receive, queue_size=10, buff_size=4*1024*1024))
    (args.output / "images-ready").touch()
    deadline, drained = time.monotonic() + 50, None
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if (args.output / "stop").exists():
            if drained is None:
                drained = time.monotonic() + 1.
            if time.monotonic() >= drained:
                break
        time.sleep(.05)
    for subscription in subscriptions:
        subscription.unregister()
    (args.output / "images.json").write_text(json.dumps(frames, indent=2)+"\n")
    rospy.signal_shutdown("Complete")

if __name__ == "__main__":
    main()
