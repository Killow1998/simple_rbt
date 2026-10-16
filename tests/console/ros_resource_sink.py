"""Read-only ROS traffic receiver; its cost is reported separately from SpRobo."""
import json
import sys
import time
import rospy

seconds, destination = float(sys.argv[1]), sys.argv[2]
rospy.init_node("sprobo_resource_receiver", anonymous=True, disable_signals=True)
topics = ["/livox/lidar", "/livox/imu", "/camera/image_raw", "/camera/depth/image_raw",
          "/camera/imu", "/livox/points", "/camera/panorama/image_raw",
          "/camera/panorama/depth/image_raw", "/odom", "/clock"]
counts = {topic: dict(messages=0, bytes=0) for topic in topics}

def receive(message, topic):
    counts[topic]["messages"] += 1
    counts[topic]["bytes"] += len(message._buff)

subscriptions = [rospy.Subscriber(topic, rospy.AnyMsg, receive, topic, queue_size=1,
                                  buff_size=16*1024*1024) for topic in topics]
start = time.monotonic()
while time.monotonic()-start < seconds and not rospy.is_shutdown():
    time.sleep(.1)
elapsed = time.monotonic()-start
with open(destination,"w") as file:
    json.dump(dict(elapsed_s=elapsed, topics=counts,
                   published_types=rospy.get_published_topics()),file,indent=2)
rospy.signal_shutdown("measurement complete")
