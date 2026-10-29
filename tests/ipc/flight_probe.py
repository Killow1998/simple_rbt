"""Independent native ROS flight/sensor observer; test code only."""
import atexit, argparse, json, math, statistics, subprocess, sys, threading, time
from pathlib import Path

parser=argparse.ArgumentParser()
parser.add_argument('version',choices=['ros1','ros2']);parser.add_argument('output')
parser.add_argument('--motion-only',action='store_true');parser.add_argument('--no-camera',action='store_true')
args=parser.parse_args();seen={};stamps={};samples=[]
def ns(t):return t.sec*10**9+t.nanosec if args.version=='ros2' else t.to_nsec()
def receive(name):
    def callback(msg):
        seen[name]=msg
        if hasattr(msg,'header'):stamps.setdefault(name,[]).append(ns(msg.header.stamp))
        if name=='odom':
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            samples.append([time.monotonic(),p.x,p.y,p.z,2*math.atan2(q.z,q.w)])
    return callback
if args.version=='ros1':
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu,Image,CameraInfo
    from livox_ros_driver2.msg import CustomMsg
    rospy.init_node('flight_oracle',anonymous=True)
    publish=lambda cls,topic:rospy.Publisher(topic,cls,queue_size=10)
    subscribe=lambda cls,topic,key:rospy.Subscriber(topic,cls,receive(key),queue_size=10)
else:
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu,Image,CameraInfo
    from livox_ros_driver2.msg import CustomMsg
    rclpy.init();node=rclpy.create_node('flight_motion_oracle' if args.motion_only else 'flight_sensor_oracle')
    publish=lambda cls,topic:node.create_publisher(cls,topic,10)
    subscribe=lambda cls,topic,key:node.create_subscription(cls,topic,receive(key),10)
    halt=threading.Event()
    def spin():
        while not halt.is_set():rclpy.spin_once(node,timeout_sec=.02)
    reader=threading.Thread(target=spin,daemon=True);reader.start()
    def shutdown():
        halt.set();reader.join(timeout=3);node.destroy_node();rclpy.shutdown()
    atexit.register(shutdown)
subscriptions=[subscribe(Odometry,'/ground_truth/odom','odom')]
checks={};phases={}
if args.motion_only:
    command=publish(Twist,'/skydio_x2/cmd_vel');action=publish(String,'/skydio_x2/flight_action')
    wrong=publish(Twist,'/cmd_vel')
    deadline=time.monotonic()+8
    while 'odom' not in seen and time.monotonic()<deadline:time.sleep(.05)
    if 'odom' not in seen:raise RuntimeError('Missing flight ground truth')
    time.sleep(.5)
    def phase(name,seconds,kind=None,velocity=(0,0,0),yaw=0,roll=0):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if kind in ['takeoff','land','hold']:
                m=String();m.data=kind;action.publish(m)
            elif kind=='velocity':
                m=Twist();m.linear.x,m.linear.y,m.linear.z=map(float,velocity);m.angular.z=float(yaw);m.angular.x=float(roll);command.publish(m)
            foreign=Twist();foreign.linear.x=-1.;wrong.publish(foreign)
            time.sleep(.04)
        phases[name]=samples[-1]
    phase('takeoff',7,'takeoff');phase('forward',3,'velocity',(.4,0,0))
    phase('hold',2,'hold');phase('climb',3,'velocity',(0,0,.25))
    phase('yaw',2,'velocity',yaw=.4);phase('invalid_roll',2,'velocity',(.9,0,0),roll=.5)
    phase('expiry',2);phase('landing',8,'land')
    checks={'takeoff':.85<phases['takeoff'][3]<1.1,
            'forward':phases['forward'][1]-phases['takeoff'][1]>.7,
            'climb':phases['climb'][3]-phases['hold'][3]>.4,
            'yaw':abs(phases['yaw'][4]-phases['climb'][4])>.5,
            'unsupported_roll_rejected':math.dist(phases['invalid_roll'][1:4],phases['yaw'][1:4])<.15,
            'expiry_hover':math.dist(phases['expiry'][1:4],phases['invalid_roll'][1:4])<.08,
            'landed':abs(phases['landing'][3])<.04}
else:
    subscriptions += [subscribe(Imu,'/livox/imu_si','imu'),subscribe(Imu,'/camera/imu','camera_imu'),
                      subscribe(CustomMsg,'/livox/lidar','lidar')]
    if not args.no_camera:
        subscriptions += [subscribe(Image,'/camera/image_raw','rgb'),subscribe(Image,'/camera/depth/image_raw','depth'),
                          subscribe(Image,'/camera/panorama/image_raw','panorama'),subscribe(CameraInfo,'/camera/camera_info','info')]
    output=Path(args.output);motion=output.with_suffix('.motion.json')
    subprocess.run([sys.executable,__file__,args.version,str(motion),'--motion-only'],check=True,timeout=45)
    report=json.loads(motion.read_text());checks.update(report['checks']);phases=report['phases']
    for stream,interval in [('imu',5_000_000),('camera_imu',5_000_000),('lidar',100_000_000)]:
        t=stamps.get(stream,[]);dt=[b-a for a,b in zip(t,t[1:])]
        checks[stream+'_frequency']=len(t)>20 and statistics.median(dt)==interval and all(v>0 and v%interval==0 for v in dt)
    if 'lidar' in seen:
        m=seen['lidar'];checks['custom_message']=m.point_num==len(m.points)>100 and m.timebase==ns(m.header.stamp) and all(0<=p.line<4 and 0<=p.offset_time<100_000_000 for p in m.points)
    if not args.no_camera:
        for name,bytes_per_pixel in [('rgb',3),('depth',4),('panorama',3)]:
            m=seen.get(name);checks[name+'_complete']=m is not None and len(m.data)==m.width*m.height*bytes_per_pixel
        checks['camera_calibration']='info' in seen and seen['info'].width==seen['rgb'].width
result={'passed':all(checks.values()),'checks':checks,'phases':phases,
        'sample_counts':{k:len(v) for k,v in stamps.items()}}
Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'passed':result['passed'],'checks':checks}),flush=True)
sys.exit(0 if result['passed'] else 1)
