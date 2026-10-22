"""Independent ROS observer: validate a separate C++ adapter via real middleware."""
import argparse, json, math, os, statistics, struct, subprocess, sys, threading, time
parser=argparse.ArgumentParser()
parser.add_argument("version", choices=["ros1","ros2"])
parser.add_argument("output")
parser.add_argument("--physical", action="store_true")
parser.add_argument("--robot", default="go2w")
parser.add_argument("--external-odometry", action="store_true")
parser.add_argument("--camera-imu", action="store_true")
parser.add_argument("--no-camera", action="store_true")
parser.add_argument("--motion-only", action="store_true", help=argparse.SUPPRESS)
args=parser.parse_args();version,output=args.version,args.output
seen = {}
odom_topic = "/ground_truth/odom" if args.external_odometry else "/odom"
odometry_samples=[]
lidar_stamps=[]
camera_imu_stamps=[]
imu_stamps=[]
custom_frames={}
cloud_frames={}
def stamp_ns(stamp):
    return stamp.sec*1000000000+stamp.nanosec if version=='ros2' else stamp.to_nsec()
def centred_imu_clock(stamps):
    intervals = [b-a for a,b in zip(stamps, stamps[1:])]
    return (len(stamps)>100 and statistics.median(intervals)==5000000
            and all(ns%5000000==2500000 for ns in stamps)
            and all(dt>0 and dt%5000000==0 for dt in intervals))

def received(name):
    def callback(message):
        seen[name] = message
        if name=="camera_imu":camera_imu_stamps.append(stamp_ns(message.header.stamp))
        if name=="imu":imu_stamps.append(stamp_ns(message.header.stamp))
        if name=="odom":odometry_samples.append((time.monotonic(),message.twist.twist.linear.x))
        if name in ('lidar','points'):
            ns=stamp_ns(message.header.stamp)
            frames=custom_frames if name=='lidar' else cloud_frames
            frames[ns]=message
            while len(frames)>4:frames.pop(next(iter(frames)))
            if name=='lidar':lidar_stamps.append((ns,message.timebase))
    return callback
if version == 'ros2':
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, Image, PointCloud2, CameraInfo
    from livox_ros_driver2.msg import CustomMsg
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    rclpy.init(); node = rclpy.create_node('ipc_ros_motion_oracle' if args.motion_only else 'ipc_ros_oracle')
    publisher = node.create_publisher(Twist, '/' + args.robot + '/cmd_vel', 10)
    wrong = node.create_publisher(Twist, '/unrelated_robot/cmd_vel', 10)
    global_command = node.create_publisher(Twist, '/cmd_vel', 10)
    subscribe = lambda cls,topic,key,qos=10: node.create_subscription(cls,topic,received(key),qos)
    clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    static_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # Receiving high-rate sensors must not share the command pacing loop: a
    # single spin_once per command can leave odometry behind image/IMU events.
    # ROS 1 already receives subscriptions on its own callback threads.
    reader_stop = threading.Event()
    reader_errors = []
    def receive_loop():
        try:
            while not reader_stop.is_set():
                rclpy.spin_once(node,timeout_sec=.02)
        except Exception as error:
            reader_errors.append(str(error))
    reader = threading.Thread(target=receive_loop,daemon=True)
    tick = lambda: time.sleep(.02)
else:
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, Image, PointCloud2, CameraInfo
    from livox_ros_driver2.msg import CustomMsg
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    rospy.init_node('ipc_ros_oracle', anonymous=True)
    publisher = rospy.Publisher('/' + args.robot + '/cmd_vel',Twist,queue_size=10)
    wrong = rospy.Publisher('/unrelated_robot/cmd_vel',Twist,queue_size=10)
    global_command = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    subscribe = lambda cls,topic,key,qos=10: rospy.Subscriber(topic,cls,received(key),queue_size=10)
    clock_qos = static_qos = 10
    tick = lambda: time.sleep(.02)
subscriptions = [subscribe(Clock,'/clock','clock',clock_qos), subscribe(Imu,'/livox/imu_si','imu'),
    subscribe(CustomMsg,'/livox/lidar','lidar'),subscribe(PointCloud2,'/livox/points','points'),
    subscribe(Odometry,odom_topic,'odom'),subscribe(TFMessage,'/tf_static','tf',static_qos)]
expected=['clock','imu','lidar','points','odom','tf']
subscriptions += [subscribe(TFMessage,'/tf','dynamic_tf')]
if args.external_odometry:
    subscriptions += [subscribe(Odometry,'/odom','reserved_odom')]
if args.motion_only:
    for subscription in subscriptions:
        node.destroy_subscription(subscription)
    subscriptions=[subscribe(Clock,'/clock','clock',clock_qos),subscribe(Odometry,odom_topic,'odom')]
    expected=['clock','odom']
elif not args.no_camera:
    subscriptions += [subscribe(CameraInfo,'/camera/camera_info','info'),
        subscribe(Image,'/camera/image_raw','rgb'),subscribe(Image,'/camera/depth/image_raw','depth'),
        subscribe(Image,'/camera/panorama/image_raw','panorama')]
    expected += ['info','rgb','depth','panorama']
if args.camera_imu and not args.motion_only:
    subscriptions += [subscribe(Imu,'/camera/imu','camera_imu')]
    expected += ['camera_imu']
if args.physical and not args.no_camera and not args.motion_only:
    subscriptions += [subscribe(CameraInfo,'/camera/depth/camera_info','depth_info')]
    expected += ['depth_info']
if version == 'ros2':reader.start()
velocities=[]
if args.physical:
    # Wait for full sensor discovery before commanding a settling legged robot.
    discovery=time.monotonic()
    while time.monotonic()-discovery<12:
        tick()
        if not all(name in seen for name in expected):continue
        clock=seen['clock'].clock
        simtime=clock.sec+clock.nanosec/1e9 if version=='ros2' else clock.to_sec()
        if simtime>=2.:break
start=time.monotonic()
motion_result=None
if version=='ros2' and args.physical and not args.motion_only:
    # Python CustomMsg conversion can monopolize an executor/GIL long enough to
    # overwrite odometry history. Keep the full sensor observer running while
    # an independent ROS process owns command pacing and motion observation.
    # No production publication rate, payload, or acceptance threshold changes.
    motion_path=output+'.motion.json'
    child=subprocess.Popen([sys.executable,__file__,version,motion_path,'--physical','--motion-only','--robot',args.robot]+(['--external-odometry'] if args.external_odometry else []))
    try:
        child.wait(timeout=25.)
    finally:
        if child.poll() is None:child.terminate();child.wait(timeout=3.)
    with open(motion_path) as stream:motion_result=json.load(stream)
else:
    while time.monotonic()-start<(6.5 if args.physical else 7.):
        elapsed=time.monotonic()-start
        cmd=Twist();cmd.linear.x=(.3 if elapsed<2. else -.25 if elapsed<4. else 0.) if args.physical else (.4 if elapsed<4. else 0.)
        if not args.physical or elapsed<4.:publisher.publish(cmd)
        foreign=Twist();foreign.linear.x=-.8
        wrong.publish(foreign);global_command.publish(foreign)
        tick()
if version == 'ros2':
    reader_stop.set();reader.join(timeout=2.)
    if reader.is_alive() or reader_errors:raise RuntimeError('ROS subscriber executor: '+str(reader_errors))

checks={k:k in seen for k in expected}
if not args.motion_only:
    checks['dynamic_tf_owner'] = ('dynamic_tf' not in seen) if args.external_odometry else any(t.header.frame_id=='odom' and t.child_frame_id=='base_link' for t in seen.get('dynamic_tf', TFMessage()).transforms)
    if args.external_odometry: checks['estimated_odom_topic_reserved'] = 'reserved_odom' not in seen
if checks.get('imu'):checks['imu_si']=math.isclose(seen['imu'].linear_acceleration.z,9.80665)
if checks.get('lidar'):checks['lidar_columns']=seen['lidar'].point_num==2 and seen['lidar'].points[1].line==3 and seen['lidar'].points[1].offset_time==10000
if checks.get('rgb'):checks['image_complete']=len(seen['rgb'].data)==32*24*3 and all(x==91 for x in seen['rgb'].data)
if checks.get('depth'):checks['depth_complete']=len(seen['depth'].data)==32*24*4
if checks['odom']:checks['odometry_pose']=math.isclose(seen['odom'].pose.pose.position.x,.5)
if checks.get('tf'):checks['extrinsics']=any(t.child_frame_id=='livox_frame' and math.isclose(t.transform.translation.z,.3) for t in seen['tf'].transforms)
phases={}
livox={}
if checks.get('lidar'):
    custom=seen['lidar']
    checks['custom_stamp']=bool(lidar_stamps) and all(a==b for a,b in lidar_stamps)
    checks['custom_stamp_increases']=len(lidar_stamps)>2 and all(a[0]<b[0] for a,b in zip(lidar_stamps,lidar_stamps[1:]))
    checks['custom_header']=custom.header.frame_id=='livox_frame' and custom.timebase>0
    checks['custom_count']=custom.point_num==len(custom.points)>0
    checks['custom_device_fields']=custom.lidar_id==0 and list(custom.rsvd)==[0,0,0]
    checks['custom_point_fields']=all(math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z) and
        p.reflectivity==100 and p.tag==0 and 0<=p.line<4 and 0<=p.offset_time<100000000 for p in custom.points)
    checks['custom_point_offsets']=all(a.offset_time<=b.offset_time for a,b in zip(custom.points,custom.points[1:]))
    if not args.physical:
        checks['custom_fixture_xyz']=[(p.x,p.y,p.z,p.offset_time,p.line) for p in custom.points]==[(1.,2.,3.,0,0),(-1.,-2.,-3.,10000,3)]
    copies=custom_frames.copy();clouds=cloud_frames.copy();common=set(copies)&set(clouds)
    checks['custom_pointcloud2_agrees']=False
    if common:
        ns=max(common);c=copies[ns];cloud=clouds[ns]
        expected_fields=[('x',0,7,1),('y',4,7,1),('z',8,7,1),('intensity',12,7,1),('tag',16,2,1),('line',17,2,1),('timestamp',18,8,1)]
        valid=cloud.header.frame_id==c.header.frame_id and cloud.height==1 and cloud.width==c.point_num and cloud.point_step==26 and cloud.row_step==26*c.point_num and len(cloud.data)==cloud.row_step and not cloud.is_bigendian and cloud.is_dense
        valid=valid and [(f.name,f.offset,f.datatype,f.count) for f in cloud.fields]==expected_fields
        if valid:
            for index,p in enumerate(c.points):
                actual=struct.unpack_from('<ffffBBd',cloud.data,index*26)
                if actual!=(p.x,p.y,p.z,float(p.reflectivity),p.tag,p.line,float(c.timebase+p.offset_time)):
                    valid=False;break
        checks['custom_pointcloud2_agrees']=valid
        livox={'stamp_ns':ns,'points':c.point_num,'first_offset_ns':c.points[0].offset_time if c.points else None,
               'last_offset_ns':c.points[-1].offset_time if c.points else None,'frame_id':c.header.frame_id}
if args.physical:
    checks.pop('imu_si',None);checks.pop('lidar_columns',None);checks.pop('image_complete',None);checks.pop('depth_complete',None);checks.pop('odometry_pose',None);checks.pop('extrinsics',None)
    if 'tf' in seen:
        transforms={t.child_frame_id:t for t in seen['tf'].transforms}
        if 'livox_frame' in transforms and 'imu_frame' in transforms:
            l,i=transforms['livox_frame'].transform,transforms['imu_frame'].transform
            delta=[getattr(i.translation,a)-getattr(l.translation,a) for a in ['x','y','z']]
            q=[l.rotation.w,l.rotation.x,l.rotation.y,l.rotation.z];v=q[1:]
            # Inverse quaternion rotation into the MID360 device coordinates.
            cross=lambda a,b:[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
            c=cross(v,delta);cc=cross(v,c)
            local=[delta[a]-2*q[0]*c[a]+2*cc[a] for a in range(3)]
            checks['mid360_rigid_offset']=all(math.isclose(a,b,abs_tol=1e-8) for a,b in zip(local,[.011,.02329,-.04412]))
    if 'imu' in seen:checks['imu_finite']=all(math.isfinite(getattr(seen['imu'].linear_acceleration,a)) for a in ['x','y','z'])
    if 'lidar' in seen:checks['lidar_nonempty']=seen['lidar'].point_num==len(seen['lidar'].points)>0
    if 'rgb' in seen:checks['image_complete']=len(seen['rgb'].data)==seen['rgb'].width*seen['rgb'].height*3
    if 'depth' in seen:checks['depth_complete']=len(seen['depth'].data)==seen['depth'].width*seen['depth'].height*4
    if 'imu' in seen: checks['imu_200hz']=centred_imu_clock(imu_stamps)
    if args.camera_imu:
        imu=seen.get('camera_imu')
        checks['camera_imu_si_frame']=imu is not None and imu.header.frame_id=='camera_imu_optical_frame' and imu.orientation_covariance[0]==-1 and all(math.isfinite(getattr(imu.linear_acceleration,a)) for a in ['x','y','z'])
        checks['camera_imu_200hz']=centred_imu_clock(camera_imu_stamps)
        checks['camera_imu_optical_gravity']=imu is not None and -12<imu.linear_acceleration.y<-7 and abs(imu.linear_acceleration.z)<3
    if not args.no_camera and not args.motion_only:
        transforms={t.child_frame_id:t for t in seen.get('tf',TFMessage()).transforms}
        frames=['camera_optical_frame','camera_depth_optical_frame']+(['camera_imu_optical_frame'] if args.camera_imu else [])
        checks['camera_device_tf']=all(f in transforms for f in frames)
        if checks['camera_device_tf']:
            color=transforms['camera_optical_frame'].transform;depth=transforms['camera_depth_optical_frame'].transform
            checks['color_depth_baseline']=all(math.isclose(getattr(color.translation,a)-getattr(depth.translation,a),b,abs_tol=1e-8) for a,b in zip(['x','y','z'],[0,.015,0]))
            if args.camera_imu:
                imu_tf=transforms['camera_imu_optical_frame'].transform
                checks['d435i_rigid_offset']=all(math.isclose(getattr(imu_tf.translation,a)-getattr(depth.translation,a),b,abs_tol=1e-8) for a,b in zip(['x','y','z'],[-.01174,-.00552,.0051]))
        checks['separate_optical_headers']=seen.get('rgb') is not None and seen['rgb'].header.frame_id=='camera_optical_frame' and seen.get('depth') is not None and seen['depth'].header.frame_id=='camera_depth_optical_frame'
        checks['depth_camera_info']=seen.get('depth_info') is not None and seen['depth_info'].header.frame_id=='camera_depth_optical_frame' and seen['depth_info'].width==seen['depth'].width
    if motion_result is not None:
        phases=motion_result['phases']
        checks.update({name:motion_result['checks'][name] for name in ['forward','reverse','deadman']})
        checks['independent_motion_observer']=motion_result['passed']
    else:
        for name,lo,hi in [('forward',1.2,2.),('reverse',3.2,4.),('deadman',5.5,6.5)]:
            values=[v for t,v in odometry_samples if lo<=t-start<hi]
            mean=sum(values)/len(values) if values else float('nan')
            phases[name]={'mean_vx_m_s':mean,'samples':len(values)}
            checks[name]=bool(values) and (mean>.03 if name=='forward' else mean<-.03 if name=='reverse' else abs(mean)<.15)
with open(output,'w') as f:json.dump({'ros':version,'checks':checks,'phases':phases,'livox':livox,'passed':all(checks.values())},f,indent=2)
if version=='ros2':node.destroy_node();rclpy.shutdown()
else:rospy.signal_shutdown('complete')
if not all(checks.values()):raise SystemExit(1)
