"""Noetic E2E fixture: route requests on stdin, real sensor topics observed."""
import collections,json,math,sys,threading
from pathlib import Path
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,Image
from livox_ros_driver2.msg import CustomMsg
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

output=Path(sys.argv[1]);lock=threading.Lock();streams={};transforms={};lidar_valid=True;imu_valid=True
rospy.init_node('sprobo_tutorial_io',anonymous=True)
publisher=rospy.Publisher('/go2w/cmd_vel',Twist,queue_size=1)

def received(name):
 def callback(msg):
  global lidar_valid,imu_valid
  stamp=msg.clock.to_nsec() if name=='clock' else msg.header.stamp.to_nsec()
  with lock:
   entry=streams.setdefault(name,{'count':0,'first':stamp,'last':stamp,'intervals':collections.Counter(),'monotonic':True})
   if entry['count']:
    delta=stamp-entry['last'];entry['intervals'][delta]+=1;entry['monotonic'] &= delta>0
   entry['count']+=1;entry['last']=stamp
   if name in ['rgb','depth']:entry['shape']=[msg.width,msg.height];entry['encoding']=msg.encoding
   if name=='lidar':lidar_valid &= msg.point_num==len(msg.points)>0 and msg.header.frame_id=='livox_frame' and all(0<=p.offset_time<100000000 for p in msg.points)
   if name in ['imu','camera_imu']:imu_valid &= all(math.isfinite(getattr(v,k)) for v in [msg.linear_acceleration,msg.angular_velocity] for k in ['x','y','z'])
 return callback

def tf_callback(msg):
 with lock:
  for t in msg.transforms:
   v=t.transform.translation;q=t.transform.rotation
   transforms[t.child_frame_id]={'parent':t.header.frame_id,'position':[v.x,v.y,v.z],'quaternion':[q.w,q.x,q.y,q.z]}

subscriptions=[rospy.Subscriber(topic,cls,received(name),queue_size=10,buff_size=2**23) for cls,topic,name in [
 (Clock,'/clock','clock'),(Imu,'/livox/imu_si','imu'),(Imu,'/camera/imu','camera_imu'),
 (CustomMsg,'/livox/lidar','lidar'),(Image,'/camera/image_raw','rgb'),(Image,'/camera/depth/image_raw','depth')]]
subscriptions.append(rospy.Subscriber('/tf_static',TFMessage,tf_callback,queue_size=10))

def flush(_=None):
 with lock:
  data={k:dict(v,intervals={str(a):b for a,b in v['intervals'].items()}) for k,v in streams.items()}
  tf=dict(transforms);checks={'custom_msg_fields':lidar_valid,'imu_finite':imu_valid}
  for name,dt in [('imu',5000000),('camera_imu',5000000),('lidar',100000000),('rgb',100000000),('depth',100000000)]:
   v=streams.get(name);checks[name+'_rate']=bool(v and v['count']>20 and v['monotonic'] and v['intervals'] and v['intervals'].most_common(1)[0][0]==dt)
  checks['clock']=streams.get('clock',{}).get('count',0)>100
  checks['rigid_frames_present']=all(k in tf for k in ['livox_frame','imu_frame'])
 report={'checks':checks,'streams':data,'transforms':tf}
 tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(report,indent=2)+'\n');tmp.replace(output)

timer=rospy.Timer(rospy.Duration(1),flush,reset=True)
try:
 for line in sys.stdin:
  vx,vy,wz=json.loads(line);m=Twist();m.linear.x=vx;m.linear.y=vy;m.angular.z=wz;publisher.publish(m)
finally:
 timer.shutdown();flush();rospy.signal_shutdown('fixture complete')
