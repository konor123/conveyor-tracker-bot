import time,json,signal
from pathlib import Path
import cv2,numpy as np,rclpy
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
cv2.setNumThreads(1)
dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
rclpy.init(); n=rclpy.create_node('bench_aruco_reference_load'); rows=[]; live=True
signal.signal(signal.SIGTERM,lambda *_:globals().update(live=False))
def cb(m):
 t=time.monotonic();a=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
 gray=cv2.cvtColor(a,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
 _,ids,_=cv2.aruco.detectMarkers(gray,dictionary)
 rows.append({'t_ros_s':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'t_steady_s':t,'processing_s':time.monotonic()-t,'markers':[] if ids is None else ids.flatten().tolist()})
 if len(rows)%300==0:print('processed_frames',len(rows),flush=True)
n.create_subscription(Image,'/camera/camera/color/image_raw',cb,qos_profile_sensor_data)
t0=time.monotonic()
try:
 while live and time.monotonic()-t0<350:rclpy.spin_once(n,timeout_sec=.1)
finally:
 out={'workload':'1280x800 grayscale + OpenCV DICT_4X4_50 ArUco detection; reference workload, production vision_node unavailable','duration_s':time.monotonic()-t0,'frames':len(rows),'rows':rows}
 Path('/home/umm/conveyor-tracker-bot/records/bench/vision-load-2026-11-07.json').write_text(json.dumps(out))
 print('finished',len(rows),flush=True);n.destroy_node();rclpy.try_shutdown()
