#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
import sys
import tf
import tf2_ros
import numpy as np

vehicle_type = str(rospy.get_param('/multi_coordinate_transfer/vehicle'))
vehicle_num = int(rospy.get_param('/multi_coordinate_transfer/number'))
multi_pose_pub = [None] * vehicle_num
multi_odom_pub = [None] * vehicle_num
bias = np.array([[0,0,0],[100,100,0], [-100,-100,0],[-100,100,0],[100,-100,0]])

def multi_aloam_tf(i):
  tfs = TransformStamped()
  # --- 头信息
  tfs.header.frame_id = "world"
  tfs.header.stamp = rospy.Time.now()
  tfs.header.seq = 101
  # --- 子坐标系
  tfs.child_frame_id = vehicle_type + "_" + str(i) + "/odom"
  # --- 坐标系相对信息
  # ------ 偏移量
  tfs.transform.translation.x = bias[i][0]
  tfs.transform.translation.y = bias[i][1]
  tfs.transform.translation.z = bias[i][2]
  # ------ 四元数
  qtn = tf.transformations.quaternion_from_euler(0,0,0)
  tfs.transform.rotation.x = qtn[0]
  tfs.transform.rotation.y = qtn[1]
  tfs.transform.rotation.z = qtn[2]
  tfs.transform.rotation.w = qtn[3]
  return tfs
    
if __name__ == '__main__':
  rospy.init_node(vehicle_type+'_coordinate_transfer')
  for i in range(vehicle_num):
    multi_pose_pub[i] = tf2_ros.StaticTransformBroadcaster()
  rate = rospy.Rate(20)
  s = set()
  
  while not rospy.is_shutdown():    
      for i in range(vehicle_num):
          multi_pose_pub[i].sendTransform(multi_aloam_tf(i))
          if i not in s:
            s.update([i])
            print(f"\n================> transform robot {i}'s coordinate successed ! <=================")
      try:
          rate.sleep()
      except:
          continue