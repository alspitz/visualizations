#!/usr/bin/env python

import math
import time

import rospy

from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

from tf.transformations import quaternion_from_euler, quaternion_multiply

import tf2_ros

def main():
  rospy.init_node('vis_vehicle')

  # Parameters:
  speed = 1.5
  angle_limit = math.pi / 6
  framerate = 60.0

  scale = 20
  mesh_name = "vehicle_configs/mesh/acerodon.ply"
  mesh_scale = 1.0
  roll_offset = 0.0

  #scale = 20
  #mesh_name = "lumenier_danaus_mesh/mesh/LumenierDanaus.mesh"
  #mesh_scale = 1.0 / 14
  #roll_offset = math.pi

  pitch_offset = 0.25
  yaw_offset = 0

  y_positions = [-scale * 1.0, 0, scale * 1.0]

  ts = [TransformStamped() for _ in range(3)]

  markers = []
  pubs = []

  for i, t in enumerate(ts):
    marker = Marker()
    marker.header.stamp = rospy.Time()
    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = scale * mesh_scale
    marker.scale.y = scale * mesh_scale
    marker.scale.z = scale * mesh_scale
    marker.color.a = 1.0
    marker.color.r = 0.8
    marker.color.g = 0.8
    marker.color.b = 0.8
    marker.frame_locked = True
    marker.mesh_resource = "package://" + mesh_name

    marker.header.frame_id = "acerodon" + str(i + 1)
    pub = rospy.Publisher('acerodon_mesh' + str(i + 1), Marker, queue_size=1, latch=True)
    pub.publish(marker)

    pubs.append(pub)
    markers.append(marker)

    t.header.frame_id = "world"
    t.child_frame_id = "acerodon" + str(i + 1)
    t.transform.translation.y = y_positions[i]

  quat_y = quaternion_from_euler(0, pitch_offset, 0)
  quat_off = quaternion_from_euler(0, math.pi / 2, math.pi)
  quat_const = quaternion_multiply(quat_y, quat_off)

  t = 0
  inc = speed  / framerate

  br = tf2_ros.TransformBroadcaster()

  while not rospy.is_shutdown():
    time_now = time.time()

    for i, tr in enumerate(ts):
      tr.header.stamp = rospy.Time.from_sec(time_now)

      angle_to_use = angle_limit * math.sin(t)

      angles = [0, 0, yaw_offset]
      static_angles = angles[:]

      angles[i] += angle_to_use
      quat_dyn = quaternion_from_euler(*angles)
      quat = quaternion_multiply(quat_const, quat_dyn)

      tr.transform.rotation.x = quat[0]
      tr.transform.rotation.y = quat[1]
      tr.transform.rotation.z = quat[2]
      tr.transform.rotation.w = quat[3]

      if i < 1:
        tr.transform.translation.x = 0
        tr.transform.translation.y = -scale - 8.4 * angle_to_use
        tr.transform.translation.z = 0
      elif i == 1:
        markers[1].scale.x = (1 + 0.36 * angle_to_use) * scale * mesh_scale
        markers[1].scale.y = (1 + 0.36 * angle_to_use) * scale * mesh_scale
        markers[1].scale.z = (1 + 0.36 * angle_to_use) * scale * mesh_scale
        pubs[1].publish(markers[1])

      br.sendTransform(tr)

    time.sleep(1 / framerate)
    t += inc

if __name__ == "__main__":
  main()
