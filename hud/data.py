import numpy as np

import rospy

from multirotor_control.msg import (
  AccelerationDisturbanceObserverLuenbergerDebug,
  TorqueDisturbanceObserverLuenbergerDebug
)
from quadrotor_msgs.msg import BatteryStatus, RPMCommand, TrackingError
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Temperature

from tf.transformations import euler_from_quaternion

class DataSource:
  def __init__(self, data_ser):
    self.data_ser = data_ser

    rospy.init_node("telemetry")

    self.pos = np.zeros(3)
    self.vel = np.zeros(3)
    self.accel_dist = np.zeros(3)
    self.torque_dist = np.zeros(3)
    self.euler = np.zeros(3)
    self.rpms = np.zeros(4)
    self.voltage = 0
    self.current = 0
    self.temp = 0
    self.yawerr = 0

    rospy.Subscriber("danaus06/vicon_odom", Odometry, self.odom_callback, tcp_nodelay=True)
    rospy.Subscriber("danaus06/motion_manager/tracking_error", TrackingError, self.te_callback, tcp_nodelay=True)
    rospy.Subscriber("danaus06/motion_manager/position_controller/l1ac_debug", AccelerationDisturbanceObserverLuenbergerDebug, self.accel_dist_callback, tcp_nodelay=True)
    rospy.Subscriber("danaus06/motion_manager/attitude_controller/l1ac_debug", TorqueDisturbanceObserverLuenbergerDebug, self.torque_dist_callback, tcp_nodelay=True)
    rospy.Subscriber("danaus06/px4/rpm_cmd", RPMCommand, self.rpm_callback, tcp_nodelay=True)
    rospy.Subscriber("danaus06/px4/voltage", BatteryStatus, self.bat_callback, tcp_nodelay=True)
    rospy.Subscriber("danaus06/px4/temperature", Temperature, self.temp_callback, tcp_nodelay=True)

    print("ROS Node started.")
    rospy.spin()

  def write(self):
    self.data_ser.write(self)

  def odom_callback(self, msg):
    quat = np.array((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
    euler_rad = euler_from_quaternion(quat)
    self.euler = np.degrees(euler_rad)
    self.write()

  def te_callback(self, msg):
    self.pos = np.array((msg.pos_err.x, msg.pos_err.y, msg.pos_err.z))
    self.vel = np.array((msg.vel_err.x, msg.vel_err.y, msg.vel_err.z))
    self.yawerr = msg.yaw_err
    self.write()

  def accel_dist_callback(self, msg):
    self.accel_dist = np.array((msg.lpd.x, msg.lpd.y, msg.lpd.z))
    self.write()

  def torque_dist_callback(self, msg):
    self.torque_dist = np.array((msg.lpd.x, msg.lpd.y, msg.lpd.z))
    self.write()

  def rpm_callback(self, msg):
    self.rpms = np.array((msg.motor_rpm[0], msg.motor_rpm[1], msg.motor_rpm[2], msg.motor_rpm[3]))
    self.write()

  def bat_callback(self, msg):
    self.voltage = msg.voltage
    self.current = msg.current
    self.write()

  def temp_callback(self, msg):
    self.temp = msg.temperature
    self.write()
