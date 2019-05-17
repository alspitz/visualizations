import mmap
import struct

import numpy as np

class Data:
  pass

class DataSer:
  data_fmt = 15 * 'f' + 4 * 'i' + 4 * 'f'

  def __init__(self):
    self.shared_file = mmap.mmap(-1, 200)

  def write(self, data):
    self.shared_file.seek(0)
    self.shared_file.write(struct.pack(self.data_fmt,
      data.pos[0],
      data.pos[1],
      data.pos[2],
      data.vel[0],
      data.vel[1],
      data.vel[2],
      data.accel_dist[0],
      data.accel_dist[1],
      data.accel_dist[2],
      data.torque_dist[0],
      data.torque_dist[1],
      data.torque_dist[2],
      data.euler[0],
      data.euler[1],
      data.euler[2],
      data.rpms[0],
      data.rpms[1],
      data.rpms[2],
      data.rpms[3],
      data.voltage,
      data.current,
      data.temp,
      data.yawerr
    ))

  def read(self):
    self.shared_file.seek(0)
    data = self.shared_file.read(struct.calcsize(self.data_fmt))
    floats = struct.unpack(self.data_fmt, data)

    ret_data = Data()

    ret_data.pos = floats[0:3]
    ret_data.vel = floats[3:6]
    ret_data.accel_dist = floats[6:9]
    ret_data.torque_dist = floats[9:12]
    ret_data.euler = floats[12:15]
    ret_data.rpms = floats[15:19]
    ret_data.voltage = floats[19:20]
    ret_data.current = floats[20:21]
    ret_data.temp = floats[21:22]
    ret_data.yawerr = floats[22:23]

    return ret_data
