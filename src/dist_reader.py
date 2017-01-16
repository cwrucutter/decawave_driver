#!/usr/bin/env python
#
# The MIT License (MIT)
#
# Copyright (c) 2016 Matthew Klein
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy
from snowmower_msgs.msg import DecaWaveMsg


import serial, time, os
from serial import SerialException
from decimal import Decimal
import numpy as np

class DecaWaveDriver:
  def __init__(self):
    rospy.init_node('decawave_driver')
    #Init DecaWave port
    dwPort = rospy.get_param('~port','/dev/ttyUSB0')
    dwRate = rospy.get_param('~baud',9600)
    dwID   = rospy.get_param('~id','t0')
    dwPub = rospy.Publisher('dw/' + dwID + '/data', DecaWaveMsg, queue_size = 5)
    anchor_angle = rospy.get_param('~anchor_angle',0)
    tag_angle = rospy.get_param('~tag_angle',0)
    beacon_distance = rospy.get_param('~beacon_distance',0)
    
    try:
      ser = serial.Serial(
      port = dwPort,
      timeout = 10,
      baudrate = dwRate
      )
      
      dwMsg = DecaWaveMsg()
      dwMsg.header.frame_id = "base_decawave_" + dwID
      
      ser.close()
      ser.open()

      # initialize range values in case a measurement is dropped the first time
      range0 = 0

      # create a file location to store data
      filename = '/home/snowmower/d' + str(Decimal(beacon_distance).quantize(Decimal('1')))+ 'm_t' + str(Decimal(tag_angle).quantize(Decimal('1'))) + 'd_a' + str(Decimal(anchor_angle).quantize(Decimal('1'))) + 'd.csv'
      # erase all the data in that file
      f = open(filename, 'w').close()
      # then open it again to append to it.
      f = open(filename, 'a')

      # Create an empty numpy array to store data (for mean and std)
      data_array = np.array([])

      # Take 100 good data points and store them in the file
      i = 0
      while (i<300):

        raw_data = ser.readline()
        if raw_data == serial.to_bytes([]):
          print "serial timeout"
        else:
          data = raw_data.split()

          if data[0] == 'mc':
            #print "read data as a list:"
            #print data
            mask = int(data[1],16)
            if (mask & 0x01):
              #print "range0 good"
              range0 = int(data[2],16)/1000.0
              print range0
              # write to file
              f.write(str(range0)+'\n')
              # add data to the array
              data_array = np.append(data_array,range0)
              # increase counter for while loop
              i = i + 1
            else:
              print "range0 bad"
      
      ser.close()
      f.flush()
      f.close()
      print('mean = ' + str(np.mean(data_array)))
      print('std  = ' + str(np.std(data_array)))
      
    except SerialException:
      print("Could not connect to the serial port")
    
if __name__ == "__main__":
  dw = DecaWaveDriver()
