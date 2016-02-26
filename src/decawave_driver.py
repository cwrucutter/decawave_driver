#!/usr/bin/env python
#
# The MIT License (MIT)
#
# Copyright (c) 2015 Kevin Griesser
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

class DecaWaveDriver:
  def __init__(self):
    rospy.init_node('decawave_driver')
    #Init DecaWave port
    dwPort = rospy.get_param('~port','/dev/ttyUSB0')
    dwRate = rospy.get_param('~baud',9600)
    dwID   = rospy.get_param('~id','t0')
    dwPub = rospy.Publisher('dw/' + dwID + '/data', DecaWaveMsg, queue_size = 5)
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
      range1 = 0
      range2 = 0
      range3 = 0

      while not rospy.is_shutdown():

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
            else:
              print "range0 bad"
              # range0 = -1
            if (mask & 0x02):
              #print "range1 good"
              range1 = int(data[3],16)/1000.0
            else:
              print "range1 bad"
              # range1 = -1
            if (mask & 0x04):
              #print "range2 good"
              range2 = int(data[4],16)/1000.0
            else:
              print "range2 bad"
              # range2 = -1
            if (mask & 0x08):
              #print "range3 good"
              range3 = int(data[5],16)/1000.0
            else:
              print "range3 bad"
              # range3 = -1

            dwMsg.dist = (range0, range1, range2, range3)
            dwMsg.header.stamp = rospy.get_rostime()
            dwPub.publish(dwMsg)
            rospy.loginfo(dwMsg.dist) 
      
      ser.close()
      
    except SerialException:
      print("Could not connect to the serial port")
    
if __name__ == "__main__":
  dw = DecaWaveDriver()
