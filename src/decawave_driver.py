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
      
      d0 = 0
      d1 = 0
      d2 = 0
      d3 = 0
      while not rospy.is_shutdown():
        # Check to make sure the bitstream is at the beginning of a message
        match = '\x6D' # x6D is the ascii character 'm'
        sync = ser.read()
        if sync != match:
          print "Sync didn't match."
          continue
        else:
          print "Sync matched. Starting Read."
        i = 0
        while i <= 2:
          if i != 0:
            sync = ser.read() # read the m from the beginning of the message
          raw_data = ser.readline()
          if ((i == 0) and (raw_data[0:3] != b'a00')):
              print "Advance until at a00."
              break
          if raw_data[0:3] == b'a00':
            d0 = int(raw_data[8:16], 16) / 1000.00
          elif raw_data[0:3] == b'a01':
            d1 = int(raw_data[8:16], 16) / 1000.00
          elif raw_data[0:3] == b'a02':
            d2 = int(raw_data[8:16], 16) / 1000.00
          elif raw_data[0:3] == b'a03':
            d3 = int(raw_data[8:16], 16) / 1000.00
          i+=1
          
        dwMsg.dist = (d0, d1, d2, d3)
        dwMsg.header.stamp = rospy.get_rostime()
        dwPub.publish(dwMsg)
        rospy.loginfo(dwMsg.dist)
      
      ser.close()
      
    except SerialException:
      print("Could not connect to the serial port")
    
if __name__ == "__main__":
  dw = DecaWaveDriver()
