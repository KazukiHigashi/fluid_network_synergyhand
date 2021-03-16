#!/usr/bin/env python
## coding: UTF-8

import rospy
from std_msgs.msg import String
from datetime import datetime

rospy.init_node('talker')
pub = rospy.Publisher('chatter', String, queue_size=10)
rate = rospy.Rate(10)
print("conection saterted..")
while not rospy.is_shutdown():
  hello_str = String()
  timestamp = rospy.get_time()
  time = datetime.fromtimestamp(timestamp)
  hello_str.data = "hello world(%s)" % time
  pub.publish(hello_str)
  rate.sleep()
 
