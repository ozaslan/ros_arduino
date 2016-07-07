#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int16MultiArray

def publisher():
  pub = rospy.Publisher('arduino_trigger/data', Int16MultiArray, queue_size=10)
  rospy.init_node('arduino_trigger', anonymous=True)
  rate = rospy.Rate(10) # 1hz
  i = 0;
  while not rospy.is_shutdown():
    trig_msg = Int16MultiArray();
    trig_msg.data = [None] * 6;
    trig_msg.data[0] = i;     # cam-led id
    trig_msg.data[1] = 32;    # period
    trig_msg.data[2] = 8 * i; # cam delay
    trig_msg.data[3] = 8;     # cam exposure
    trig_msg.data[4] = 8 * i; # led delay
    trig_msg.data[5] = 8;     # led exposure
    pub.publish(trig_msg);
    i = (i + 1) % 4;
    rate.sleep();

if __name__ == '__main__':
  try:
    publisher();
  except rospy.ROSInterruptException:
    pass
