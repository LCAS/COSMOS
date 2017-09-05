#!/usr/bin/env python

# license removed for brevity

#import rospy
#from std_msgs.msg import String
import serial
import time



def talker():
    ser = serial.Serial('/dev/tnt0', 115200, timeout=0, parity=serial.PARITY_EVEN, rtscts=1)
#    pub = rospy.Publisher('/cosmos_reading', String, queue_size=10)
#    rospy.init_node('talker', anonymous=True)
#    rate = rospy.Rate(10) # 10hz
    f = open('a.log', 'r')
    
    for line in f:
        print line
        ser.write(line)
        time.sleep(0.1)

        
    #ser.close()
    

if __name__ == '__main__':
    talker()
