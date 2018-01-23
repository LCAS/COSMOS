#!/usr/bin/env python

# license removed for brevity

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
import serial
import time



def talker():

    pub = rospy.Publisher('/penetrometer/force', Float32, queue_size=10)

    print "opening serial port"
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0, parity=serial.PARITY_NONE)
    ser.write('\r')
    rospy.sleep(0.1)
    
    bytesToRead = ser.inWaiting()
    sdata = ser.read(bytesToRead)

    print sdata
    print "reseting force sensor"

    ser.write('CT0\r')
    rospy.sleep(0.1)

    bytesToRead = ser.inWaiting()
    sdata = ser.read(bytesToRead)

    print sdata
    print "reseting force sensor"


    ser.write('O0W0\r')
    rate = rospy.Rate(10) # 10hz
    rospy.sleep(0.01)
    while not rospy.is_shutdown():
        #hello_str = ser.readline()
        bytesToRead = ser.inWaiting()
        sdata = ser.read(bytesToRead)
        datao = sdata.split('\r\n')
#        print sdata, bytesToRead
        if len(datao)>2:
#            print '--' 
#            print datao[1]
#            print '::'
            rospy.sleep(0.05)
    #        rospy.loginfo(hello_str)
            val = (float(datao[1])*0.453592)/1000.0
            pub.publish(val)
        else:
            print "WARNING"
            print sdata
        rate.sleep()
    
    ser.write('\n')
    rospy.sleep(0.1)
    bytesToRead = ser.inWaiting()
    sdata = ser.read(bytesToRead)   
    #ser.flush()
    ser.flushInput()
    ser.flushOutput()
    rospy.sleep(0.1)
    ser.close()
    print "All done"

if __name__ == '__main__':
    rospy.init_node('force_sensor_interface')   
    talker()
