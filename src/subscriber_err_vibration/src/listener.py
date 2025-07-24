#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Point

goal_reach                  = rospy.get_param('~/goal_reach', False)

cmd = Point()


def err_cb(data):

    goal_reach=False

    if(data.data>1):
        freq=100
    elif (data.data<-1):
        freq=25;
    else:
        freq=0;
        goal_reach=True;


    print (abs(data.data))

    if(abs(data.data)>5):
        ampl=145
    elif(abs(data.data)<1):
        ampl=0
    else:
        ampl=(145/5)*abs(data.data)

    print "----------------------TRANSF_CB--------------------------"

    cmd.x=freq
    cmd.y=ampl


def main():
    global cmd

    rospy.init_node('from_err_to_vibCmd', anonymous=True)

    rate = rospy.Rate(100) # 40hz

    err_sub                   = rospy.Subscriber('/error_angle', Float64, err_cb)
    vib_cmd_pub               = rospy.Publisher("/cmd", Point, queue_size=10)
    while not rospy.is_shutdown():
        #print("------------------ MARKERs PUBLISHER -------------------")
        vib_cmd_pub.publish(cmd)
        print "-------------------------- "+str(cmd)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
