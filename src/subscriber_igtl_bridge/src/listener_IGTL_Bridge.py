#!/usr/bin/env python

import sys
import rospy
import math
import numpy, quaternion

from std_msgs.msg import String, Float64
from ros_igtl_bridge.msg import igtltransform
from tf.transformations import euler_from_quaternion

i=0;
a=numpy.empty([20])



class SubscriberIGTL(object):
    def __init__(self):
        #Initialize ROS PARAMETERS
        self.IGTL_INIT_W                 = rospy.get_param('/igtl_ref_w_s')
        self.IGTL_INIT_X                 = rospy.get_param('/igtl_ref_x_s')
        self.IGTL_INIT_Y                 = rospy.get_param('/igtl_ref_y_s')
        self.IGTL_INIT_Z                 = rospy.get_param('/igtl_ref_z_s')


        igtl_ori_topic_name              = rospy.get_param('igtl_ori_topic_name')
        igtl_publisher                   = rospy.get_param('igtl_publisher_name')

        #Inizitialize Subscribers & Publishers
        self.sub_igtlmsg=rospy.Subscriber(igtl_ori_topic_name,igtltransform ,self.igtl_callback)
        self.pub_eulermsg=rospy.Publisher(igtl_publisher,Float64, queue_size=10)
        

    def fromQuaternionToEulerAngle (self,data):
        ###print "FROM QUATERNION TO EULER ANGLE_ with Trasformation"
        #uso la libreria NUMPY.QUATERNION per effettuare la rotazione tra i 2 sistemi di riferimento

        quaternion_1=(data.x , data.y, data.z, data.w)
        quat=numpy.quaternion(data.w,data.x,data.y,data.z)
        ###print quat
        quat_init=numpy.quaternion(self.IGTL_INIT_W,self.IGTL_INIT_X,self.IGTL_INIT_Y, self.IGTL_INIT_Z)
        #quat_init=numpy.quaternion(0.707,0.707,0.0,0.0)

        qr_=quat*quat_init
        quaternion=(qr_.x, qr_.y, qr_.z, qr_.w) #l'ordine degli assi e diverso
        #quaternion=(quaterion[0],quaternion[1], quaternion[2], quaternion[3])

        euler_1=euler_from_quaternion(quaternion_1)
        ###print "\nPHI: "+str(math.degrees(euler_1[0]))+"\nTHETA: "+str(math.degrees(euler_1[1]))+"\nPSI: "+str(math.degrees(euler_1[2]))+"\n\n"
        euler=euler_from_quaternion(quaternion)
        #print math.degrees(euler)
        return euler


    # FUNZIONE CALLBACK SENZA MEDIA

    def transform_callback(self,data):
        ###print "------------------------------------------------"
        #print str(data.transform.rotation) # PRENDE IL VETTORE DI QUATERNIONI
        euler=self.fromQuaternionToEulerAngle(data.transform.rotation)
        ###print "\nPHI: "+str(math.degrees(euler[0]))+"\nTHETA: "+str(math.degrees(euler[1]))+"\nPSI: "+str(math.degrees(euler[2]))+"\n\n"

        #pub_eulermsg=rospy.Publisher('state',Float64, queue_size=10)
        rate=rospy.Rate(400) #dipende dalla frequenza dell'EM Sensor
        self.pub_eulermsg.publish(math.degrees(euler[0])) #pubblico l'angolo intorno all'asse X
        #va aggiunto il publisher per l'angolo intorno all'asse Y
        
        rate.sleep();

    #def transform_callback(data):
    #    global i
    #    print "------------------------------------------------"
    #    #print str(data.transform.rotation) # PRENDE IL VETTORE DI QUATERNIONI
    #    euler=fromQuaternionToEulerAngle(data.transform.rotation)
    #    print "\nPHI: "+str(math.degrees(euler[0]))+"\nTHETA: "+str(math.degrees(euler[1]))+"\nPSI: "+str(math.degrees(euler[2]))+"\n\n"
    #    pub_eulermsg=rospy.Publisher('state',Float64, queue_size=10)
    #    rate=rospy.Rate(400) #dipende dalla frequenza dell'EM Sensor
    #    print i
    #
    #    a[i]=math.degrees(euler[0])
    #
    #
    #    if i==19:
    #        euler_av=movingaverage(a,20)
    #
    #        pub_eulermsg.publish(euler_av[0]) #pubblico l'angolo intorno all'asse X
    #        #va aggiunto il publisher per l'angolo intorno all'asse Y
    #        print "\n PHI: "+str(euler_av)+"\n"
    #        i=0
    #    else:
    #        i=i+1
    #
    #    rate.sleep();

    def igtl_callback(self,data):
        ###rospy.loginfo(data)
        self.transform_callback(data)

    #def listener_igtl():
    #
    #    sub_igtlmsg=rospy.Subscriber('IGTL_TRANSFORM_IN',igtltransform , igtl_callback)
    #    #sub_odom=rospy.Subscriber('/sensor_sensor_1_rpy', Odometry, odom_callback)
    #
    #    rospy.init_node('listener_igtl', anonymous=True)
    #
    #    #rospy.init_node('pub_euler',anonymous=True)
    #    rospy.spin()




def main (args):

    rospy.init_node('listener_igtl', anonymous=True)
    sub_igtl=SubscriberIGTL()

    rospy.spin()
    #rospy.on_shutdown(sub_igtl.shutdown_listener)

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
