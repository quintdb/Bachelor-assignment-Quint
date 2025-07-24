#!/usr/bin/env python

import sys
import rospy
import math
import numpy, quaternion
import time

from std_msgs.msg import String, Bool, Float64
from sensor_msgs.msg import Imu

from tf.transformations import euler_from_quaternion

#i=0;
#a=numpy.empty([20])
#control_is_enabled= False   #
#bool_first_quat = False     #save_current_first_ori
#bool_last_quat= False       #save_last_ori
#last_quaternion=numpy.quaternion(1,0,0,0)
#first_quat=numpy.quaternion(1,0,0,0);
#phi_round_last=0
#cd_gain= 1







class SubscriberAwinda(object):
    def __init__(self):
        #Inizitialize ROS PARAMETERS

        self.IMU_INIT_W                 = rospy.get_param('/imu_ref_w')
        self.IMU_INIT_X                 = rospy.get_param('/imu_ref_x')
        self.IMU_INIT_Y                 = rospy.get_param('/imu_ref_y')
        self.IMU_INIT_Z                 = rospy.get_param('/imu_ref_z')

        imu_ori_topic                   = rospy.get_param('/imu_ori_topic_name')
        footswitch_topic                = rospy.get_param('/footswitch_topic_name')
        use_footswitch                  = rospy.get_param('/use_footswitch', True) # set default to use the footswitch
        imu_publisher_topic             = rospy.get_param('/imu_publisher_name')

        #Initialize Subscribers & Publishers
        imu_sub=rospy.Subscriber(imu_ori_topic, Imu, self.imu_callback)

        if use_footswitch:
            print("NO")
            footswitch_sub = rospy.Subscriber(footswitch_topic, Bool, self.footswitch_callback)
            self.control_is_enabled = False   #METTI FALSO
            self.bool_last_quat = False
            self.bool_first_quat = False
            self.last_quaternion = numpy.quaternion(1,0,0,0)
            self.first_quat = numpy.quaternion(1,0,0,0)
            self.phi_round_last = 0
        else:
            self.control_is_enabled = True
            print("YES")

         # Initialize variables
        self.cd_gain = 1
        self.phi_round_last = 0

        self.euler_1=numpy.zeros(2) #global name
        self.euler_rid=[0,0,0]
        self.delta_e=2 #Variazione di grado ( inserire nel param)

    def reduceVelocity(self,data):
        self.euler_rid=data
        self.euler_1[1]=self.euler_1[0]
        self.euler_1[0]=math.degrees(data[0])
        #print self.euler_rid[0]

        if (self.euler_1[0]-self.euler_1[1]>self.delta_e):
            ###print ("CASE POSITIVE")
            self.euler_rid=list(self.euler_rid)
            self.euler_rid[0]= math.radians(self.euler_1[1]+self.delta_e)
            self.euler_1[0]=math.degrees(self.euler_rid[0])
            self.euler_rid=tuple(self.euler_rid)

        elif(self.euler_1[1]-self.euler_1[0]>self.delta_e):
            ###print ("CASE NEGATIVE")
            self.euler_rid=list(self.euler_rid)
            self.euler_rid[0]= math.radians(self.euler_1[1]-self.delta_e)
            self.euler_1[0]=math.degrees(self.euler_rid[0])
            self.euler_rid=tuple(self.euler_rid)

        ###else:
            ###print ("CASE NEUTRAL")
        #print self.euler_1[0]
        #data[0]=math.radians(self.euler_1[0])
        #data[0]=self.euler_1[0]
        #data=(math.radians(self.euler_1[0]),)+math.radians(self.euler_1[1:])

        return self.euler_rid

    def fromQuaternionToEulerAngle (self,data):

        #global bool_first_quat, bool_last_quat, last_quaternion, cd_gain, first_quat

        ###print ("FROM QUATERNION TO EULER ANGLE_ with Trasformation")
        #uso la libreria NUMPY.QUATERNION per effettuare la rotazione tra i 2 sistemi di riferimento
        #print data
        quat=numpy.quaternion(data.orientation.w , data.orientation.x, data.orientation.y, data.orientation.z)
        #print quat
        quat_init=numpy.quaternion(self.IMU_INIT_W,self.IMU_INIT_X,self.IMU_INIT_Y,self.IMU_INIT_Z)
        #quat_init=numpy.quaternion(1,0,0,0)
        qr_=quat*quat_init

        #print qr_
        #print "____________________________CONTROLL:"+str(control_is_enabled)

        if (self.control_is_enabled==False and self.bool_last_quat):

            qr_=self.first_quat.conjugate() * qr_
            qr_= qr_**numpy.floor(self.cd_gain) * quaternion.slerp_evaluate(numpy.quaternion(1,0,0,0),qr_,self.cd_gain-numpy.floor(self.cd_gain))
            self.last_quaternion=self.last_quaternion*qr_
            self.bool_last_quat=False
            print last_quaternion

        if self.control_is_enabled:
            #print ("control_is_enabled==TRUE")

            if self.bool_first_quat:
                #print "first quaterion"
                self.first_quat=qr_
                self.bool_first_quat= False
        #    last_quaternion=qr_

        #else:
        #    qr_=last_quaternion

        qr_=self.first_quat.conjugate()*qr_#Difference between current ori and first orientation
        qr_=qr_**numpy.floor(self.cd_gain) * quaternion.slerp_evaluate(numpy.quaternion(1,0,0,0),qr_,self.cd_gain-numpy.floor(self.cd_gain))
        qr_=self.last_quaternion*qr_
        quaternion_fin=(qr_.x, qr_.y, qr_.z, qr_.w) #l'ordine degli assi e diverso
        #quaternion=(quaterion[0],quaternion[1], quaternion[2], quaternion[3])



        #euler_1=euler_from_quaternion(quaternion_1)
        #print "\nPHI: "+str(math.degrees(euler_1[0]))+"\nTHETA: "+str(math.degrees(euler_1[1]))+"\nPSI: "+str(math.degrees(euler_1[2]))+"\n\n"
        euler=euler_from_quaternion(quaternion_fin)
        #print math.degrees(euler)
        return euler



    def imu_callback(self,data):

        #global control_is_enabled,phi_round_last
        #time.sleep(1)
        ###print ("------------------------------------------------")
        #print str(data.transform.rotation) # PRENDE IL VETTORE DI QUATERNIONI
        euler=self.fromQuaternionToEulerAngle(data)
        #FUNZIONE RIDUZIONE VELOCITA
        ###print ("\nPHI: "+str(math.degrees(euler[0]))+"\nTHETA: "+str(math.degrees(euler[1]))+"\nPSI: "+str(math.degrees(euler[2]))+"\n\n")

        pub_eulermsg=rospy.Publisher('/setpoint',Float64, queue_size=10)  #I CAN NOT USE imu_publisher_name
        rate=rospy.Rate(40) #impostato a 40 MA dipende dall'impostazione del Imu



        #print "____________________________CONTROLL:"+str(control_is_enabled)

        if self.control_is_enabled:

            #print('control is enabled')
            euler=self.reduceVelocity(euler)
            phi=math.degrees(euler[0])
            theta=math.degrees(euler[1])
            psi=math.degrees(euler[2])
            phi_round=round(phi)

            pub_eulermsg.publish(phi_round) #pubblico l'angolo intorno all'asse X
            self.phi_round_last=phi_round

            #va aggiunto il publisher per l'angolo intorno all'asse Y
        else:
            pub_eulermsg.publish(self.phi_round_last) #pubblico l'angolo intorno all'asse X


        rate.sleep()

    def footswitch_callback(self,data):

        #global control_is_enabled, bool_last_quat,bool_first_quat

        self.control_is_enabled = data.data
        if (self.control_is_enabled == True):
            self.bool_last_quat = True
            #print "BLAST_QUAT == TRUE"

        else:
            self.bool_first_quat=True
            #print "BFIRST_QUAT == TRUE"

        print (data.data)


#   def listener_igtl():
#       imu_sub=rospy.Subscriber('/sensor_sensor_1',Imu , imu_callback)
#       footswitch_sub = rospy.Subscriber('/footswitch/control_is_enabled', Bool,footswitch_callback)
#       #imu_sub_rpy=rospy.Subscriber('/sensor_sensor_1_rpy', vector, imu_callback)  #conviene usare i Quaternioni
#


def main(args):

    rospy.init_node('listener_igtl', anonymous=True)
    subA=SubscriberAwinda()

    rospy.spin()
    #rospy.on_shutdown(subA.shutdown)  #Da implementare

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
