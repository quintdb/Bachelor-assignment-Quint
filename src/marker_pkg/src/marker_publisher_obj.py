#!/usr/bin/env python
import sys
import tf
import geometry_msgs.msg

import roslib, rospy, numpy, quaternion, math, time
from std_msgs.msg import Float64
from nav_msgs.msg import Path

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from ros_igtl_bridge.msg import igtltransform
from visualization_msgs.msg import Marker , MarkerArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion, PoseWithCovarianceStamped





#Non utilizzato
offset_x=0.0
offset_y=0.0
offset_z=0.0
offset= Vector3(0.0,0.0,0.0)



class MarkerPublisher(object):
    def __init__(self):

        self.q_state = numpy.quaternion(1.0,0.0,0.0,0.0).normalized()
        self.qr_ = numpy.quaternion(1.0,0.0,0.0,0.0).normalized()
        self.qr_2=numpy.quaternion(1,0,0.0,0.0).normalized()

        self.final_goal=0
        self.t=0 #POSSO TOGLIERLO?

        self.IGTL_INIT_W                 = rospy.get_param('/igtl_ref_w', 0.707)
        self.IGTL_INIT_X                 = rospy.get_param('/igtl_ref_x', 0.0)
        self.IGTL_INIT_Y                 = rospy.get_param('/igtl_ref_y', 0.707)
        self.IGTL_INIT_Z                 = rospy.get_param('/igtl_ref_z', 0.0)

        self.resolution                  = rospy.get_param('~/resolution', 0.01)

        self.increase_path_x             = rospy.get_param('/increase_path_x', 0.0)
        self.increase_path_y             = rospy.get_param('/increase_path_y', 0.0)
        self.increase_path_z             = rospy.get_param('/increase_path_z', 0.0)

        self.bool_first                  = rospy.get_param('~/bool_first', True)
        self.posr_                       = rospy.get_param('~/posr_', [0.0,0.0,0.0])

        self.euler                       = rospy.get_param('~/euler', Vector3(0.0,0.0,0.0))
        self.orientation_bottom          = rospy.get_param('~/orientation_bottom', Quaternion(1.0,0.0,0.0,0.0))
        self.qua_tip                     = rospy.get_param('~/qua_tip', Quaternion(1.0,0.0,0.0,0.0))

        self.euler_bottom                = rospy.get_param('~/euler_bottom', Vector3(0.0,0.0,0.0))

        self.deltam                      = rospy.get_param('~/deltam', 200)
        self.h                           = rospy.get_param('~/h', 0.55)


        self.prev_a                      = rospy.get_param('~/prev_a', 0.0)
        self.prev_pos                    = rospy.get_param('~/prev_pos', 50)

        self.hk =self.h/self.deltam

        self.pose_0 = PoseStamped()
        self.min_pose = PoseStamped()
        self.pose = PoseStamped()
        self.current_pose = PoseStamped()


        self.pose_init = PoseWithCovarianceStamped()



        #Inizitialize Subscribers & Publishers
        self.enable_marker_vis       = rospy.get_param('/publish_marker_vis', True)
        #marker_vis_pub = rospy.Publisher('/marker', Marker, queue_size=10)
        self.marker_vis_pub          = rospy.Publisher('/markers', MarkerArray, queue_size=10)
        self.pose_pub                = rospy.Publisher('/end_effector/pose', PoseStamped, queue_size=10)
        self.sub_igtlmsg             = rospy.Subscriber('/IGTL_TRANSFORM_IN',igtltransform ,self.igtl_callback)
        self.path_sub                = rospy.Subscriber('/plan', Path, self.path_callback)

        self.run_pub                = rospy.Publisher('/pose_path', PoseStamped, queue_size=10)
        self.run_sub                = rospy.Subscriber('/pose_path', PoseStamped, self.run_callback)

        self.sub                     = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_pose)

        self.path_pose_pub           = rospy.Publisher('/path_pose', PoseStamped, queue_size=10)
        self.min_pose_pub            = rospy.Publisher('/min_path_pose', PoseStamped, queue_size=10)
        self.pose_0_pub              = rospy.Publisher('/end_effector/pose_0', PoseStamped, queue_size=10)
        self.current_pose_pub         = rospy.Publisher('/end_effector/current_pose', PoseStamped, queue_size=10)

        self.error_pub               = rospy.Publisher("/error_angle", Float64, queue_size=10)
	self.goal_angle_pub	     = rospy.Publisher("/goal_angle", Float64, queue_size=10)

        br = tf.TransformBroadcaster()

    def init_pose(self, pose):
        """ Reset robot pose. """

        self.pose_init = pose

        quaternion_bot=self.pose_init.pose.pose.orientation
        quat_bot=numpy.quaternion(quaternion_bot.w,quaternion_bot.x,quaternion_bot.y,quaternion_bot.z).normalized()
        quaternion = quaternion_from_euler(0.0,0.0,math.radians(270.0))
        qua_0_init=numpy.quaternion(quaternion[3],quaternion[0],quaternion[1],quaternion[2]).normalized()
        self.orientation_bottom=qua_0_init*quat_bot

        self.increase_path_x=self.pose_init.pose.pose.position.x
        self.increase_path_y=self.pose_init.pose.pose.position.y
        self.increase_path_z=self.pose_init.pose.pose.position.z

        self.publish_marker(self.marker_vis_pub)
        #print self.orientation_bottom
        self.publish_tf(self.pose_init.pose.pose.position,self.orientation_bottom,"world","bottom")

    def init_path_pose(self, pose):
        """ Reset robot pose. """

        self.pose_init = pose

        quaternion_bot=self.pose_init.pose.orientation
        quat_bot=numpy.quaternion(quaternion_bot.w,quaternion_bot.x,quaternion_bot.y,quaternion_bot.z).normalized()
        quaternion = quaternion_from_euler(0.0,0.0,math.radians(270.0))
        qua_0_init=numpy.quaternion(quaternion[3],quaternion[0],quaternion[1],quaternion[2]).normalized()
        self.orientation_bottom=qua_0_init*quat_bot

        self.increase_path_x=self.pose_init.pose.position.x
        self.increase_path_y=self.pose_init.pose.position.y
        self.increase_path_z=self.pose_init.pose.position.z

        self.publish_marker(self.marker_vis_pub)
        #print self.orientation_bottom
        self.publish_tf(self.pose_init.pose.position,self.orientation_bottom,"world","bottom")

    def path_callback(self,data):



        #global t,resolution,path_pose_pub, min_pose_pub
        #global orientation_bottom
        #global increase_path_x,increase_path_y,increase_path_z

        #global min_pose,path
        #global error_pub
        #if self.enable_marker_vis:
        #    rate=rospy.Rate(400)
        #    rate.sleep()
        print "_______________________________________"

        self.path=data
        self.t=0

        self.pose_init = self.path.poses[1]
        print self.pose_init

        quaternion_bot=self.pose_init.pose.orientation
        quat_bot=numpy.quaternion(quaternion_bot.w,quaternion_bot.x,quaternion_bot.y,quaternion_bot.z).normalized()
        quaternion = quaternion_from_euler(0.0,0.0,math.radians(0.0))
        qua_0_init=numpy.quaternion(quaternion[3],quaternion[0],quaternion[1],quaternion[2]).normalized()
        self.orientation_bottom=qua_0_init*quat_bot

        #self.increase_path_x=self.pose_init.pose.position.x
        #self.increase_path_y=self.pose_init.pose.position.y
        #self.increase_path_z=self.pose_init.pose.position.z

        #while True:
        #    self.publish_marker(self.marker_vis_pub)
        #    if raw_input("Press Enter to continue..."): break
        #    else: continue

        self.publish_marker(self.marker_vis_pub)
        self.publish_tf(self.pose_init.pose.position,self.orientation_bottom,"world","bottom")

        #quaternion=(self.path.poses[self.t].pose.orientation.x, self.path.poses[self.t].pose.orientation.y, self.path.poses[self.t].pose.orientation.z, self.path.poses[self.t].pose.orientation.w)  #Eseguo nella transform_callback
        #self.euler=euler_from_quaternion(quaternion) #Da voler eseguire nella transform_callback
        self.prev_a=0.0

        raw_input("Press Enter to continue...")

        while(self.t< len(self.path.poses)-1):
            rate=rospy.Rate(50)
            self.run_pub.publish(self.path.poses[self.t])
            self.t=self.t+1
            rate.sleep()

        cnt=1
        while(True):

            if(cnt==1):
                print "PERCORSO FINITO"
                cnt=cnt+1





    def run_callback(self,data):

        hk=self.hk

        #raw_input("Press Enter to continue...")
        ori_tip_0=numpy.quaternion(self.pose_0.pose.orientation.w, self.pose_0.pose.orientation.x, self.pose_0.pose.orientation.y, self.pose_0.pose.orientation.z).normalized()
        quaternion = quaternion_from_euler(0.0,0.0,math.radians(-self.euler[0]))
        c_pose_off=numpy.quaternion(quaternion[3],quaternion[0],quaternion[1],quaternion[2]).normalized()
        quat_init=numpy.quaternion(0.707,0.0,0.0,0.707).normalized()

        self.c_quat=self.qua_tip*quat_init
        self.current_pose.pose.position.x = self.increase_path_x + (self.deltam*self.hk+self.hk/2)*math.sin((-self.euler_bottom[2]+(self.deltam*self.euler[0])/self.deltam))
        self.current_pose.pose.position.z = self.increase_path_z + 0.0
        self.current_pose.pose.position.y = self.increase_path_y + (self.deltam*self.hk+self.hk/2)*math.cos((-self.euler_bottom[2]+(self.deltam*self.euler[0])/self.deltam))
        self.current_pose.pose.orientation.y = self.c_quat.y;
        self.current_pose.pose.orientation.z = self.c_quat.z;
        self.current_pose.pose.orientation.w = self.c_quat.w;

        self.publish_marker_pose(self.current_pose_pub,self.current_pose.pose)


        #self.t=0

        self.publish_marker(self.marker_vis_pub)

    #angle=math.radians(22.62)
    #if(self.t==int((2.0+2.5*math.pi+1.0+self.resolution)/self.resolution)): #VA INSERITO IL NUMERO MASSIMO DI "SEQ"
    #    self.t=0
    #    self.increase_path_x=0
    #    self.increase_path_y=0
    #    self.increase_path_z=0
    #    self.orientation_bottom=Quaternion(1.0,0.0,0.0,0.0)
    #    print "SEQ:"+str(self.t)
    #else:

        quaternion_bot=self.path.poses[self.t].pose.orientation
        quat_bot=numpy.quaternion(quaternion_bot.w,quaternion_bot.x,quaternion_bot.y,quaternion_bot.z).normalized()
        quaternion = quaternion_from_euler(0.0,0.0,math.radians(0.0))
        qua_0_init=numpy.quaternion(quaternion[3],quaternion[0],quaternion[1],quaternion[2]).normalized()
        self.orientation_bottom=qua_0_init*quat_bot


        #self.orientation_bottom=self.path.poses[self.t].pose.orientation

        print "SEQ:"+str(self.t)+"-----------------"+str(len(self.path.poses))

        self.publish_marker_pose(self.path_pose_pub,self.path.poses[self.t].pose)
        self.publish_tf(self.path.poses[self.t].pose.position,self.orientation_bottom,"world","bottom")

        ## ARCO CIRC MINIMA

        #x_tip=self.path.poses[self.t].pose.position.x+0.55*math.sin(-self.euler_bottom[2])
        #y_tip=self.path.poses[self.t].pose.position.y+0.55*math.cos(-self.euler_bottom[2])
        #x_tip_path=self.path.poses[self.t].pose.position.x
        #y_tip_path=self.path.poses[self.t].pose.position.y
        #x_diff=x_tip-x_tip_path
        #y_diff=y_tip-y_tip_path
        #min=math.sqrt(x_diff**2+y_diff**2)
        min=3
        theta=0
        pos=0
        A=0
        found=False
        #print x_tip,y_tip
        counter=0
        #print self.euler_bottom[2]
        ## CICLI FOR GENERALE
        #if(-self.euler_bottom[2]==0.0):
        #    print "____________________________________________CASO_1"
        #    theta = 0.0
        #    A=0.0
        #elif(-self.euler_bottom[2]==math.pi/2 and (self.path.poses[self.t+1].pose.position.x+0.55*math.sin(-self.euler_bottom[2])-(self.path.poses[self.t].pose.position.x+0.55*math.sin(-self.euler_bottom[2])))>0):
        #    print "____________________________________________CASO_2"
        #    theta = 0.0
        #    A=0.0
        #    print " thets 0"
        #elif(-self.euler_bottom[2]==-math.pi/2 and (self.path.poses[self.t+1].pose.position.x+0.55*math.sin(-self.euler_bottom[2])-(self.path.poses[self.t].pose.position.x+0.55*math.sin(-self.euler_bottom[2])))<0):
        #    print "____________________________________________CASO_3"
        #    theta = 0.0
        #    A=0.0
        #    print " thets 0"
        #elif(-self.euler_bottom[2]==math.pi/2 and (self.path.poses[self.t+1].pose.position.x+0.55*math.sin(-self.euler_bottom[2])-(self.path.poses[self.t].pose.position.x+0.55*math.sin(-self.euler_bottom[2])))<0):
        #    print "____________________________________________CASO_4"
        #    theta = 180.0
        #    A=0.0
        #    print " theta 180"
        #else:
        print "CASI_GENERALI"
        prev_a=self.prev_a
        prev_pos=self.prev_pos

        if ((prev_pos+2)>len(self.path.poses)-2): pos_fin=len(self.path.poses)-2
        else: pos_fin=prev_pos+2
        cnt=1
        if(self.prev_pos==len(self.path.poses)-2): #CONTROLLO PER LA PARTE FINALE IN UN PERCORSO
            #print "------------------CASO CON IF---------------------"
            p=len(self.path.poses)-1
            if ((self.path.poses[p].pose.position.y-self.path.poses[self.t].pose.position.y)==0):
                b=0.0
            else:
                #quaternion_bot=numpy.quaternion(self.path.poses[self.t].pose.orientation.w,self.path.poses[self.t].pose.orientation.x,self.path.poses[self.t].pose.orientation.y,self.path.poses[self.t].pose.orientation.z).normalized()

                b=math.atan((self.path.poses[p].pose.position.x-self.path.poses[self.t].pose.position.x)/(self.path.poses[p].pose.position.y-self.path.poses[self.t].pose.position.y))

                print self.path.poses[p].pose.position.x
                print self.path.poses[self.t].pose.position.x
                print "___ \n"

                print self.path.poses[p].pose.position.y
                print self.path.poses[self.t].pose.position.y
                #euler_from_quaternion(quaternion_bot)
                #quaternion_fin=(self.path.poses[p].pose.orientation.x,self.path.poses[p].pose.orientation.y,self.path.poses[p].pose.orientation.z,self.path.poses[p].pose.orientation.w)
                #euler_fin=euler_from_quaternion(quaternion_fin)
                #print "EUER FIN ---------- "+str(math.degrees(euler_fin[2]))
                #theta= -math.degrees(euler_fin[2])+math.degrees(self.euler_bottom[2])

            #if(b>0 and b<math.pi):
            #    b=b-math.pi
            #    print "BBBBBBBBBBBBBBBBBB"

            print math.degrees(b)

            if(self.euler_bottom[2]<-math.degrees(179.0)):
                print "________________________IF______________________________"
                theta=0.0#math.degrees(b)+math.degrees(-self.euler_bottom[2])
            elif (self.euler_bottom[2]>math.radians(89.0)):
                print "_____________________ELFIF________________"
                theta=360.0
            elif (self.euler_bottom[2]>math.radians(-89.0)):
                cnt=cnt+1;
                theta=0.0
                #theta=math.degree(self.final_goal)/(2*cnt)
            else:
                print "________________________ELSE______________________________"

                theta=-math.degrees(b)
            print theta

            print "EUER BOTTOM ---------- "+str(math.degrees(self.euler_bottom[2]))

        else:
            for a in numpy.arange(prev_a-4,prev_a+4,0.1):  #size marker?? ## CICLO PER L'ANGOLO
            #for a in numpy.arange(-55,55,0.1):  #size marker?? ## CICLO PER L'ANGOLO
                #print prev_a , prev_pos
                #print a

                for p in range (prev_pos-2, pos_fin+2):    ## MODIFICARE INSERENDO IL NUMERO MASSIMO DI SEQUENZE
                #for p in range (self.t+10, len(self.path.poses)-1):    ## MODIFICARE INSERENDO IL NUMERO MASSIMO DI SEQUENZE
                    #print p
                    if ((self.path.poses[p].pose.position.y-self.path.poses[self.t].pose.position.y)==0):
                        b=0.0
                        #print "CASO B==0"
                    else:
                        b=math.atan((self.path.poses[p].pose.position.x-self.path.poses[self.t].pose.position.x)/(self.path.poses[p].pose.position.y-self.path.poses[self.t].pose.position.y))

                        R=math.sqrt((self.path.poses[p].pose.position.x-self.path.poses[self.t].pose.position.x)**2+(self.path.poses[p].pose.position.y-self.path.poses[self.t].pose.position.y)**2)

                        #print math.degrees(-euler_bottom[2])

                        if(-self.euler_bottom[2]+math.radians(a)>math.pi/2):
                            #print "CASO IF"
                            x_tip       = self.path.poses[self.t].pose.position.x + (hk/2+self.h)*math.sin(-self.euler_bottom[2]+math.radians(a)+math.pi)
                            y_tip       = self.path.poses[self.t].pose.position.y + (hk/2+self.h)*math.cos(-self.euler_bottom[2]+math.radians(a)-math.pi)
                            #print math.degrees(self.euler_bottom[2]+math.radians(a)+math.pi)
                        #elif(-self.euler_bottom[2]+math.radians(a)<-math.pi/2):
                        #    print "_______________________________________CASO ELSE IF"
                        else:
                            #print "CASO ELSE"
                            x_tip       = self.path.poses[self.t].pose.position.x + (hk/2+self.h)*math.sin(-self.euler_bottom[2]+math.radians(a))
                            y_tip       = self.path.poses[self.t].pose.position.y + (hk/2+self.h)*math.cos(-self.euler_bottom[2]+math.radians(a))

                        x_tip_path  = self.path.poses[self.t].pose.position.x + R*math.sin((b))
                        y_tip_path  = self.path.poses[self.t].pose.position.y + R*math.cos((b))
                        x_diff      = x_tip-x_tip_path
                        y_diff      = y_tip-y_tip_path
                        err         = math.sqrt(x_diff**2+y_diff**2)
                        #found=False
                        if err<min:
                            min=err
                            pos=p
                            A=a
                            theta= a
                            #found=True

        #if(found==False): print "NON TROVATO"
        self.prev_a=theta
        if (pos>self.prev_pos):
            self.prev_pos=pos

        quaternion = quaternion_from_euler(0.0,0.0,math.radians(-theta))
        min_quat_off=numpy.quaternion(quaternion[3],quaternion[0],quaternion[1],quaternion[2]).normalized()

        ori_tip_0=numpy.quaternion(self.pose_0.pose.orientation.w, self.pose_0.pose.orientation.x, self.pose_0.pose.orientation.y, self.pose_0.pose.orientation.z).normalized()

        min_path_pose_ori=min_quat_off*ori_tip_0

        self.min_pose.pose.position.x = self.path.poses[self.t].pose.position.x + (hk/2)*math.sin(-self.euler_bottom[2])
        self.min_pose.pose.position.y = self.path.poses[self.t].pose.position.y + (hk/2)*math.cos(-self.euler_bottom[2])

        #min_pose.pose.orientation = min_path_pose_ori
        self.min_pose.pose.orientation.x = min_path_pose_ori.x;
        self.min_pose.pose.orientation.y = min_path_pose_ori.y;
        self.min_pose.pose.orientation.z = min_path_pose_ori.z;
        self.min_pose.pose.orientation.w = min_path_pose_ori.w;

        self.publish_marker_pose(self.min_pose_pub,self.min_pose.pose)
        self.publish_marker_pose(self.pose_0_pub,self.pose_0.pose)
        print "---------------------------------------- "+str(counter)
        print "MIN: "+str(min)
        print "POS: "+str(pos)
        #print "beta: "+str(math.degrees(math.atan((path.poses[pos].pose.position.x)/(path.poses[pos].pose.position.y))))
        #R=math.sqrt(path.poses[p].pose.position.x**2+path.poses[p].pose.position.y**2))
        print "THETA: "+str(theta)
        print "BOTTOM:" +str(math.degrees(-self.euler_bottom[2]))
        print "A:" +str(A)
        ## FINE

        self.increase_path_x=self.path.poses[self.t].pose.position.x
        self.increase_path_y=self.path.poses[self.t].pose.position.y
        self.increase_path_z=self.path.poses[self.t].pose.position.z

        quaternion=(self.qua_tip.x, self.qua_tip.y, self.qua_tip.z, self.qua_tip.w)
        euler_tip=euler_from_quaternion(quaternion)
        print "EUER TIP --- "+str(math.degrees(euler_tip[2]))
        quaternion1=(self.min_pose.pose.orientation.x, self.min_pose.pose.orientation.y, self.min_pose.pose.orientation.z, self.min_pose.pose.orientation.w)
        euler_goal=euler_from_quaternion(quaternion1)
	self.goal_angle_pub.publish(euler_goal[2])
        print "EULER GOAL --- "+str(math.degrees(euler_goal[2]))
        beta= math.pi/2.0 - euler_goal[2] + euler_tip[2]
        print  "BETA --- "+str(beta)
        print str(" \n ")
        self.final_goal=euler_goal[2]
        if(beta==2*math.pi): beta=0.0
        self.error_pub.publish(math.degrees(beta))


    def publish_marker(self, marker_vis_publisher):
        #global qr_, deltam, h
        #global pose_pub, pose_0_pub
        #
        #global orientation_bottom, euler_bottom
        #
        #global euler,hk,qua, qua_0
        #
        #global increase_path_x, increase_path_y, increase_path_z
        #global qua_tip, pos_tip, pose_0, pos_tip_0
        hk=self.hk
        markerArray=MarkerArray()

        for i in range (0,self.deltam+1):

            marker= Marker()
            marker.header.frame_id = "/world"
            marker.header.stamp = rospy.Time.now()
            #   Set the namespace and id for this marker.  This serves to create a unique ID
            marker.ns = "Cylinder1"
            marker.id = i
            marker.type = Marker.CYLINDER #SHAPE
            marker.action = Marker.ADD
            #   Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = hk
            quaternion=(self.qr_.x, self.qr_.y, self.qr_.z, self.qr_.w)  #Eseguo nella transform_callback
            self.euler=euler_from_quaternion(quaternion) #Da voler eseguire nella transform_callback
            ## EULER_BOTTOM SERVE PER LA POSIZIONE INIZIALE

            quaternion=(self.orientation_bottom.x, self.orientation_bottom.y, self.orientation_bottom.z, self.orientation_bottom.w)
            self.euler_bottom=euler_from_quaternion(quaternion)

            #   Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            if(i==(self.deltam+1)):   #NON ENTRA MAI (TUTTO IN ELSE)
                euler_i=self.euler[0]/self.deltam            #Da voler eseguire nella transform_callback
                print "EULER_I"+ str(euler_i)
                ##  ___CASO CON EM-TRACKER su X-Y
                marker.pose.position.x = ((self.hk/2+i*self.hk)*math.sin(i*(self.euler[0]/self.deltam)))
                marker.pose.position.z = 0.0;
                marker.pose.position.y = ((self.hk/2+i*self.hk)*math.cos(i*(self.euler[0]/self.deltam)))

                marker.pose.orientation.x = self.qua.x;
                marker.pose.orientation.y = self.qua.y;
                marker.pose.orientation.z = self.qua.z;
                marker.pose.orientation.w = self.qua.w;

                self.publish_marker_pose(self.pose_pub,marker.pose)

            else:
                euler_i=self.euler[0]/self.deltam           #Da voler eseguire nella transform_callback

                ##  ___CASO CON EM-TRACKER su X-Y

                marker.pose.position.x = self.increase_path_x + (i*self.hk+self.hk/2)*math.sin((-self.euler_bottom[2]+(i*self.euler[0])/self.deltam))
                marker.pose.position.z = self.increase_path_z + 0.0
                marker.pose.position.y = self.increase_path_y + (i*self.hk+self.hk/2)*math.cos((-self.euler_bottom[2]+(i*self.euler[0])/self.deltam))

                quaternion_bot=self.orientation_bottom
                quat_bot=numpy.quaternion(quaternion_bot.w,quaternion_bot.x,quaternion_bot.y,quaternion_bot.z).normalized()

                quaternion = quaternion_from_euler(0.0,0.0,-i*euler_i) #DEVO RUOTARE SU 'Z'
                quat=numpy.quaternion(quaternion[3],quaternion[0],quaternion[1],quaternion[2]).normalized()
                quat_init=numpy.quaternion(0.707,0.707,0.0,0.0).normalized()
                self.qua=quat_bot*quat*quat_init               # qua=quat_init*quat  DOVREBBE ANDARE SENZA QUA_INIT

                quaternion = quaternion_from_euler(0.0,0.0,math.radians(90.0))
                qua_0_init=numpy.quaternion(quaternion[3],quaternion[0],quaternion[1],quaternion[2]).normalized()
                self.qua_0=qua_0_init*quat_bot*quat_init

                marker.pose.orientation.x = self.qua.x;
                marker.pose.orientation.y = self.qua.y;
                marker.pose.orientation.z = self.qua.z;
                marker.pose.orientation.w = self.qua.w;

                if i==self.deltam:
                    self.publish_marker_pose(self.pose_pub,marker.pose)
                    quat_tip=numpy.quaternion(0.707,-0.707,0.0,0.0).normalized()
                    self.qua_tip=self.qua*quat_tip  #NECESSITO PER FAR COINCIDERE GLI ORIENTAMENTI DELLE 2 TF
                    self.pos_tip=marker.pose.position
                    self.publish_tf(self.pos_tip,self.qua_tip,"world","tip")              # --> cambiare il vettore di posizione (FIX)

                    #POSA FISSA SENZA BENDING ( necessito per calcolo errore )

                    #pose_0.header.seq=t*10
                    self.pose_0.pose.position.x = self.increase_path_x + (hk/2)*math.sin(-self.euler_bottom[2])
                    self.pose_0.pose.position.y = self.increase_path_y + (hk/2)*math.cos(-self.euler_bottom[2])

                    self.pose_0.pose.orientation.x = self.qua_0.x;
                    self.pose_0.pose.orientation.y = self.qua_0.y;
                    self.pose_0.pose.orientation.z = self.qua_0.z;
                    self.pose_0.pose.orientation.w = self.qua_0.w;

                    #publish_marker_pose(pose_0_pub,pose_0.pose)
                    self.pos_tip_0=self.pose_0.pose.position

                    #   Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = 0.1
            marker.color.g = 0.8
            marker.color.b = 0.1
            marker.color.a = 1.0
            markerArray.markers.append(marker)

        marker_vis_publisher.publish(markerArray)



    def publish_tf(self, msg_trans, msg_rot, fathername, childname):
        br = tf.TransformBroadcaster()

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = fathername
        t.child_frame_id = childname
        t.header.stamp = rospy.Time.now()

        t.transform.translation = msg_trans
        t.transform.rotation = msg_rot

        quat_init=numpy.quaternion(0.707,0.707,0.0,0.0).normalized()

        br.sendTransform((t.transform.translation.x,t.transform.translation.y,t.transform.translation.z),
        (t.transform.rotation.x, t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w),
        t.header.stamp, t.child_frame_id, t.header.frame_id)


    def publish_marker_pose(self,pose_publisher,marker_pose):

        self.pose.header.frame_id = "/world"
        self.pose.header.stamp = rospy.Time.now()
        self.pose.pose.orientation.x = marker_pose.orientation.x
        self.pose.pose.orientation.y = marker_pose.orientation.y
        self.pose.pose.orientation.z = marker_pose.orientation.z
        self.pose.pose.orientation.w = marker_pose.orientation.w
        self.pose.pose.position.x = marker_pose.position.x
        self.pose.pose.position.y = marker_pose.position.y
        self.pose.pose.position.z = marker_pose.position.z
        pose_publisher.publish(self.pose)

    def transform_callback(self,data):
        #global q_state, qr_, qr_2, IGTL_INIT_W, IGTL_INIT_X, IGTL_INIT_Y, IGTL_INIT_Z, bool_first,offset_x,offset_y,offset_z,posr_
        #global euler

        #print "----------------------TRANSF_CB--------------------------"
        q_out=data.transform.rotation
        pos_out=data.transform.translation
        if self.bool_first:
            #offset_x=pos_out.x
            #offset_y=pos_out.y
            #offset_z=pos_out.z
            self.bool_first=False
        else:
            self.posr_[0]=data.transform.translation.x
            self.posr_[1]=data.transform.translation.y
            self.posr_[2]=data.transform.translation.z
            q_state=numpy.quaternion(q_out.w,q_out.x,q_out.y,q_out.z).normalized()
            quat_init=numpy.quaternion(self.IGTL_INIT_W,self.IGTL_INIT_X,self.IGTL_INIT_Y,self.IGTL_INIT_Z).normalized()
            self.qr_=quat_init*q_state



            #self.current_pose.pose.position.x = self.path.poses[self.t].pose.position.x + (self.hk/2)*math.sin(-self.euler_bottom[2])
            #self.current_pose.pose.position.y = self.path.poses[self.t].pose.position.y + (self.hk/2)*math.cos(-self.euler_bottom[2])


            #ori_tip_0=numpy.quaternion(self.pose_0.pose.orientation.w, self.pose_0.pose.orientation.x, self.pose_0.pose.orientation.y, self.pose_0.pose.orientation.z).normalized()
            #quat_init=numpy.quaternion(0.707,0.0,-0.707,0.0).normalized()

            #self.c_quat=self.qr_*ori_tip_0*quat_init

            #self.current_pose.pose.orientation.x = self.c_quat.x;
            #self.current_pose.pose.orientation.y = self.c_quat.y;
            #self.current_pose.pose.orientation.z = self.c_quat.z;
            #self.current_pose.pose.orientation.w = self.c_quat.w;

            #self.publish_marker_pose(self.current_pose_pub,self.current_pose.pose)


            rate=rospy.Rate(400) #dipende dalla frequenza dell'EM Sensor
            rate.sleep();

    def igtl_callback(self,data):
        #rospy.loginfo(data) #NON STAMPO I DATI
        self.transform_callback(data)








def main (args):

    rospy.init_node('listener_igtl', anonymous=True)
    marker_pub=MarkerPublisher()
    rospy.spin()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass

