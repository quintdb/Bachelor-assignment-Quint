#!/usr/bin/env python

import rospy
import tf, numpy, quaternion
import geometry_msgs.msg

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped

import math

i=0



def path_callback(data):
    global i,resolution

    path=data
    if(i==5/resolution):
        i=0
        print "SEQ:"+str(i)
    else:
        print "SEQ:"+str(i)
        #if path.poses[i].header.seq == i:
        #publish_tf(path.poses[i].pose.position, path.poses[i].pose.orientation,"path_seg")
        i=i+1
        publish_marker_pose(path_pose_pub,path.poses[i].pose)


def publish_marker_pose(pose_publisher,marker_pose):
    pose = PoseStamped()
    pose.header.frame_id = "/world"
    pose.header.stamp = rospy.Time.now()
    pose.pose.orientation.x = marker_pose.orientation.x
    pose.pose.orientation.y = marker_pose.orientation.y
    pose.pose.orientation.z = marker_pose.orientation.z
    pose.pose.orientation.w = marker_pose.orientation.w
    pose.pose.position.x = marker_pose.position.x
    pose.pose.position.y = marker_pose.position.y
    pose.pose.position.z = marker_pose.position.z
    pose_publisher.publish(pose)


def publish_tf(msg_trans, msg_rot, childname):
    global euler,hk,qua

    br = tf.TransformBroadcaster()

    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "world"
    t.child_frame_id = childname
    t.header.stamp = rospy.Time.now()

    t.transform.translation = msg_trans
    t.transform.rotation = msg_rot

    #q_state=numpy.quaternion(q_out.w,q_out.x,q_out.y,q_out.z)
    quat_init=numpy.quaternion(0.707,0.707,0.0,0.0)
    #t.transform.rotation=quat_init*q_state

    #br.sendTransform((msg_trans.x,msg_trans.y,msg_trans.z),
    #br.sendTransform((((hk/2+deltam*hk)*math.sin((euler[0]))) , ((hk/2+deltam*hk)*math.cos(euler[0])) ,0.0),
    br.sendTransform((t.transform.translation.x,t.transform.translation.y,t.transform.translation.z),
                        (t.transform.rotation.x, t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w),
                        t.header.stamp, t.child_frame_id, t.header.frame_id)

def main():
    global resolution

    global path_pose_pub
    rospy.init_node('path_node')

    path_pub = rospy.Publisher('/plan', Path, queue_size=10000)
    #path_sub = rospy.Subscriber('/path1', Path, path_callback)
    path_pose_pub = rospy.Publisher('/path_pose', PoseStamped, queue_size=1000)


    path = Path()


    br = tf.TransformBroadcaster()




    path.header.frame_id = rospy.get_param('~output_frame', 'world')
    radius = rospy.get_param('~radius', 10.0)
    resolution = rospy.get_param('~resolution', 0.01)
    holonomic = rospy.get_param('~holonomic', False)
    offset_x = rospy.get_param('~offset_x', 0.0)
    offset_y = rospy.get_param('~offset_y', 0.0)
    update_rate = rospy.get_param('~update_rate', 10.0)

    has_initialize = True

    x0 = rospy.get_param('~x0', 0.0)
    y0 = rospy.get_param('~y0', 0.0)
    m = rospy.get_param('~m', 0.0)

    bool_pos_neg = rospy.get_param('~bool_pos_neg', False)  # Boleano per capire se passa da positivo a negativo o viceversa (TRUE: positivo )
    bool_neg_pos = rospy.get_param('~bool_neg_pos', True)  # Boleano per capire se passa da positivo a negativo o viceversa (TRUE: positivo )


    for t in numpy.arange(0.0 , math.pi+1.0 ,resolution):


        if(t<0.5):
            x=t
            y=0.0
        elif (t>math.pi+0.5):
            t=t-math.pi-0.5
            x=math.cos(math.pi/2)+0.5-t
            y=old_y
        else:
            t=t-0.5
            y=math.sin(t-math.pi/2)+1
            x=math.cos(t-math.pi/2)+0.5
            z=0.0;


        if has_initialize:
            old_x = x
            old_y = y
            yaw_old=1
            has_initialize = False

        pose = PoseStamped()
        pose.header.seq=t/resolution
        pose.pose.position.x = x
        pose.pose.position.y = y


        delta_x=x-old_x
        delta_y=y-old_y

        if delta_y==0.0 and delta_x<0: yaw=-math.pi/2
        elif delta_y==0.0 and delta_x>0: yaw=math.pi/2
        elif delta_x==0.0: yaw=0.0
        else:
            yaw =math.atan(delta_x/delta_y)

        #if (yaw_old<0 and yaw > 0):
        #    bool_neg_pos=True
        #    bool_pos_neg=False
        #elif (yaw_old>0 and yaw<0):
        #    bool_pos_neg=True
        #    bool_neg_pos=False

        if (yaw < 0 and yaw-yaw_old < math.radians(-90)):
            bool_pos_neg=True
        elif (yaw < 0 and yaw-yaw_old < 0):
            bool_pos_neg=False
        else: bool_pos_neg=False

        if (bool_pos_neg==True): yaw = yaw + math.pi

        #if (yaw < 0): yaw=yaw+math.pi


    ## METTERE LE CONDIZIONI IN CUI DIVENTANO FALSE

        #if(y > 0 and bool_sign==True): bool_sign=False

        print math.degrees(yaw), bool_pos_neg

        #if holonomic:
        #    yaw = -math.sin(t) / math.cos(t)
        #else:
        #    if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
        #        yaw = math.atan2(old_y - y, old_x - x)
        #    else:
        #        yaw = math.atan2(y - old_y, x - old_x)


        q = tf.transformations.quaternion_from_euler(0, 0, -yaw-math.radians(360))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)

        old_x = x
        old_y = y
        yaw_old=yaw

    r = rospy.Rate(update_rate)

    while not rospy.is_shutdown():

        path.header.stamp = rospy.Time.now()
        path_pub.publish(path)

        #for i in range(0,5):
        #    if path.poses[i].header.seq == i:
        #        publish_tf(path.poses[i].pose.position, path.poses[i].pose.orientation,"path_seg")
        r.sleep()


if __name__ == '__main__':
    main()
