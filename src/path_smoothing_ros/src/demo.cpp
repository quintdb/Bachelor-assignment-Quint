/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, George Kouros.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  George Kouros
*********************************************************************/

#include "path_smoothing_ros/cubic_spline_interpolator.hpp"
#include <tf/tf.h>
#include <math.h>

#define _USE_MATH_DEFINES
#include <cmath>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_smoothing_ros_demo");
  ros::NodeHandle nh("~");
  ROS_INFO_STREAM("Namespace:" << nh.getNamespace());

  ros::Publisher initialPosePub = nh.advertise<geometry_msgs::PoseStamped>("initial_pose", 1, true);
  ros::Publisher finalPosePub = nh.advertise<geometry_msgs::PoseStamped>("final_pose", 1, true);
  ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("/initial_path", 1, true);
  ros::Publisher smoothedPathPub = nh.advertise<nav_msgs::Path>("/plan", 1, true);

  int pointsPerUnit, skipPoints;
  bool useEndConditions, useMiddleConditions;

  float yaw_old,x_old,y_old,delta_x,delta_y;

  nh.param<int>("points_per_unit", pointsPerUnit, 400);
  nh.param<int>("skip_points", skipPoints, 0);
  nh.param<bool>("use_end_conditions", useEndConditions, false);
  nh.param<bool>("use_middle_conditions", useMiddleConditions, false);

  XmlRpc::XmlRpcValue poseList;
  if (!nh.getParam("path_poses", poseList))
  {
    ROS_FATAL("Failed to load path point list");
    exit(EXIT_FAILURE);
  }

  nav_msgs::Path path, smoothedPath;
  path.header.frame_id = "world";
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";

  float old_x=0;
  float old_y=0;
  float yaw=0;
  float x=0;
  float y=0;
  bool bool_pos_neg=false;

  for (int i = 0; i < poseList.size(); i++)
  {
    pose.pose.position.x = static_cast<double>(poseList[i]["x"]);
    pose.pose.position.y = static_cast<double>(poseList[i]["y"]);

    //x=pose.pose.position.x;
    //y=pose.pose.position.y;

    //delta_x=x-old_x;
    //delta_y=y-old_y;

    //if(delta_y==0.0) {yaw=0.0;}
    //else{

    //  yaw=atan(delta_x/delta_y);
    //}

    //if(yaw<0 && yaw-yaw_old<-M_PI){
    //  bool_pos_neg=true;
    //}
    //else{
    //  bool_pos_neg=false;
    //}


    //if(bool_pos_neg==true){yaw=yaw+M_PI;}

    //printf("GRADI: %f\n", (yaw*180)/M_PI );;
    //old_x=pose.pose.position.x;
    //old_y=pose.pose.position.y;

    pose.pose.orientation = tf::createQuaternionMsgFromYaw(poseList[i]["yaw"]);
    pose.header.seq=i;
    path.poses.push_back(pose);
  }

  // create a cubic spline interpolator
  path_smoothing::CubicSplineInterpolator csi("lala");
    // pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
  csi.interpolatePath(path, smoothedPath);

  initialPosePub.publish(path.poses.front());
  finalPosePub.publish(path.poses.back());


  ros::Time currTime = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();
    pathPub.publish(path);
    smoothedPathPub.publish(smoothedPath);
  }

  return 0;
}
