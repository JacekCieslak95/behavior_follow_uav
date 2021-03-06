/*!*******************************************************************************************
 *  \file       behavior_follow_uav.h
 *  \brief      behavior follow uav definition file.
 *  \details    This file contains the behaviorFollowUAV declaration. To obtain more information about
 *              it's definition consult the behavior_follow_UAV.cpp file.
 *  \authors    Jacek Cieślak
 *  \copyright  Copyright 2017 Politechnika Poznańska (PUT)
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program. If not, see http://www.gnu.org/licenses/.
 ********************************************************************************/
#ifndef FOLLOW_UAV_H
#define FOLLOW_UAV_H

#include <string>
#include <math.h>
// ROS
#include <ros/ros.h>
#include "std_srvs/Empty.h"
#include <droneMsgsROS/droneSpeeds.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <yaml-cpp/yaml.h>
#include <droneMsgsROS/dronePositionRefCommandStamped.h>
#include <droneMsgsROS/dronePositionTrajectoryRefCommand.h>
#include <droneMsgsROS/droneYawRefCommand.h>
#include <droneMsgsROS/droneTrajectoryControllerControlMode.h>
#include <droneMsgsROS/setControlMode.h>
#include <droneMsgsROS/ConsultBelief.h>
#include <tuple>
// Aerostack msgs
#include <droneMsgsROS/BehaviorEvent.h>
#include <droneMsgsROS/dronePose.h>
#include <droneMsgsROS/droneCommand.h>
#include <droneMsgsROS/dronePitchRollCmd.h>
#include <droneMsgsROS/droneDAltitudeCmd.h>
#include <droneMsgsROS/droneDYawCmd.h>
#include <droneMsgsROS/askForModule.h>
//Aerostack libraries
#include <behavior_process.h>

class BehaviorFollowUAV: public BehaviorProcess{
public:
  BehaviorFollowUAV();
  ~BehaviorFollowUAV();
private:
  ros::NodeHandle node_handle;

  //Config variables
  std::string drone_id;
  std::string drone_id_namespace;
  std::string my_stack_directory;
  std::string behavior_name_str;
  std::string estimated_pose_str;
  std::string estimated_leader_pose_str;
  std::string rotation_angles_str;
  std::string controllers_str;
  std::string estimated_speed_str;
  std::string estimated_leader_speed_str;
  std::string yaw_controller_str;
  std::string service_topic_str;
  std::string drone_position_str;
  std::string speed_topic;
  std::string drone_control_mode_str;
  std::string d_altitude_str;
  std::string d_yaw_str;
  std::string execute_query_srv;
  std::string rotation_start_srv;
  std::string rotation_stop_srv;

  //Subscriber
  ros::Subscriber estimated_pose_sub;
  ros::Subscriber estimated_leader_pose_sub;
  ros::Subscriber estimated_speed_sub;
  ros::Subscriber estimated_leader_speed_sub;
  ros::Subscriber rotation_angles_sub;

  //Publisher
  ros::Publisher controllers_pub;
  ros::Publisher yaw_controller_pub;
  ros::Publisher drone_position_pub;
  ros::Publisher speed_topic_pub;
  ros::Publisher d_altitude_pub;
  ros::Publisher d_yaw_pub;

  //Service Clients
  ros::ServiceClient mode_service;
  ros::ServiceClient query_client;
  //ros::ServiceClient rotation_start_client;
  //ros::ServiceClient rotation_stop_client;

  //Message
  droneMsgsROS::dronePose estimated_pose_msg;
  droneMsgsROS::dronePose estimated_leader_pose_msg;
  droneMsgsROS::dronePose target_position;
  droneMsgsROS::dronePose relative_target_position;
  droneMsgsROS::droneSpeeds estimated_speed_msg;
  droneMsgsROS::droneSpeeds estimated_leader_speed_msg;
  droneMsgsROS::droneSpeeds setpoint_speed_msg;
  geometry_msgs::Vector3Stamped rotation_angles_msg;

  bool is_finished;
  void ownSetUp();
  void ownStart();
  void ownRun();
  void ownStop();
  float calculateDYaw();


  float angle;
  float speed;
  float distance;
  int leaderID;

  std::tuple<bool, std::string> ownCheckSituation();

  //CallBacks
  void estimatedPoseCallBack(const droneMsgsROS::dronePose&);
  void estimatedLeaderPoseCallBack(const droneMsgsROS::dronePose&);
  void estimatedSpeedCallback(const droneMsgsROS::droneSpeeds&);
  void estimatedLeaderSpeedCallback(const droneMsgsROS::droneSpeeds&);
  void rotationAnglesCallback(const geometry_msgs::Vector3Stamped&);
};

#endif
