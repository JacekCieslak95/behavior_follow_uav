/*!*******************************************************************************************
 *  \file       behavior_follow_UAV.cpp
 *  \brief      Behavior Follow UAV implementation file.
 *  \details    This file implements the behaviorFollowUAV class.
 *  \authors    Jacek Cieślak
 *  \copyright  Copyright 2017 Politechnika Poznańska (PUT) *
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
#include "../include/behavior_follow_uav.h"

BehaviorFollowUAV::BehaviorFollowUAV(){

}

BehaviorFollowUAV::~BehaviorFollowUAV(){

}

void BehaviorFollowUAV::ownSetUp(){
  std::cout << "ownSetUp" << std::endl;

  node_handle.param<std::string>("drone_id", drone_id, "1");
  node_handle.param<std::string>("drone_id_namespace", drone_id_namespace, "drone"+drone_id);
  node_handle.param<std::string>("my_stack_directory", my_stack_directory,
                                 "~/workspace/ros/quadrotor_stack_catkin/src/quadrotor_stack");
  node_handle.param<std::string>("estimated_pose_topic", estimated_pose_str, "estimated_pose");
  node_handle.param<std::string>("controllers_topic", controllers_str, "command/high_level");
  node_handle.param<std::string>("rotation_angles_topic", rotation_angles_str, "rotation_angles");
  node_handle.param<std::string>("estimated_speed_topic",estimated_speed_str,"estimated_speed");
  node_handle.param<std::string>("yaw_controller_str",yaw_controller_str , "droneControllerYawRefCommand");
  node_handle.param<std::string>("service_topic_str",service_topic_str , "droneTrajectoryController/setControlMode");
  node_handle.param<std::string>("drone_position_str",drone_position_str , "dronePositionRefs");
  node_handle.param<std::string>("speed_topic",speed_topic , "droneSpeedsRefs");
  node_handle.param<std::string>("drone_control_mode",drone_control_mode_str,"droneTrajectoryController/controlMode");
  node_handle.param<std::string>("d_altitude",d_altitude_str,"command/dAltitude");
  node_handle.param<std::string>("consult_belief",execute_query_srv,"consult_belief");
  //node_handle.param<std::string>("behavior_rotate_start",rotation_start_srv,"behavior_rotate/start");
  //node_handle.param<std::string>("behavior_rotate_stop",rotation_stop_srv,"behavior_rotate/stop");

}

void BehaviorFollowUAV::ownStart(){
  is_finished = false;
  state = 0;
  std::cout << "ownStart" << std::endl;
  //Initialize topics
  estimated_pose_sub = node_handle.subscribe(estimated_pose_str, 1000, &BehaviorFollowUAV::estimatedPoseCallBack, this);
  rotation_angles_sub = node_handle.subscribe(rotation_angles_str, 1000, &BehaviorFollowUAV::rotationAnglesCallback, this);
  estimated_speed_sub = node_handle.subscribe(estimated_speed_str, 1000, &BehaviorFollowUAV::estimatedSpeedCallback, this);
  controllers_pub = node_handle.advertise<droneMsgsROS::droneCommand>(controllers_str, 1, true);
  yaw_controller_pub=node_handle.advertise<droneMsgsROS::droneYawRefCommand>(yaw_controller_str,1000);
  mode_service=node_handle.serviceClient<droneMsgsROS::setControlMode>(service_topic_str);
  drone_position_pub=node_handle.advertise< droneMsgsROS::dronePositionRefCommandStamped>(drone_position_str,1000);
  speed_topic_pub=node_handle.advertise<droneMsgsROS::droneSpeeds>(speed_topic,1000);
  d_altitude_pub = node_handle.advertise<droneMsgsROS::droneDAltitudeCmd>(d_altitude_str,1);
  query_client = node_handle.serviceClient <droneMsgsROS::ConsultBelief> (execute_query_srv);
  /*
   * Add rotation? in time of XY movement
   */

  //get arguments
  std::string arguments=getArguments();
  YAML::Node config_file = YAML::Load(arguments);

  //get leader ID
  if(config_file["droneID"].IsDefined()){
    leaderID=config_file["droneID"].as<int>();
  }
  else{
    setStarted(false);
    return;
  }
  //get relative position to leader
  if(config_file["relative_position"].IsDefined()){
    std::vector<double> points = config_file["relative_position"].as<std::vector<double>>();
    relative_target_position.x = points[0];
    relative_target_position.y = points[1];
    relative_target_position.z = points[2];
  }
  else{
    relative_target_position.x = -1;
    relative_target_position.y = 0;
    relative_target_position.z = 0;
    //setStarted(false);
    //return;
  }

  estimated_leader_pose_str = std::string("/drone") + std::to_string(leaderID) + std::string("/estimated_pose");
  estimated_leader_pose_sub = node_handle.subscribe(estimated_leader_pose_str, 1000, &BehaviorFollowUAV::estimatedLeaderPoseCallBack, this);

  //calculate target position for the first time
  target_position.x = estimated_leader_pose_msg.x + relative_target_position.x * cos(estimated_leader_pose_msg.yaw);
  target_position.y = estimated_leader_pose_msg.y + relative_target_position.y * sin(estimated_leader_pose_msg.yaw);
  target_position.z = estimated_leader_pose_msg.z + relative_target_position.z;
  target_position.yaw = estimated_leader_pose_msg.yaw;

  //calculate distance and speed
  distance = sqrt(pow(target_position.x-estimated_pose_msg.x,2)
                         + pow(target_position.y-estimated_pose_msg.y,2));
  //                       + pow(target_position.z-estimated_pose_msg.z,2));

  if (distance < 0.1) speed = 0.0;
  else if (distance > 5.0 ) speed = 5.0;
  else speed = 1.0 * distance;

  //calculate setpoint speeds in xy for the first time

  setpoint_speed_msg.dx = speed * (target_position.x - estimated_pose_msg.x) / distance;
  setpoint_speed_msg.dy = speed * (target_position.y - estimated_pose_msg.y) / distance;
  setpoint_speed_msg.dz = 0;

  //set SPEED_CONTROL mode and command MOVE
  estimated_speed_msg = *ros::topic::waitForMessage<droneMsgsROS::droneSpeeds>(estimated_speed_str, node_handle, ros::Duration(2));
  estimated_pose_msg = *ros::topic::waitForMessage<droneMsgsROS::dronePose>(estimated_pose_str, node_handle, ros::Duration(2));

  droneMsgsROS::setControlMode mode;
  mode.request.controlMode.command=mode.request.controlMode.SPEED_CONTROL;
  mode_service.call(mode);

  droneMsgsROS::droneSpeeds point;
  point.dx=estimated_speed_msg.dx;
  point.dy=estimated_speed_msg.dy;
  point.dz=estimated_speed_msg.dz;
  speed_topic_pub.publish(point);

  ros::topic::waitForMessage<droneMsgsROS::droneTrajectoryControllerControlMode>(
    drone_control_mode_str, node_handle
  );

  droneMsgsROS::droneSpeeds droneSpeed;
  droneSpeed.dx=setpoint_speed_msg.dx;
  droneSpeed.dy=setpoint_speed_msg.dy;
  droneSpeed.dz=setpoint_speed_msg.dz;
  speed_topic_pub.publish(droneSpeed);

  droneMsgsROS::droneCommand msg;
  msg.command = droneMsgsROS::droneCommand::MOVE;
  controllers_pub.publish(msg);

  estimated_speed_msg = *ros::topic::waitForMessage<droneMsgsROS::droneSpeeds>(estimated_speed_str, node_handle, ros::Duration(2));

}

void BehaviorFollowUAV::ownRun(){

  std::cout << "ownRun" << std::endl;
  //calculate target position for the first time
  target_position.x = estimated_leader_pose_msg.x + relative_target_position.x * cos(estimated_leader_pose_msg.yaw) + relative_target_position.y * sin(estimated_leader_pose_msg.yaw);
  target_position.y = estimated_leader_pose_msg.y + relative_target_position.x * sin(estimated_leader_pose_msg.yaw) + relative_target_position.y * cos(estimated_leader_pose_msg.yaw);
  target_position.z = estimated_leader_pose_msg.z + relative_target_position.z;
  target_position.yaw = estimated_leader_pose_msg.yaw;

  //calculate distance and speed
  distance = sqrt(pow(target_position.x-estimated_pose_msg.x,2)
                         + pow(target_position.y-estimated_pose_msg.y,2));
  //                       + pow(target_position.z-estimated_pose_msg.z,2));

  if (distance < 0.1) speed = 0.0;
  else if (distance > 5.0 ) speed = 5.0;
  else speed = 1.0 * distance;

  //calculate setpoint speeds in xy for the first time

  setpoint_speed_msg.dx = speed * (target_position.x - estimated_pose_msg.x) / distance;
  setpoint_speed_msg.dy = speed * (target_position.y - estimated_pose_msg.y) / distance;
  setpoint_speed_msg.dz = 0;

  droneMsgsROS::droneSpeeds droneSpeed;
  droneSpeed.dx=setpoint_speed_msg.dx;
  droneSpeed.dy=setpoint_speed_msg.dy;
  droneSpeed.dz=setpoint_speed_msg.dz;
  speed_topic_pub.publish(droneSpeed);


/*
  std::cout << " x: " << estimated_leader_pose_msg.x
            << " y: " << estimated_leader_pose_msg.y
            << " z: " << estimated_leader_pose_msg.z
            << " yaw: " << estimated_leader_pose_msg.yaw
            << " xt: " << target_position.x
            << " yt: " << target_position.y
            << " zt: " << target_position.z
            << " yawt: " << target_position.yaw << std::endl;
*/



  /*
   * calculate speeds based on target position
   *
   * send calculated speed
   */
}

void BehaviorFollowUAV::ownStop(){


  //think about what to add here

  estimated_pose_sub.shutdown();
  estimated_speed_sub.shutdown();
  rotation_angles_sub.shutdown();

}

std::tuple<bool,std::string> BehaviorFollowUAV::ownCheckSituation()
{
  droneMsgsROS::ConsultBelief query_service;
  std::ostringstream capturador;
  capturador << "battery_level(self,LOW)";
  std::string query(capturador.str());
  query_service.request.query = query;
  query_client.call(query_service);
  if(query_service.response.success)
  {
    return std::make_tuple(false,"Error: Battery low, unable to perform action");
    //return false;
  }
  std::ostringstream capturador2;
  capturador2<<"flight_state(self,LANDED)";
  std::string query2(capturador2.str());
  query_service.request.query = query2;
  query_client.call(query_service);
  if(query_service.response.success)
  {
    //Change it after tests!!!
    //return std::make_tuple(true,"Warning: Drone landed");
    return std::make_tuple(false,"Error: Drone landed");
    //return false;
  }

  return std::make_tuple(true,"");
}


//CallBacks
void BehaviorFollowUAV::estimatedSpeedCallback(const droneMsgsROS::droneSpeeds& msg){
  estimated_speed_msg=msg;
}
void BehaviorFollowUAV::estimatedPoseCallBack(const droneMsgsROS::dronePose& msg){
  estimated_pose_msg=msg;
}
void BehaviorFollowUAV::estimatedLeaderPoseCallBack(const droneMsgsROS::dronePose& msg){
  estimated_leader_pose_msg=msg;
}
void BehaviorFollowUAV::rotationAnglesCallback(const geometry_msgs::Vector3Stamped& msg){
  rotation_angles_msg=msg;
}
