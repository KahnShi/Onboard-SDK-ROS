// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
 *********************************************************************/

#include <dji_sdk_jsk/DjiInterface.h>

namespace dji_interface{
  const float deg2rad = C_PI/180.0;
  const float rad2deg = 180.0/C_PI;
  DjiInterface::DjiInterface(ros::NodeHandle nh, ros::NodeHandle nhp){
    nh_ = nh;
    nhp_ = nhp;

    flight_status_ = 255;

    gps_position_sub_ = nh_.subscribe("dji_sdk/gps_position", 10, &DjiInterface::gpsPositionCallback, this, ros::TransportHints().tcpNoDelay());
    gps_health_sub_ = nh_.subscribe("dji_sdk/gps_health", 10, &DjiInterface::gpsHealthCallback, this, ros::TransportHints().tcpNoDelay());
    flight_status_sub_ = nh_.subscribe("dji_sdk/flight_status", 10, &DjiInterface::flightStatusCallback, this, ros::TransportHints().tcpNoDelay());
    local_position_sub_ = nh_.subscribe("dji_sdk/local_position", 10, &DjiInterface::localPositionCallback, this, ros::TransportHints().tcpNoDelay());

    sdk_ctrl_authority_srv_ = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
    set_local_pos_reference_srv_ = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");
    query_version_srv_ = nh_.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
    drone_task_srv_ = nh_.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");

    ROS_INFO("DJI init finished.");
  }

  bool DjiInterface::connectM100(){
    ROS_INFO("Start to connect M100.");
    if (!obtainControl())
      return false;
    sleep(0.1);
    if (!setLocalPosition()){
      ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
      return false;
    }
    if(!isM100()){
      ROS_ERROR("DRONE is not M100!");
      return false;
    }

    ROS_INFO("Succeed to connect M100.");
    return true;
  }

  bool DjiInterface::M100TakeOff(){
    ROS_INFO("M100 start to takeoff.");
    ros::Time start_time = ros::Time::now();
    float home_altitude = current_gps_position_.altitude;
    if(!DjiTaskControl(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)){
      ROS_ERROR("TAKEOFF task service fails to send to M100");
      return false;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();

    // // Step 1: If M100 is not in the air after 10 seconds, fail.
    while (ros::Time::now() - start_time < ros::Duration(10))
      {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
      }

    if(flight_status_ != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
       current_gps_position_.altitude - home_altitude < 1.0){
      ROS_ERROR("Takeoff failed.");
      return false;
    }
    else{
      start_time = ros::Time::now();
      ROS_INFO("Successful takeoff!");
      ros::spinOnce();
    }
    ROS_INFO("M100 succeed to takeoff.");
    return true;
  }

  bool DjiInterface::obtainControl(){ // from demo_local_position_control::obtain_control
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable=1;
    sdk_ctrl_authority_srv_.call(authority);

    if(!authority.response.result){
      ROS_ERROR("obtain control failed!");
      return false;
    }
    else
      return true;
  }

  void DjiInterface::localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                                              sensor_msgs::NavSatFix& target,
                                              sensor_msgs::NavSatFix& origin)
  {
    double deltaLon = target.longitude - origin.longitude;
    double deltaLat = target.latitude - origin.latitude;

    deltaNed.y = deltaLat * deg2rad * C_EARTH;
    deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
    deltaNed.z = target.altitude - origin.altitude;
  }

  bool DjiInterface::setLocalPosition(){ // from demo_local_position_control::set_local_position
    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference_srv_.call(localPosReferenceSetter);
    return (bool)localPosReferenceSetter.response.result;
  }

  bool DjiInterface::isM100(){ // from demo_local_position_control::is_M100
    dji_sdk::QueryDroneVersion query;
    query_version_srv_.call(query);

    if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
      return true;
    else
      return false;
  }

  bool DjiInterface::DjiTaskControl(int task) // from demo_local_position_control::takeoff_land
  {
    dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = task;

    drone_task_srv_.call(droneTaskControl);

    if(!droneTaskControl.response.result){
        ROS_ERROR("DJI drone task control fails");
        return false;
    }
    return true;
  }

  void DjiInterface::gpsPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    current_gps_position_ = *msg;
  }

  void DjiInterface::flightStatusCallback(const std_msgs::UInt8::ConstPtr& msg)
  {
    flight_status_ = msg->data;
  }

  void DjiInterface::localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
    current_local_position_ = *msg;
  }

  void DjiInterface::gpsHealthCallback(const std_msgs::UInt8::ConstPtr& msg){
    current_gps_health_ = msg->data;
  }
}
