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

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Joy.h>

#include <tf/tf.h>

//DJI SDK includes
#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>

namespace dji_interface{
#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

  class DjiInterface{
  public:
    DjiInterface(ros::NodeHandle nh, ros::NodeHandle nhp);
    bool connectM100();
    bool M100TakeOff();
    void velocityControl(double vel_x, double vel_y, double vel_z, double vel_yaw);
    sensor_msgs::NavSatFix current_gps_position_;
    uint8_t flight_status_;
    uint8_t current_gps_health_;
    geometry_msgs::PointStamped current_local_position_;
    bool current_local_position_update_;
    geometry_msgs::Quaternion current_atti_;
    void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                                  sensor_msgs::NavSatFix& target,
                                  sensor_msgs::NavSatFix& origin);
    geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Subscriber gps_position_sub_;
    ros::Subscriber gps_health_sub_;
    ros::Subscriber flight_status_sub_;
    ros::Subscriber local_position_sub_;
    ros::Subscriber attitude_sub_;

    ros::Publisher flight_control_pub_;

    ros::ServiceClient sdk_ctrl_authority_srv_;
    ros::ServiceClient set_local_pos_reference_srv_;
    ros::ServiceClient query_version_srv_;
    ros::ServiceClient drone_task_srv_;

    bool obtainControl();
    bool setLocalPosition();
    bool isM100();
    bool DjiTaskControl(int task);

    void gpsPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void gpsHealthCallback(const std_msgs::UInt8::ConstPtr& msg);
    void flightStatusCallback(const std_msgs::UInt8::ConstPtr& msg);
    void localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void attitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
  };
}
