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

#include <dji_sdk_jsk/DjiInterface.h>

namespace dji_interface{
#define NO_TASK 0
#define TRACKING 1
#define POS 0
#define POS_VEL 1
  class DjiMotion{
  public:
    DjiMotion(ros::NodeHandle nh, ros::NodeHandle nhp);
    int motion_state_;
    int command_mode_;
    std::vector<double> local_target_position_;
    std::vector<double> target_velocity_;
    double target_yaw_;
    double max_vel_xy_;
    double max_vel_z_;
    double max_vel_yaw_;
    double pos_p_gains_;
    double yaw_p_gains_;
    double pos_task_pos_thre_;
    double pos_task_yaw_thre_;
    int pos_task_in_bound_cnt_;

    bool connect();
    bool takeOff();
    void setRelativeLocalTarget(double dx, double dy, double dz);
    void setRelativeLocalTarget(double dx, double dy, double dz, double yaw);
    void setLocalTarget(double x, double y, double z);
    void setLocalTarget(double x, double y, double z, double yaw);
    void setGpsTarget(sensor_msgs::NavSatFix& target);
    void setGpsTarget(sensor_msgs::NavSatFix& target, double yaw);
    void setVelocityTarget(double vx, double vy, double vz, double vyaw);
    void setMaxVelocity(double max_vel_xy, double max_vel_z, double nax_vel_yaw);
    Eigen::Vector3d getLocalPosition();
    Eigen::Vector3d getGpsPosition();
    Eigen::Vector3d getVelocity();
    void checkVelocityLimits(Eigen::Vector3d& cmd_vel);

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Timer control_timer_;
    double control_timer_freq_;

    DjiInterface* dji_interface_;

    void rangeCheck(double& value, double max_val);

    // ros::Subscriber gps_position_sub_;

    // void localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg);

    void controlTimercallback(const ros::TimerEvent&);
  };
}
