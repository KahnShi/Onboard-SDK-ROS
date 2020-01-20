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

#include <dji_sdk_jsk/DjiMotion.h>

namespace dji_interface{
  DjiMotion::DjiMotion(ros::NodeHandle nh, ros::NodeHandle nhp){
    nh_ = nh;
    nhp_ = nhp;

    motion_state_ = NO_TASK;
    command_mode_ = POS;
    local_target_position_.resize(3);
    for (int i = 0; i < 3; ++i) target_velocity_.push_back(0.0);
    pos_task_in_bound_cnt_ = 0;

    dji_interface_ = new DjiInterface(nh_, nhp_);
    ros::Time start_time = ros::Time::now();
    while (ros::Time::now() - start_time < ros::Duration(1.0)){
      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }

    nhp_.param("control_timer_freq", control_timer_freq_, 0.01);
    nhp_.param("max_vel_xy", max_vel_xy_, 3.0);
    nhp_.param("max_vel_z", max_vel_z_, 1.0);
    nhp_.param("max_vel_yaw", max_vel_yaw_, 0.4);
    nhp_.param("pos_p_gains", pos_p_gains_, 0.3);
    nhp_.param("yaw_p_gains", yaw_p_gains_, 0.2);
    nhp_.param("pos_task_pos_thre_", pos_task_pos_thre_, 0.25);
    nhp_.param("pos_task_yaw_thre_", pos_task_yaw_thre_, 0.1);

    control_timer_ = nh.createTimer(ros::Duration(0.02), &DjiMotion::controlTimercallback, this);
  }

  void DjiMotion::controlTimercallback(const ros::TimerEvent&){
    if (motion_state_ == NO_TASK)
      return;

    // pid
    std::vector<double> relative_position;
    relative_position.push_back(local_target_position_[0] - dji_interface_->current_local_position_.point.x);
    relative_position.push_back(local_target_position_[1] - dji_interface_->current_local_position_.point.y);
    relative_position.push_back(local_target_position_[2] - dji_interface_->current_local_position_.point.z);
    std::vector<double> cmd_vel;
    for (int i = 0; i < 3; ++i)
      cmd_vel.push_back(relative_position[i] * pos_p_gains_);
    if (command_mode_ == POS_VEL){
      for (int i = 0; i < 3; ++i)
        cmd_vel[i] += target_velocity_[i];
    }
    rangeCheck(cmd_vel[0], max_vel_xy_);
    rangeCheck(cmd_vel[1], max_vel_xy_);
    rangeCheck(cmd_vel[2], max_vel_z_);
    // yaw
    double relative_yaw = target_yaw_ - (dji_interface_->toEulerAngle(dji_interface_->current_atti_)).z;
    while(relative_yaw > M_PI)
      relative_yaw -= 2.0 * M_PI;
    while(relative_yaw < -M_PI)
      relative_yaw += 2.0 * M_PI;
    double cmd_yaw_vel = relative_yaw * yaw_p_gains_;
    setVelocityTarget(cmd_vel[0], cmd_vel[1], cmd_vel[2], cmd_yaw_vel);

    // task finish flag
    bool in_bound_flag = true;
    for (int i = 0; i < 3; ++i){
      if (fabs(relative_position[i]) > pos_task_pos_thre_)
        in_bound_flag = false;
    }
    if (fabs(relative_yaw) > pos_task_yaw_thre_)
      in_bound_flag = false;
    if (in_bound_flag)
      ++pos_task_in_bound_cnt_;
    if (pos_task_in_bound_cnt_ > 50){
      pos_task_in_bound_cnt_ = 0;
      dji_interface_->velocityControl(0, 0, 0, 0);
      motion_state_ = NO_TASK;
      ROS_INFO("Position task finished");
      ROS_INFO("current local state: %f, %f, %f, %f", dji_interface_->current_local_position_.point.x,
               dji_interface_->current_local_position_.point.y,
               dji_interface_->current_local_position_.point.z,
               (dji_interface_->toEulerAngle(dji_interface_->current_atti_)).z);
    }
  }

  bool DjiMotion::connect(){
    return dji_interface_->connectM100();
  }

  bool DjiMotion::takeOff(){
    bool connected = connect();
    if (connected){
      if (dji_interface_->M100TakeOff())
        return true;
      else
        return false;}
    else
      return false;
  }

  void DjiMotion::setRelativeLocalTarget(double dx, double dy, double dz, double yaw){
    ROS_INFO("setRelativeLocalTarget: %f, %f, %f, %f", dx, dy, dz, yaw);
    ROS_INFO("current local state: %f, %f, %f, %f", dji_interface_->current_local_position_.point.x,
             dji_interface_->current_local_position_.point.y,
             dji_interface_->current_local_position_.point.z,
             (dji_interface_->toEulerAngle(dji_interface_->current_atti_)).z);
    setLocalTarget(dji_interface_->current_local_position_.point.x + dx,
                   dji_interface_->current_local_position_.point.y + dy,
                   dji_interface_->current_local_position_.point.z + dz,
                   yaw);
  }

  void DjiMotion::setRelativeLocalTarget(double dx, double dy, double dz){
    ROS_INFO("setRelativeLocalTarget: %f, %f, %f", dx, dy, dz);
    ROS_INFO("current local state: %f, %f, %f", dji_interface_->current_local_position_.point.x,
             dji_interface_->current_local_position_.point.y,
             dji_interface_->current_local_position_.point.z);
    setLocalTarget(dji_interface_->current_local_position_.point.x + dx,
                   dji_interface_->current_local_position_.point.y + dy,
                   dji_interface_->current_local_position_.point.z + dz);
  }

  void DjiMotion::setGpsTarget(sensor_msgs::NavSatFix& target, double yaw){
    geometry_msgs::Vector3  deltaNed;
    dji_interface_->localOffsetFromGpsOffset(deltaNed, target, dji_interface_->current_gps_position_);
    setRelativeLocalTarget(deltaNed.x, deltaNed.y, deltaNed.z, yaw);
  }

  void DjiMotion::setGpsTarget(sensor_msgs::NavSatFix& target){
    geometry_msgs::Vector3  deltaNed;
    dji_interface_->localOffsetFromGpsOffset(deltaNed, target, dji_interface_->current_gps_position_);
    setRelativeLocalTarget(deltaNed.x, deltaNed.y, deltaNed.z);
  }

  void DjiMotion::setLocalTarget(double x, double y, double z){
    setLocalTarget(x, y, z, (dji_interface_->toEulerAngle(dji_interface_->current_atti_)).z);
  }

  void DjiMotion::setLocalTarget(double x, double y, double z, double yaw){
    // ROS_INFO("setLocalTarget: %f, %f, %f, %f", x, y, z, yaw);
    local_target_position_[0] = x;
    local_target_position_[1] = y;
    local_target_position_[2] = z;
    target_yaw_ = yaw;

    // todo
    motion_state_ = TRACKING;
  }

  void DjiMotion::setVelocityTarget(double vx, double vy, double vz, double vyaw){
    dji_interface_->velocityControl(vx, vy, vz, vyaw);
  }

  void DjiMotion::checkVelocityLimits(Eigen::Vector3d& cmd_vel){
    rangeCheck(cmd_vel(0), max_vel_xy_);
    rangeCheck(cmd_vel(1), max_vel_xy_);
    rangeCheck(cmd_vel(2), max_vel_z_);
  }

  void DjiMotion::rangeCheck(double& value, double max_val){
    if (value > 0 && value > max_val)
      value = max_val;
    else if (value < 0 && value < -max_val)
      value = -max_val;
  }

  void DjiMotion::setMaxVelocity(double max_vel_xy, double max_vel_z, double max_vel_yaw){
    max_vel_xy_ = max_vel_xy;
    max_vel_z_ = max_vel_z;
    max_vel_yaw_ = max_vel_yaw;
  }

  Eigen::Vector3d DjiMotion::getLocalPosition(){
    return Eigen::Vector3d(dji_interface_->current_local_position_.point.x,
                           dji_interface_->current_local_position_.point.y,
                           dji_interface_->current_local_position_.point.z);
  }

  Eigen::Vector3d DjiMotion::getGpsPosition(){
    return Eigen::Vector3d(dji_interface_->current_gps_position_.latitude,
                           dji_interface_->current_gps_position_.longitude,
                           dji_interface_->current_gps_position_.altitude);
  }

}
