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

#include <dji_sdk_jsk/MbzircBalloonPiercingMotion.h>

namespace mbzirc_balloon_piercing_motion {
  MbzircBalloonPiercingMotion::MbzircBalloonPiercingMotion(ros::NodeHandle nh, ros::NodeHandle nhp)
    : waypoint_list_sp_(0), is_task_start_(false), is_go_home_(false), nh_(nh), nhp_(nhp)
  {
    nhp_.getParam("area_corner_lati_list", area_corner_lati_list_);
    nhp_.getParam("area_corner_long_list", area_corner_long_list_);
    nhp_.getParam("relative_alti_list", relative_alti_list_);
    nhp_.getParam("trip_number", trip_number_);
    nhp_.getParam("takeoff_wait_time", takeoff_wait_time_);
    nhp_.getParam("return_alt", return_alt_);

    createWayPointList();

    motion_type_sub_ = nh_.subscribe("/dji_jsk/task_start", 10, &MbzircBalloonPiercingMotion::taskStartCallback, this, ros::TransportHints().tcpNoDelay());

    dji_motion_ = new DjiMotion(nh_, nhp_);

    // if (dji_motion_->takeOff())
    if (!dji_motion_->connect())
      throw std::runtime_error("[MbzircBalloonPiercingMotion] M100 NOT connected");
    else{
      ROS_INFO("[MbzircBalloonPiercingMotion] M100 is connected");
    }

    nhp_.param("control_timer_freq", control_timer_freq_, 0.01);

    control_timer_ = nh.createTimer(ros::Duration(0.02), &MbzircBalloonPiercingMotion::controlTimercallback, this);
  }

  void MbzircBalloonPiercingMotion::controlTimercallback(const ros::TimerEvent&){
    if (!is_task_start_)
      return;

    if (dji_motion_->motion_state_ == NO_TASK && !is_go_home_){
      ROS_INFO_STREAM( "[MbzircBalloonPiercingMotion] Waypoint motion " << waypoint_list_sp_+1 << "/" << waypoint_list_.size() << " started.");

      sensor_msgs::NavSatFix target_gps;
      target_gps.latitude = waypoint_list_[waypoint_list_sp_][0];
      target_gps.longitude = waypoint_list_[waypoint_list_sp_][1];
      target_gps.altitude = waypoint_list_[waypoint_list_sp_][2];// - dji_motion_->getLocalPosition()[2] + dji_motion_->getGpsPosition()[2];
      dji_motion_->setGpsTarget(target_gps);

      waypoint_list_sp_++;
      if (waypoint_list_sp_>waypoint_list_.size()-1) {
        is_go_home_ = true;
        waypoint_list_sp_ = 0;
        ROS_INFO("[MbzircBalloonPiercingMotion] task finished");
      }
      return;
    }

    if (dji_motion_->motion_state_ == NO_TASK && is_go_home_){
      ROS_INFO("[MbzircBalloonPiercingMotion] returning to initial position");
      sensor_msgs::NavSatFix target_gps;
      target_gps.latitude = initial_gps_position_[0];
      target_gps.longitude = initial_gps_position_[1];
      target_gps.altitude = return_alt_;
      dji_motion_->setGpsTarget(target_gps);
      is_task_start_ = false;
      is_go_home_ = false;
      return;
    }
    return;
  }

  void MbzircBalloonPiercingMotion::taskStartCallback(const std_msgs::Empty::ConstPtr& msg){
    initial_gps_position_ = dji_motion_->getGpsPosition();
    if (!dji_motion_->takeOff()) {
      is_task_start_ = false;
      ROS_INFO("[MbzircBalloonPiercingMotion] takeoff failed");
    }
    ros::Duration(takeoff_wait_time_).sleep();

    is_task_start_ = true;
    ROS_INFO("[MbzircBalloonPiercingMotion] Waypoint motion starts, please be very careful");
  }

  void MbzircBalloonPiercingMotion::createWayPointList() {
    if (area_corner_long_list_.size() != 4 || area_corner_lati_list_.size() != 4) {
      ROS_INFO_STREAM(area_corner_lati_list_.size()<<area_corner_long_list_.size());
      ROS_ERROR("[MbzircBalloonPiercingMotion] area_corner_{lati,long}_list_ should have exactly 4 elements");
      ros::shutdown();
    }

    for (const auto& alt : relative_alti_list_) {
      for (int i = 0; i < trip_number_; ++i) {
        std::vector<double> waypoint_tmp1;
        int int_number=trip_number_-1;
        waypoint_tmp1.push_back((area_corner_lati_list_[1]-area_corner_lati_list_[0])/int_number*i + area_corner_lati_list_[0]);
        waypoint_tmp1.push_back((area_corner_long_list_[1]-area_corner_long_list_[0])/int_number*i + area_corner_long_list_[0]);
        waypoint_tmp1.push_back(alt);
        waypoint_list_.push_back(waypoint_tmp1);

        std::vector<double> waypoint_tmp2;
        waypoint_tmp2.push_back((area_corner_lati_list_[2]-area_corner_lati_list_[3])/int_number*i + area_corner_lati_list_[3]);
        waypoint_tmp2.push_back((area_corner_long_list_[2]-area_corner_long_list_[3])/int_number*i + area_corner_long_list_[3]);
        waypoint_tmp2.push_back(alt);
        waypoint_list_.push_back(waypoint_tmp2);
      }
    }
  }

}

int main(int argc, char **argv) {
  ros::init (argc, argv, "mbzirc_balloon_piercing_motion_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  mbzirc_balloon_piercing_motion::MbzircBalloonPiercingMotion mbzirc_balloon_piercing_motion(nh, nh_private);
  ros::spin();
  return 0;
}
