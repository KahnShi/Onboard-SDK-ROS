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

#include <dji_sdk_jsk/DjiMotion.h>

using namespace dji_interface;
namespace mbzirc_motion{
#define NO_MOTION 0
#define LINE_MOTION 1
#define EIGHT_MOTION 2
#define CLOCK_WISE 1
#define COUNTER_CLOCK -1

  class MbzircMotion{
  public:
    MbzircMotion(ros::NodeHandle nh, ros::NodeHandle nhp);
    void setMotionType(int type);

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    int motion_type_;
    double line_motion_vel_x_;
    double line_motion_vel_y_;
    bool eight_motion_init_flag_;
    double eight_motion_vel_;
    Eigen::Vector3d eight_motion_center_position_;
    Eigen::Vector3d eight_motion_left_down_corner_position_;
    Eigen::Vector3d eight_motion_right_down_corner_position_;
    Eigen::Vector3d eight_motion_left_circle_center_position_;
    Eigen::Vector3d eight_motion_right_circle_center_position_;
    Eigen::VectorXd eight_motion_time_checkpoint_;
    double eight_motion_time_cnt_;
    double eight_motion_time_gap_;
    double eight_motion_radius_;
    double eight_motion_cross_ang_;
    double eight_motion_ang_vel_;
    int eight_motion_route_id_;
    Eigen::Vector3d eight_motion_control_I_term_;
    double eight_motion_p_gain_;
    double eight_motion_i_gain_;

    ros::Timer control_timer_;
    double control_timer_freq_;

    DjiMotion* dji_motion_;

    ros::Subscriber motion_type_sub_;

    void initCircleMotion();
    void eightMotionStraightPart(Eigen::Vector3d start_pt, double direction, Eigen::Vector3d& cur_pose, Eigen::Vector3d& cur_vel);
    void eightMotionCirclePart(Eigen::Vector3d circle_center, double rotation, double start_ang, Eigen::Vector3d& cur_pose, Eigen::Vector3d& cur_vel);
    void motionTypeCallback(const std_msgs::UInt8::ConstPtr& msg);
    void controlTimercallback(const ros::TimerEvent&);
  };
}
