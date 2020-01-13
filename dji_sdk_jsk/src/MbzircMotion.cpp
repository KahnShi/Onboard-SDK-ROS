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

#include <dji_sdk_jsk/MbzircMotion.h>

namespace mbzirc_motion{
  MbzircMotion::MbzircMotion(ros::NodeHandle nh, ros::NodeHandle nhp){
    nh_ = nh;
    nhp_ = nhp;

    motion_type_ = NO_MOTION;

    nhp_.param("line_motion_vel_x", line_motion_vel_x_, 5.0);
    nhp_.param("line_motion_vel_y", line_motion_vel_y_, 0.0);
    nhp_.param("eight_motion_vel", eight_motion_vel_, 5.0);
    nhp_.param("eight_motion_radius", eight_motion_radius_, 16.0);
    double eight_motion_cross_ang_deg;
    nhp_.param("eight_motion_cross_ang_deg", eight_motion_cross_ang_deg, 45.0);
    eight_motion_cross_ang_ = eight_motion_cross_ang_deg / 180.0 * M_PI;
    nhp_.param("eight_motion_p_gain", eight_motion_p_gain_, 0.4);
    nhp_.param("eight_motion_i_gain", eight_motion_i_gain_, 0.1);

    motion_type_sub_ = nh_.subscribe("/dji_jsk/motion_type", 10, &MbzircMotion::motionTypeCallback, this, ros::TransportHints().tcpNoDelay());

    dji_motion_ = new DjiMotion(nh_, nhp_);

    // if (dji_motion_->takeOff())
    if (!dji_motion_->connect())
      throw std::runtime_error("[MbzircMotion] M100 NOT connected");
    else{
      ROS_INFO("[MbzircMotion] M100 is connected");
    }

    nhp_.param("control_timer_freq", control_timer_freq_, 0.01);

    control_timer_ = nh.createTimer(ros::Duration(0.02), &MbzircMotion::controlTimercallback, this);
  }

  void MbzircMotion::controlTimercallback(const ros::TimerEvent&){
    if (motion_type_ == NO_MOTION)
      return;
    else if (motion_type_ == LINE_MOTION){
      dji_motion_->setVelocityTarget(line_motion_vel_x_, line_motion_vel_y_, 0, 0);
    }
    else if (motion_type_ == EIGHT_MOTION){
      //
      if (!eight_motion_init_flag_){
        initCircleMotion();
        eight_motion_init_flag_ = true;
      }

      if (eight_motion_time_cnt_ > eight_motion_time_checkpoint_[eight_motion_route_id_ + 1])
        eight_motion_route_id_ += 1;
      if (eight_motion_route_id_ >= eight_motion_time_checkpoint_.size() - 1){
        eight_motion_time_cnt_ -= eight_motion_time_checkpoint_[eight_motion_route_id_];
        eight_motion_route_id_ = 0;
      }
      Eigen::Vector3d target_pose, target_vel;
      if (eight_motion_route_id_ == 0)
        eightMotionStraightPart(eight_motion_left_down_corner_position_, eight_motion_cross_ang_, target_pose, target_vel);
      else if (eight_motion_route_id_ == 1)
        eightMotionCirclePart(eight_motion_right_circle_center_position_, COUNTER_CLOCK, M_PI / 2.0 + eight_motion_cross_ang_, target_pose, target_vel);
      else if (eight_motion_route_id_ == 2)
        eightMotionStraightPart(eight_motion_right_down_corner_position_, M_PI - eight_motion_cross_ang_, target_pose, target_vel);
      else if (eight_motion_route_id_ == 3)
        eightMotionCirclePart(eight_motion_left_circle_center_position_, CLOCK_WISE, M_PI / 2.0 - eight_motion_cross_ang_, target_pose, target_vel);
      eight_motion_time_cnt_ += eight_motion_time_gap_;

      // pos_vel mode is needed
      Eigen::Vector3d cur_pose = dji_motion_->getLocalPosition();
      eight_motion_control_I_term_ += (target_pose - cur_pose) * control_timer_freq_;
      Eigen::Vector3d vel = target_vel + (target_pose - cur_pose) * eight_motion_p_gain_
        + eight_motion_control_I_term_ * eight_motion_i_gain_;
      dji_motion_->checkVelocityLimits(vel);
      dji_motion_->setVelocityTarget(vel(0), vel(1), vel(2), 0);
    }

    /*
    // if set far waypoint
    dji_motion_->setMaxVelocity(3.0, 1.5, 0.3); // max_vel_xy, max_vel_z, nax_vel_yaw
    dji_motion_->setRelativeLocalTarget(10, 5, 5, M_PI);
    */
  }

  void MbzircMotion::motionTypeCallback(const std_msgs::UInt8::ConstPtr& msg){
    if (msg->data == LINE_MOTION){
      motion_type_ = LINE_MOTION;
      ROS_INFO("[MbzircMotion] Line motion starts, please stop manually");
    }
    else if (msg->data == EIGHT_MOTION){
      eight_motion_init_flag_ = false;
      motion_type_ = EIGHT_MOTION;
      ROS_INFO("[MbzircMotion] Eight-shape motion starts");
    }
    else{
      motion_type_ = NO_MOTION;
      dji_motion_->setVelocityTarget(0, 0, 0, 0);
      dji_motion_->motion_state_ = POS;
      ROS_ERROR("[MbzircMotion] Unknown motion type %d, velocity changes to 0", msg->data);
    }
  }

  void MbzircMotion::eightMotionStraightPart(Eigen::Vector3d start_pt, double direction, Eigen::Vector3d& cur_pose, Eigen::Vector3d& cur_vel){
    double time = eight_motion_time_cnt_ - eight_motion_time_checkpoint_[eight_motion_route_id_];
    cur_pose = Eigen::Vector3d(start_pt[0], start_pt[1], start_pt[2]);
    cur_vel = Eigen::Vector3d(eight_motion_vel_ * cos(direction),
                              eight_motion_vel_ * sin(direction),
                              0.0);
    cur_pose[0] += cur_vel[0] * time;
    cur_pose[1] += cur_vel[1] * time;
  }

  void MbzircMotion::eightMotionCirclePart(Eigen::Vector3d circle_center, double rotation, double start_ang, Eigen::Vector3d& cur_pose, Eigen::Vector3d& cur_vel){
    double time = eight_motion_time_cnt_ - eight_motion_time_checkpoint_[eight_motion_route_id_];
    double ang_vel = rotation * eight_motion_ang_vel_;
    double ang = start_ang + ang_vel * time;
    cur_pose = Eigen::Vector3d(circle_center[0], circle_center[1], circle_center[2]);
    cur_pose[0] += eight_motion_radius_ * cos(ang);
    cur_pose[1] += eight_motion_radius_ * sin(ang);

    cur_vel = Eigen::Vector3d(-eight_motion_radius_ * sin(ang) * ang_vel,
                              eight_motion_radius_ * cos(ang) * ang_vel,
                              0.0);
  }

  void MbzircMotion::initCircleMotion(){
    dji_motion_->setMaxVelocity(10.0, 10.0, 0.3);
    // dji_motion_->motion_state_ = POS_VEL;
    /* start from left_down_corner */
    eight_motion_center_position_ = dji_motion_->getLocalPosition()
      + Eigen::Vector3d(eight_motion_radius_ / tan(eight_motion_cross_ang_) * cos(eight_motion_cross_ang_),
                        eight_motion_radius_ / tan(eight_motion_cross_ang_) * sin(eight_motion_cross_ang_),
                        0.0);
    eight_motion_time_cnt_ = 0.0;
    eight_motion_time_gap_ = control_timer_freq_;

    eight_motion_ang_vel_ = eight_motion_vel_ / eight_motion_radius_;

    eight_motion_left_down_corner_position_ << -eight_motion_radius_ / tan(eight_motion_cross_ang_) * cos(eight_motion_cross_ang_),
      -eight_motion_radius_ / tan(eight_motion_cross_ang_) * sin(eight_motion_cross_ang_),
      0.0;
    eight_motion_left_down_corner_position_ += eight_motion_center_position_;

    eight_motion_right_down_corner_position_ << eight_motion_radius_ / tan(eight_motion_cross_ang_) * cos(eight_motion_cross_ang_),
      -eight_motion_radius_ / tan(eight_motion_cross_ang_) * sin(eight_motion_cross_ang_),
      0.0;
    eight_motion_right_down_corner_position_ += eight_motion_center_position_;

    eight_motion_left_circle_center_position_ << -eight_motion_radius_ / sin(eight_motion_cross_ang_), 0.0, 0.0;
    eight_motion_left_circle_center_position_ += eight_motion_center_position_;

    eight_motion_right_circle_center_position_ << eight_motion_radius_ / sin(eight_motion_cross_ang_), 0.0, 0.0;
    eight_motion_right_circle_center_position_ += eight_motion_center_position_;

    eight_motion_time_checkpoint_ = Eigen::VectorXd(5);
    eight_motion_time_checkpoint_ << 0.0,
      2 / tan(eight_motion_cross_ang_) * eight_motion_radius_ / eight_motion_vel_,
      (2 / tan(eight_motion_cross_ang_) + M_PI + 2.0 * eight_motion_cross_ang_) * eight_motion_radius_ / eight_motion_vel_,
      (4 / tan(eight_motion_cross_ang_) + M_PI + 2.0 * eight_motion_cross_ang_) * eight_motion_radius_ / eight_motion_vel_,
      (4 / tan(eight_motion_cross_ang_) + 2.0 * M_PI + 4.0 * eight_motion_cross_ang_) * eight_motion_radius_ / eight_motion_vel_;

    eight_motion_route_id_ = 0;
    eight_motion_control_I_term_ = Eigen::Vector3d::Zero();

    ROS_INFO("[MbzircMotion] circle radius: %f", eight_motion_radius_);
    ROS_INFO("[MbzircMotion] court size: %f, %f", 2 * (1 + 1.0 / sin(eight_motion_cross_ang_)) * eight_motion_radius_
             , 2 * eight_motion_radius_);
  }
}
