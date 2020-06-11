/*******************************************************************************
 * Copyright (c) 2018, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "social_robot_arm_sdk/motion_state.h"

using namespace social_robot_arm;

MotionState::MotionState(int num_of_joint, int num_of_via)
  : num_of_joint_(num_of_joint),
    num_of_via_(num_of_via)
{
  is_moving_      = false;
  is_generating_  = false;

  cnt_            = 0;

  mov_time_       = 1.0;
  smp_time_       = 0.008;
  all_time_steps_ = int(mov_time_ / smp_time_) + 1;

  calc_joint_tra_ = Eigen::MatrixXd::Zero(all_time_steps_, num_of_joint_ + 1);

  joint_via_pose_   = Eigen::MatrixXd::Zero(num_of_via_, num_of_joint_ + 1);
  joint_via_dpose_  = Eigen::MatrixXd::Zero(num_of_via_, num_of_joint_ + 1);
  joint_via_ddpose_ = Eigen::MatrixXd::Zero(num_of_via_, num_of_joint_ + 1);

  via_time_         = Eigen::MatrixXd::Zero(num_of_via_, 1);

  joint_cur_pose_ = Eigen::MatrixXd::Zero(num_of_joint, 1);
  joint_tar_pose_ = Eigen::MatrixXd::Zero(num_of_joint, 1);
}

MotionState::~MotionState()
{

}
