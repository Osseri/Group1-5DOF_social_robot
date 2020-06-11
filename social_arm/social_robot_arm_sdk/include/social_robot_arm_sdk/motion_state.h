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

#ifndef MOTION_STATE_H
#define MOTION_STATE_H

#include <eigen3/Eigen/Eigen>
#include "robotis_math/robotis_math.h"

namespace social_robot_arm
{

class MotionState
{
public:
  MotionState(int num_of_joint, int num_of_via = 0);
  ~MotionState();

  bool is_moving_;
  bool is_generating_;

  int cnt_;           // step counter

  double mov_time_;   // movement time
  double smp_time_;   // sampling time

  int all_time_steps_;

  Eigen::MatrixXd calc_joint_tra_;  // calculated joint trajectory

  Eigen::MatrixXd joint_cur_pose_;
  Eigen::MatrixXd joint_tar_pose_;

  int             num_of_joint_;
  int             num_of_via_;
  Eigen::MatrixXd via_time_;

  Eigen::MatrixXd joint_via_pose_;
  Eigen::MatrixXd joint_via_dpose_;
  Eigen::MatrixXd joint_via_ddpose_;

};

}


#endif // MOTION_STATE_H
