/**
* ros_dmp_tools contains tools for learning from demonstration using dynamical
* movement primitives (DMP) framework under robot operating system (ROS)
* Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
* USA
*/

#include <dmp_tools/trajectory_conversion.h>


namespace ros_dmp_tools
{

  void toKDL(const DmpBbo::Trajectory& traj_in, kdl_control_tools::JointTrajectory& traj_out) {
    // match lengths
    int len = traj_in.length();
    traj_out.setLength(len);

    // components of input trajectory
    const Eigen::VectorXd ts = traj_in.ts();
    const Eigen::MatrixXd ys = traj_in.ys();
    const Eigen::MatrixXd yds = traj_in.yds();
    const Eigen::MatrixXd ydds = traj_in.ydds();

    // convert each step
    for(int si=0; si < len; si++){
      KDL::JntArray tmp_arr;

      traj_out.setTime(si, ts(si));

      tmp_arr.data = ys.row(si);
      traj_out.setPoint(si, tmp_arr);

      tmp_arr.data = yds.row(si);
      traj_out.setVelocity(si, tmp_arr);

      tmp_arr.data = ydds.row(si);
      traj_out.setAcceleration(si, tmp_arr);
    }
  }

  void toKDL(const DmpBbo::Trajectory& traj_in, kdl_control_tools::CartesianTrajectory& traj_out) {
    // match lengths
    int len = traj_in.length();
    traj_out.setLength(len);

    // components of input trajectory
    const Eigen::VectorXd ts = traj_in.ts();
    const Eigen::MatrixXd ys = traj_in.ys();
    const Eigen::MatrixXd yds = traj_in.yds();
    const Eigen::MatrixXd ydds = traj_in.ydds();

    // convert each step
    for(int si=0; si < len; si++){
      KDL::Frame tmp_frame;

      // delta time except the first step
      double dt = si? ts(si) - ts(si-1) : 0;

      traj_out.setTime(si, ts(si));

      ros_dmp_tools::toKDL(ys.row(si), tmp_frame);
      traj_out.setPoint(si, tmp_frame);

      KDL::Twist tmp_twist;
      if(dt>0)
        tmp_twist = KDL::diff(traj_out.getPoint(si-1), tmp_frame, dt);
      traj_out.setVelocity(si, tmp_twist);

      KDL::Wrench tmp_wrench;
      if(dt>0){
        KDL::Twist prev_twist = traj_out.getVelocity(si-1);
        tmp_wrench.force = KDL::diff(prev_twist.vel, tmp_twist.vel, dt);
        tmp_wrench.torque = KDL::diff(prev_twist.rot, tmp_twist.rot, dt);
      }
      traj_out.setAcceleration(si, tmp_wrench);

    }
  }

  void toDmpBbo(const kdl_control_tools::JointTrajectory& traj_in, DmpBbo::Trajectory& traj_out) {
    // get sizes
    int steps = traj_in.getLength();
    int dimensions = traj_in.getPoint(0).rows();
    // create trajectory matrices
    Eigen::VectorXd ts(steps);
    Eigen::MatrixXd ys(steps, dimensions);
    Eigen::MatrixXd yds(steps, dimensions);
    Eigen::MatrixXd ydds(steps, dimensions);

    // copy the values
    for(int si=0; si<steps; si++){
      ts(si) = traj_in.getTime(si);
      ys.row(si) = traj_in.getPoint(si).data;
      yds.row(si) = traj_in.getVelocity(si).data;
      ydds.row(si) = traj_in.getAcceleration(si).data;
    }

    traj_out = DmpBbo::Trajectory(ts, ys, yds, ydds);
  }

}
