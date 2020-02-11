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


#ifndef TRAJECTORY_CONVERSION_H_
#define TRAJECTORY_CONVERSION_H_

#include <eigen_conversions/eigen_kdl.h>
#include <dmp/Trajectory.hpp>
#include <kdl_control_tools/trajectory.h>

#include <dmp_tools/record_conversion.h>

using namespace std;

namespace ros_dmp_tools{

  // convert DmpBbo to KDL
    // joint traj
  void toKDL(const DmpBbo::Trajectory& traj_in, kdl_control_tools::JointTrajectory& traj_out);
  void toKDL(const DmpBbo::Trajectory& traj_in, kdl_control_tools::CartesianTrajectory& traj_out);

  // convert KDL to DmpBbo
  void toDmpBbo(const kdl_control_tools::JointTrajectory& traj_in, DmpBbo::Trajectory& traj_out);

} // end namespace ros_dmp_tools

#endif /* TRAJECTORY_CONVERSION_H_*/
