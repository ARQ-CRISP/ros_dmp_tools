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


#ifndef RECORD_CONVERSION_H_
#define RECORD_CONVERSION_H_

#include <ros/ros.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames.hpp>

using namespace std;

namespace ros_dmp_tools{

  void toKDL(const vector<double>& record, KDL::Frame& frame);
  void toKDL(const Eigen::VectorXd& record, KDL::Frame& frame);

  void toRecordState(const KDL::Frame& frame, vector<double>& record);

} // end namespace ros_dmp_tools

#endif /* RECORD_CONVERSION_H_*/
