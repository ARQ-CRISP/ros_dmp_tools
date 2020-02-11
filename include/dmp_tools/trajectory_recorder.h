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


#ifndef TRAJECTORY_RECORDER_H_
#define TRAJECTORY_RECORDER_H_

#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <dmp/Trajectory.hpp>

#include <dmp_tools/RecordState.h>
#include <dmp_tools/joint_state_listener.h>

namespace ros_dmp_tools{

class TrajectoryRecorder{

protected:
	ros::NodeHandle* node_handle_;
	ros::Subscriber record_state_sub_;

  // ROS Time
  ros::Time tstart_;
  ros::Time tupdate_;
  double duration_; // in sec

	int stepCount_;
	int step_;

	std::string filename_;

	// Trajectory data
  Eigen::VectorXd ts_;
  Eigen::MatrixXd ys_;
  Eigen::MatrixXd yds_;
  Eigen::MatrixXd ydds_;
	int dimCount_;

public:

	TrajectoryRecorder(ros::NodeHandle* node_handle, const std::string filename="traj.txt"):node_handle_(node_handle), filename_(filename){}
	// TODO: Joint set selection, default is all published joints
	void startRecording(const int dimCount, const double duration, const int stepCount);
	void createInitialState(const std::vector<double>& state);
  void recordStateCallback(const dmp_tools::RecordState& state);
	void endRecording();

};
} // end namespace ros_dmp_tools
#endif /* TRAJECTORY_RECORDER_H_*/
