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


#ifndef DMP_LOADER_H_
#define DMP_LOADER_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <dmp_tools/trajectory_conversion.h>

#include <dmp/Dmp.hpp>
#include <dmp/Trajectory.hpp>

#include <dmpbbo_io/EigenFileIO.hpp>


#include <eigen3/Eigen/Core>

using namespace std;
using namespace DmpBbo;

namespace ros_dmp_tools{

/**
* Loads the dmp using boost serialization, using the DmpBbo library,
* creates trajectory in std and KDL formats, for external use.
**/
class DmpLoader{

protected:

  // ROS Time
  Dmp * dmp_;

  Trajectory last_traj_;

public:

	DmpLoader(const string& dmp_filename);
  ~DmpLoader();

  void setGoalState(const vector<double> goal_state);
  void setInitialState(const vector<double> initial_state);

  vector<double> getGoalState() const;
  vector<double> getInitialState() const;

  void createTrajectory(const int time_step_count, const double tau, Trajectory& traj);
	void createTrajectory(const int time_step_count, const double tau, vector<double>& ts,
        vector<vector<double> >& qs, vector<vector<double> >& qds, vector<vector<double> >& qdds);
  inline void createTrajectory(const int time_step_count, const double tau, kdl_control_tools::JointTrajectory& traj){
    createTrajectory(time_step_count, tau, last_traj_);
    toKDL(last_traj_, traj);
  }
  inline void createTrajectory(const int time_step_count, const double tau, kdl_control_tools::CartesianTrajectory& traj){
    createTrajectory(time_step_count, tau, last_traj_);
    toKDL(last_traj_, traj);
  }

  void saveLastTrajectory();
};
} // end namespace ros_dmp_tools

#endif /* DMP_LOADER_H_*/
