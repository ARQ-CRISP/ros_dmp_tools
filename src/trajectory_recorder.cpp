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

#include "dmp_tools/trajectory_recorder.h"
#include <math.h>

using namespace std;

namespace ros_dmp_tools{
/*********************************************************************
* Comment
*********************************************************************/
void TrajectoryRecorder::startRecording(const int dimCount, const double duration, const int stepCount){
	// Set parameters
	dimCount_ = dimCount;
	duration_ = duration;
	stepCount_ = stepCount;
	step_ = 0;

	ROS_INFO("Trajectory recorder: subscribing to %s", RECORD_STATE_TOPIC.c_str() );

	// Subscribe to joint state topic
	record_state_sub_ = node_handle_->subscribe(RECORD_STATE_TOPIC, 1, // queue size
																&TrajectoryRecorder::recordStateCallback, this);

}
/*********************************************************************
* Comment
*********************************************************************/
void TrajectoryRecorder::recordStateCallback(const dmp_tools::RecordState& state){

	// Safety check: dimension count
	if (state.data.size() != dimCount_){
		ROS_ERROR("Trajectory recorder: received data has unmatching size %d", (int)state.data.size());
		return;
	}

	// Create initial state
	if(step_ == 0){
		ROS_INFO("Trajectory recorder: createInitialState" );
		createInitialState(state.data);
		step_++;
		return;
	}

	// Check if the time step has come
  ros::Time tnow_ = ros::Time::now();
  double dt_start = (tnow_ - tstart_).sec + 1e-9 * (tnow_ - tstart_).nsec;

	// If it's time, then record it
	if(dt_start >= ts_[step_]){
		ROS_INFO("Trajectory recorder: step %d, dt %5.3f", step_, dt_start );
		// Get state
		vector<double> state_vec = state.data;
		ys_.row(step_) = Eigen::Map<Eigen::VectorXd>(state_vec.data(), dimCount_);

		// Calc derivatives (yd, ydd)
		double dt_update = (tnow_ - tupdate_).sec + 1e-9 * (tnow_ - tupdate_).nsec;
	  double step_size = dt_start - ts_[step_-1]; // since first step equals to step size

		ROS_INFO("Trajectory recorder: dt_update %5.3f", dt_update );
		ROS_INFO("Trajectory recorder: step_size %5.3f", step_size );

			// velocity
			yds_.row(step_) = (ys_.row(step_) - ys_.row(step_-1)) / step_size;
			// acceleration
			ydds_.row(step_) = (yds_.row(step_) - yds_.row(step_-1)) / step_size;

		// update time
		tupdate_ = ros::Time::now();

		// Track step
		step_++;
	}

	// If end of duration, then call termination procedure
	if (step_ >= stepCount_){
		endRecording();
		ros::shutdown();
	}

}
/*********************************************************************
* Comment
*********************************************************************/
void TrajectoryRecorder::createInitialState(const std::vector<double>& state) {

	// Create the time series
	ts_ = Eigen::VectorXd::LinSpaced(stepCount_,0,duration_); // Time steps
	// Shape state and derivative matrices
	ys_.resize(stepCount_, dimCount_);
	yds_.resize(stepCount_, dimCount_);
	ydds_.resize(stepCount_, dimCount_);

	// Convert from double[] to VectorXd
	vector<double> state_vec = state;
	ys_.row(0) = Eigen::Map<Eigen::VectorXd>(state_vec.data(), dimCount_);

	// Initial derivatives are zero
	yds_.row(0) = Eigen::VectorXd::Zero(dimCount_);
	ydds_.row(0) = Eigen::VectorXd::Zero(dimCount_);

	// Start ROS time
	tstart_ = ros::Time::now();
	tupdate_ = ros::Time::now();

}

/*********************************************************************
* Comment
*********************************************************************/
void TrajectoryRecorder::endRecording(){
	// Create a trajectory
	DmpBbo::Trajectory trajectory_ = DmpBbo::Trajectory(ts_, ys_, yds_, ydds_);
	trajectory_.saveToFile(".", filename_, true); 

  ROS_INFO("Trajectory recorder: ending" );
}
} // end namespace ros_dmp_tools

/*****************
* Test node
******************/

double test_duration = 3.0;
int test_step_count = 100;

using namespace ros_dmp_tools;

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_recorder");

  ros::NodeHandle node_handle;
	ros::start();

 	// countdown
  ros::Duration sleep_time(1.0);

	for (int ci = 3; ci > 0; ci--){
	  ROS_INFO( "Trajectory recorder: starting in %d",ci );
	  sleep_time.sleep();
	}

	ROS_INFO( "Trajectory recorder: started!" );

	ros_dmp_tools::TrajectoryRecorder trajectory_recorder(&node_handle);
	JointStateListener state_listener(&node_handle);

	state_listener.startListening();
	trajectory_recorder.startRecording(16, test_duration, test_step_count);

	ros::spin();

  return 0;
}
