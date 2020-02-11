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


#ifndef STATE_LISTENER_H_
#define STATE_LISTENER_H_

#include <ros/ros.h>
#include <dmp_tools/RecordState.h>

namespace ros_dmp_tools{
const std::string RECORD_STATE_TOPIC = "record_state";

// base class for state listeners
// that can be inherited with different message type listeners
class StateListener{

protected:
  ros::NodeHandle* nh_;

  ros::Subscriber sub_;
  ros::Publisher pub_;

  void publishRecordState(const std::vector<double>& state);

public:
	StateListener(ros::NodeHandle* nh);
  ~StateListener();

  virtual void startListening() = 0;
};
} // end namespace ros_dmp_tools


#endif /* STATE_LISTENER_H_*/
