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

#include "dmp_tools/state_listener.h"


namespace ros_dmp_tools{
/*********************************************************************
* Comment
*********************************************************************/
StateListener::StateListener(ros::NodeHandle* nh)
{
  nh_ = nh;

  // create ros communication nodes
  pub_ = nh_->advertise<dmp_tools::RecordState>(RECORD_STATE_TOPIC, 1);
}

StateListener::~StateListener()
{
}

/*********************************************************************
* Comment
*********************************************************************/
void StateListener::publishRecordState(const std::vector<double>& state)
{
  dmp_tools::RecordState msg;
  msg.data = state;
  pub_.publish(msg);
}


}
