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

#include "dmp_tools/joint_state_listener.h"


namespace ros_dmp_tools
{
  /*********************************************************************
  * Comment
  *********************************************************************/
  JointStateListener::JointStateListener(ros::NodeHandle* nh):
  StateListener(nh)
  {

  }

  JointStateListener::~JointStateListener()
  {

  }

  /*********************************************************************
  * Comment
  *********************************************************************/
  void JointStateListener::startListening()
  {
    // Subscribe to joint state topic
    sub_ = nh_->subscribe(JOINT_STATE_TOPIC, 1, // queue size
                                &JointStateListener::subscriberCallback, this);

  }

  /*********************************************************************
  * Comment
  *********************************************************************/
  void JointStateListener::subscriberCallback(const sensor_msgs::JointState &msg)
  {
    // for joint state listener it's straight-forward
    publishRecordState(msg.position);
  }


}
