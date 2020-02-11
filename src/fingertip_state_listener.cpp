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

#include "dmp_tools/fingertip_state_listener.h"

using namespace std;
namespace ros_dmp_tools
{
  /*********************************************************************
  * StateListener implementation for fingertip cartesian states
  * StateListener is a part of dmp_tools package
  * This node listens to fingertip states published by another node
  * and feeds it to trajectory recorder of ros dmp_tools as RecordState msgs.
  *********************************************************************/
  FingertipStateListener::FingertipStateListener(ros::NodeHandle* nh):
  StateListener(nh)
  {

  }
  /*********************************************************************
  * Comment
  *********************************************************************/
  FingertipStateListener::~FingertipStateListener()
  {
      // k_solver_ is an external resource, hence not deleted
  }
  /*********************************************************************
  * Comment
  *********************************************************************/
  void FingertipStateListener::startListening()
  {
    // Subscribe to joint state topic
    sub_ = nh_->subscribe(FINGER_POSE_TOPIC, 1, // queue size
                                &FingertipStateListener::subscriberCallback, this);
  }
  /*********************************************************************
  * Comment
  *********************************************************************/
  void FingertipStateListener::subscriberCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
  {
    // create a double vector as RecordState data
    vector<double> rs;

    // cart_frames contains a Transform message per each finger
    // currently we only append the positions to our RecordState
    int finger_count = msg->poses.size();
    for(int fi=0; fi < finger_count; fi++){
      rs.push_back(msg->poses[fi].position.x);
      rs.push_back(msg->poses[fi].position.y);
      rs.push_back(msg->poses[fi].position.z);
    }

    // publish state for trajectory recorder
    publishRecordState(rs);
  }

}
