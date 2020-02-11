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

#ifndef LINE_TRACER_H
#define LINE_TRACER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

using namespace std;

namespace ros_dmp_tools
{

class LineTracer {

protected:

  string topic_name_;

  ros::Publisher vis_pub_;
  ros::Subscriber sub_;

  int marker_id_;
  geometry_msgs::Point last_point_;

  std_msgs::ColorRGBA color_;
  float width_;
  float duration_;

public:

  LineTracer(string topic_name);
  ~LineTracer();

  void setColor(float r, float g, float b, float a=0.6);
  void setWidth(float w);
  void setDuration(float d);

  void start(ros::NodeHandle &nh);

  void publishMarker(const geometry_msgs::Point &current_point);
  void publishMarkerArray(const vector<geometry_msgs::Point> &current_point);
  void cleanMarkers();


};


}// end namespace ros_dmp_tools

#endif // LINE_TRACER_H
