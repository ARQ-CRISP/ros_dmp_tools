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

#include "dmp_tools/line_tracer.h"


using namespace ros_dmp_tools;
/*********************************************************************
* Comment
*********************************************************************/
LineTracer::LineTracer(string topic_name)
{
  topic_name_ = topic_name;

  marker_id_= 0;

  setColor(0.4, 1.0, 0.4, 0.6);
  setWidth(0.002);
  setDuration(4.0);
}
/*********************************************************************
* Comment
*********************************************************************/
LineTracer::~LineTracer()
{

}
/*********************************************************************
* Starts the publisher
*********************************************************************/
void LineTracer::start(ros::NodeHandle &nh)
{
  // create the visualization_msgs publisher for markers
  vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic_name_+"_pub", 0);

}
/*********************************************************************
* Publish a line marker between the last point and current point.
* Marker parameters can be adjusted with class setter methods.
*********************************************************************/
void LineTracer::publishMarker(const geometry_msgs::Point &current_point){

    if(marker_id_ != 0){

      visualization_msgs::MarkerArray marker_arr;

      visualization_msgs::Marker marker;
      marker.header.frame_id = "hand_root";
      marker.header.stamp = ros::Time();
      marker.ns = topic_name_+"_pub";

      marker.id = marker_id_;

      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;

      marker.scale.x = width_;
      marker.scale.y = width_;
      marker.scale.z = 0.000;
      marker.color = color_;

      // create a line segment
      marker.points.push_back(last_point_);
      marker.points.push_back(current_point);

      marker.lifetime = ros::Duration(duration_);
      marker.frame_locked = true;

      marker_arr.markers.push_back(marker);
      vis_pub_.publish( marker_arr );
  }

  marker_id_++;
  // remember last point
  last_point_ = current_point;
}
/*********************************************************************
* Publish a line marker between the points of an array.
* Marker parameters can be adjusted with class setter methods.
*********************************************************************/
void LineTracer::publishMarkerArray(const vector<geometry_msgs::Point> &point_arr){

  // message to be published
  visualization_msgs::MarkerArray marker_arr;

  // create the common part of the message
  visualization_msgs::Marker marker;
  marker.header.frame_id = "hand_root";
  marker.header.stamp = ros::Time();
  marker.ns = topic_name_+"_pub";

  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = width_;
  marker.scale.y = width_;
  marker.scale.z = 0.000;
  marker.color = color_;

  marker.lifetime = ros::Duration(duration_);
  marker.frame_locked = true;

  // loop over the points and complete messages
  int vec_len = point_arr.size();
  for(int pi = 1; pi < vec_len; pi++){

    if(marker_id_ != 0){

      marker.id = marker_id_;

      // create a line segment
      marker.points.push_back(last_point_);
      marker.points.push_back(point_arr[pi]);

      marker_arr.markers.push_back(marker);
    }

    marker_id_++;
    // remember last point
    last_point_ = point_arr[pi];
  }
  // publish it
  vis_pub_.publish( marker_arr );
}
/*********************************************************************
* Deletes all the markers created by this object.
*********************************************************************/
void LineTracer::cleanMarkers(){
  // create the common part of the message
  visualization_msgs::Marker marker;
  marker.header.frame_id = "hand_root";
  marker.header.stamp = ros::Time();
  marker.ns = topic_name_ + "_pub";

  marker.action = visualization_msgs::Marker::DELETE;

  // retrack marker_id_'s
  while (marker_id_>0) {
    // modify message and send
    marker.id = marker_id_;
    vis_pub_.publish( marker );

    // retrack id
    marker_id_--;
  }
}
/*********************************************************************
* Setters
*********************************************************************/
void LineTracer::setColor(float r, float g, float b, float a){
  color_.r = r;
  color_.g = g;
  color_.b = b;
  color_.a = a;
}
void LineTracer::setWidth(float w){
  width_ = w;
}
// Zero for persistent markers
void LineTracer::setDuration(float d){
  duration_ = d;
}
