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

#include <dmp_tools/record_conversion.h>


namespace ros_dmp_tools
{

  void toKDL(const vector<double>& rs, KDL::Frame& frame){
    // frame position
    for(int di=0; di<3; di++)
      frame.p(di) = rs[di];

    // frame rotation
    // unit axes
    KDL::Vector r_x;
    for(int di = 0; di<3; di++)
      r_x(di) = rs[di+3];

    KDL::Vector r_z;
    for(int di = 0; di<3; di++)
      r_z(di) = rs[di+6];

    // normalize
    r_x = r_x / r_x.Norm();
    r_z = r_z / r_z.Norm();
    // calc r_y
    KDL::Vector r_y = r_z * r_x;

    // create rotation matrix
    frame.M = KDL::Rotation(r_x, r_y, r_z);
  }
  void toKDL(const Eigen::VectorXd& rs, KDL::Frame& frame){
    vector<double> rs_std;
    for(int i=0; i<rs.rows(); i++)
      rs_std.push_back(rs(i));

    toKDL(rs_std, frame);
  }

  void toRecordState(const KDL::Frame& frame, vector<double>& rs){
    rs.clear();
    // convert initial frame to recorded format
    // position
    for(int di=0; di<3; di++)
      rs.push_back(frame.p(di));

    // orientation
    // we record only the r_x and r_z vectors since r_y = r_z * r_x
    KDL::Vector r_x = frame.M.UnitX();
    for(int di=0; di<3; di++)
      rs.push_back(r_x(di));

    KDL::Vector r_z = frame.M.UnitZ();
    for(int di=0; di<3; di++)
      rs.push_back(r_z(di));
  }

}
