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


#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#include "dynamicalsystems/SpringDamperSystem.hpp"

#include "dmp/serialization.hpp"
#include "dmp/Dmp.hpp"
#include "dmp/Trajectory.hpp"

#include "functionapproximators/FunctionApproximatorLWR.hpp"
#include "functionapproximators/MetaParametersLWR.hpp"

#include "dmpbbo_io/EigenFileIO.hpp"

#include <ros/ros.h>

using namespace std;
using namespace Eigen;
using namespace DmpBbo;

namespace fs = boost::filesystem;

// DMP params
fs::path traj_path_;
fs::path dmp_path_;
int n_basis_functions_;
double intersection_;
Dmp::DmpType dmp_type_ = Dmp::KULVICIUS_2012_JOINING;
Dmp::ForcingTermScaling scaling_type_ = Dmp::NO_SCALING;
bool save_details_ = true;

bool getParams();

int main(int argc, char **argv) {

  ros::init(argc, argv, "train_dmp");

  ROS_INFO( "DMP trainer node" );
  ros::NodeHandle nh;

  if(!getParams()) return -1;

  cout << "DMP trainer: Reading trajectory " << traj_path_.filename() << endl;
  Trajectory trajectory = Trajectory::readFromFile(traj_path_.string());

  VectorXd ts = trajectory.ts(); // Time steps
  int n_dims = trajectory.dim();

  // Create a prototype LWR function approximator
  // We have to use C++ pointers because the DMP constructor asks for it
  int input_dim = 1;
  MetaParametersLWR* meta_parameters =
    new MetaParametersLWR(input_dim, n_basis_functions_, intersection_);
  FunctionApproximatorLWR* fa_lwr =
    new FunctionApproximatorLWR(meta_parameters);

  // Clone the function approximator for each dimension of the DMP
  vector<FunctionApproximator*> function_approximators(n_dims);
  for (int di=0; di<n_dims; di++)
    function_approximators[di] = fa_lwr->clone();

  // Initialize the DMP
  Dmp* dmp = new Dmp(n_dims, function_approximators,
                    dmp_type_, scaling_type_);

  cout << "Training Dmp..." << endl;
  dmp->train(trajectory);

  // TODO: auto increment DMP names if exists
  cout << "DMP trainer: Writing trained Dmp " << dmp_path_.filename() << endl;

  std::ofstream ofs(dmp_path_.string());
  boost::archive::xml_oarchive oa(ofs);
  oa << boost::serialization::make_nvp("dmp",dmp);
  ofs.close();

  // save details of the dmp for analyzing if opted
  if(save_details_){
    // Integrate to get useful info
    MatrixXd xs_ana, xds_ana, forcing_terms_ana, fa_output_ana;
    dmp->analyticalSolution(ts,xs_ana,xds_ana,forcing_terms_ana,fa_output_ana);

    MatrixXd output_ana(ts.size(),1+xs_ana.cols()+xds_ana.cols());
    output_ana << xs_ana, xds_ana, ts;

    string file_stem = dmp_path_.stem().string();
    saveMatrix(".",file_stem+"_xs_xds.txt",output_ana,true);
    saveMatrix(".",file_stem+"_forcing_terms.txt",forcing_terms_ana,true);
    saveMatrix(".",file_stem+"_fa_output.txt",fa_output_ana,true);

    // TODO: plot dmp details (using python)
  }

  // clear pointers
  for(auto fa : function_approximators)
    delete fa;
  function_approximators.clear();
  delete meta_parameters;
  delete fa_lwr;
  delete dmp;

  ROS_INFO("DMP trainer: finished.");
  return 0;
}

bool getParams(){

  string dmp_filename, traj_filename;

  if(!ros::param::get("~dmp_name", dmp_filename)){
    ROS_WARN("DMP trainer: Can't get dmp_name param.");
    // default value
    dmp_filename = "dmp.xml";
  }
  dmp_path_ = dmp_filename;
  ROS_DEBUG("DMP trainer: DMP file is %s.", dmp_path_.filename().c_str());

  if(!ros::param::get("~trajectory_name", traj_filename)){
    ROS_WARN("DMP trainer: Can't get trajectory_name param.");
    // default value
    traj_filename = "traj.txt";
  }
  traj_path_ = traj_filename;
  ROS_DEBUG("DMP trainer: Trajectory file is %s.", traj_path_.filename().c_str());

  int dmp_type;
  if(!ros::param::get("~dmp_type", dmp_type)){
    ROS_WARN("DMP trainer: Can't get dmp_type param.");
  }else
    dmp_type_ = static_cast<Dmp::DmpType>(dmp_type);
  ROS_DEBUG("DMP trainer: dmp_type is %d.", dmp_type);

  int scaling_type;
  if(!ros::param::get("~scaling_type", scaling_type)){
    ROS_WARN("DMP trainer: Can't get scaling_type param.");
  }else
    scaling_type_ = static_cast<Dmp::ForcingTermScaling>(scaling_type);
  ROS_DEBUG("DMP trainer: scaling_type is %d.", scaling_type);

  if(!ros::param::get("~basis_count", n_basis_functions_)){
    ROS_WARN("DMP trainer: Can't get basis_count param.");
    // default value
		n_basis_functions_ = 30;
  }
  ROS_DEBUG("DMP trainer: basis_count is %d.", n_basis_functions_);

  if(!ros::param::get("~intersection", intersection_)){
    ROS_WARN("DMP trainer: Can't get intersection param.");
    // default value
		intersection_ = 0.56;
  }
  ROS_DEBUG("DMP trainer: intersection is %.3f.", intersection_);

  ros::param::get("~save_details", save_details_);
  ROS_DEBUG_STREAM("DMP trainer: saving details?" << save_details_);

  return true;

}
