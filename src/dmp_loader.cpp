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

#include "dmp_tools/dmp_loader.h"

#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/export.hpp>

#include <iostream>
#include <fstream>

// imported for deserialization
#include "dmp/serialization.hpp"
#include "dynamicalsystems/DynamicalSystem.hpp"
#include "dynamicalsystems/ExponentialSystem.hpp"
#include "dynamicalsystems/SigmoidSystem.hpp"
#include "dynamicalsystems/TimeSystem.hpp"
#include "dynamicalsystems/SpringDamperSystem.hpp"

#include "functionapproximators/FunctionApproximatorLWR.hpp"
#include "functionapproximators/MetaParametersLWR.hpp"
#include "functionapproximators/ModelParametersLWR.hpp"

using namespace std;
using namespace Eigen;
using namespace DmpBbo;


namespace ros_dmp_tools{
/*********************************************************************
* Comment
*********************************************************************/
DmpLoader::DmpLoader(const string& dmp_filename){

  {
    // create and open an archive for input
    ifstream ifs(dmp_filename.c_str());
    assert(ifs.good());
    boost::archive::xml_iarchive ia(ifs);
    // read class state from archive
    ia >> BOOST_SERIALIZATION_NVP(dmp_);
  } // brackets for auto-closing streams

  // to avoid seg fault in analyticalSolution
  dmp_->set_perturbation_analytical_solution(0.0);
}

DmpLoader::~DmpLoader(){
  delete dmp_;
}
/*********************************************************************
* Getters & setters
*********************************************************************/
vector<double> DmpLoader::getGoalState() const{

  Eigen::VectorXd s_eigen;
  s_eigen = dmp_->attractor_state();

  // convert to std vector
  vector<double> s_vec;
  int state_size = s_eigen.rows();
  for (int si = 0; si < state_size; si++)
    s_vec.push_back(s_eigen(si));

  return s_vec;
}
vector<double> DmpLoader::getInitialState() const{

  Eigen::VectorXd s_eigen;
  s_eigen = dmp_->initial_state();

  // convert to std vector
  vector<double> s_vec;
  int state_size = s_eigen.rows();
  for (int si = 0; si < state_size; si++)
    s_vec.push_back(s_eigen(si));

  return s_vec;
}
void DmpLoader::setGoalState(const vector<double> goal_state){

  // create eigen vector
  int state_size = goal_state.size();
  Eigen::VectorXd s_eigen(state_size);
  // copy
  for (int si = 0; si < state_size; si++)
    s_eigen[si] = goal_state[si];

  dmp_->set_attractor_state(s_eigen);

}
void DmpLoader::setInitialState(const vector<double> initial_state){

  // create eigen vector
  int state_size = initial_state.size();
  Eigen::VectorXd s_eigen(state_size);
  // copy std to eigen
  for (int si = 0; si < state_size; si++)
    s_eigen[si] = initial_state[si];

  dmp_->set_initial_state(s_eigen);

}
/*********************************************************************
* Produce trajectory by integrating dmp
*********************************************************************/
void DmpLoader::createTrajectory(const int time_step_count, const double tau, Trajectory& traj){

  // create time series
  VectorXd ts_eigen = VectorXd::LinSpaced(time_step_count,0,tau); // Time steps

  // apply time scale and get solution
  dmp_->set_tau(tau);
  dmp_->analyticalSolution(ts_eigen, last_traj_);
}
void DmpLoader::createTrajectory(const int time_step_count, const double tau, vector<double>& ts,  vector<vector<double> >& qs, vector<vector<double> >& qds, vector<vector<double> >& qdds){

  createTrajectory(time_step_count, tau, last_traj_);

  // convert trajectory into std vectors
  ts.resize(time_step_count);
  qs.resize(time_step_count);
  qds.resize(time_step_count);
  qdds.resize(time_step_count);

  int dim_count = last_traj_.ys().cols();

  for( int ti=0; ti<time_step_count; ti++){
    ts[ti] = last_traj_.ts()(ti);

    // resize rows
    qs[ti].resize(dim_count);
    qds[ti].resize(dim_count);
    qdds[ti].resize(dim_count);
    // assign
    for( int di=0; di < dim_count; di++){
      qs[ti][di] = last_traj_.ys()(ti,di);
      qds[ti][di] = last_traj_.yds()(ti,di);
      qdds[ti][di] = last_traj_.ydds()(ti,di);
    }
  }

}
/*********************************************************************
* Writes the last trajectory to file
*********************************************************************/
void DmpLoader::saveLastTrajectory(){
  // Create a trajectory
  last_traj_.saveToFile(".","traj_last.txt",true);
}

/*********************************************************************
* Draws the trajectory in RViz.
* RViz should have the corresponding marker listener.
*********************************************************************/
// void DmpLoader::displayLastTrajectory(){
//
// }


// this function is neccessary for serialization
// because the compiler doesn't import the classes
// that are neccessary to serialize a Dmp
// if they are not used somewhere
void fakeFunction(){

  MetaParametersLWR* meta_parameters = new MetaParametersLWR(0,0,0);
  FunctionApproximatorLWR* fa_lwr = new FunctionApproximatorLWR(meta_parameters);

  vector<FunctionApproximator*> function_approximators(0);
}

} // end namespace ros_dmp_tools
