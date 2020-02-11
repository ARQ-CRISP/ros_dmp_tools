# ROS DMP Tools

Tools for learning from demonstration using *dynamical movement primitives (DMP)* framework under *robot operating system (ROS)*.

Currently, it uses the open-source DMP implementation of [*stulp/DMPBBO*](https://github.com/stulp/dmpbbo). It depends on [Eigen3 library](http://eigen.tuxfamily.org).

## Installation
*Tested on Ubuntu 16.04, ROS-Kinetic*

* **Install Eigen3:** `sudo apt install libeigen3-dev`
* **Install DMPBBO:** Clone [*stulp/DMPBBO*](https://github.com/stulp/dmpbbo) and follow [the installation instructions](https://github.com/stulp/dmpbbo/blob/master/INSTALL.txt). It should be installed as a user library, i.e. using `sudo make install`.
* **Install ROS packages:** Clone [*ARQ-CRISP/ros_dmp_tools*](https://github.com/ARQ-CRISP/ros_dmp_tools) and [*kdl_control_tools*](https://github.com/ARQ-CRISP/kdl_control_tools) into `your-catkin-workspace/src` directory. Build the workspace with `catkin_make`.

## Usage

This package contains several ROS nodes and classes for _recording_, _learning_ and _reproducing_ demonstrations.

#### Recording:

Recording is done using a `TrajectoryRecorder` object. It listens to the topic of a generic message type (`RecordState.msg`) using a `StateListener` object. `StateListener` is a template publisher, that publishes the state as `RecordState.msg`. An example implementation of `StateListener` is `JointStateListener`.

Example usage of `TrajectoryRecorder`:

```c++
TrajectoryRecorder trajectory_recorder(&node_handle);

// start a state listener of choice
state_listener_ = new JointStateListener(&node_handle);

state_listener_->startListening();
trajectory_recorder.startRecording(state_size_, duration_, step_count_);

// Keep the node running
ros::spin();
```

The trajectory is saved as a _"traj.txt"_ file at the end of the given `duration_` (seconds).

#### Learning:

Learning can be done using a `train_dmp` node:

```bash
roslaunch dmp_tools train.launch dmp_name:=mydmp.xml trajectory_name:=mytraj.txt
```

It will read the _mytraj.txt_ file in the catkin workspace directory. It will produce a _mydmp.xml_ file in the _~/.ros_ directory. File names may include the directory path optionally.

Parameters of *train*:
* *dmp_type (0):* Type of DMP implementation of DMPBBO. Integer.
* *scaling_type (0):* Type of force term scaling of DMPBBO. Integer.
* *basis_count (30):* Number of basis functions to form the forcing term.
* *intersection (0.56):* Ratio of intersection between basis functions.
* *save_details (true):* Flag to save and visualize extra info.

#### Reproducing:

`DmpLoader` class can be used for loading a saved trajectory:

```c++
// Creating the object
string dmp_filename = "dmp.xml";
DmpLoader* dmp_loader_ = DmpLoader(dmp_filename);
```

```c++
// Creating a trajectory
// Inputs: init_state_, goal_state_, step_count_, duration_
// Outputs: ts (time-step vector), qs, qds, qdds (q: state variable, d: dot/derivative)
dmp_loader_->setInitialState(init_state_);
dmp_loader_->setGoalState(goal_state_);
dmp_loader_->createTrajectory(step_count_, duration_, ts, qs, qds, qdds);

```
