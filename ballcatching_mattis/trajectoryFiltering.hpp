#ifndef __TRAJECTORY_FILTERING_HPP
#define __TRAJECTORY_FILTERING_HPP

#include "config.hpp"

class TrajectoryFilteringParameters{
public:
  //time (in milli) after which a trajectory is considered lost.
  int loosingFrames;
  //Max numer of frame per trajectory
  int maxFramePerTrajectory;

  TrajectoryFilteringParameters(){
    loosingFrames = 3;
    maxFramePerTrajectory = 50;
  }
};

void filterTrajectories(list<Balls::Trajectory>& trajectories,
			const vector<double>& times,
			int frame,
			const TrajectoryFilteringParameters& p 
			= TrajectoryFilteringParameters());


#endif
