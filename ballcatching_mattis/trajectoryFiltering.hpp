#ifndef __TRAJECTORY_FILTERING_HPP
#define __TRAJECTORY_FILTERING_HPP

#include "config.hpp"

class TrajectoryFilteringParameters{
public:
  //Number of frames after which a trajectory is considered lost. 
  int loosingFrames;

  TrajectoryFilteringParameters(){
    loosingFrames = 4;
  }
};

void filterTrajectories(list<Balls::Trajectory>& trajectories,
			const vector<double>& times,
			int frame,
			const TrajectoryFilteringParameters& p 
			= TrajectoryFilteringParameters());


#endif
