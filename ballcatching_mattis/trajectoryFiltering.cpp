#include "trajectoryFiltering.hpp"

void filterTrajectories(list<Balls::Trajectory>& trajectories,
			const vector<double>& times,
			int frame,
			const TrajectoryFilteringParameters& p){
  
	for(list<Balls::Trajectory>::iterator it = trajectories.begin();
		it!=trajectories.end();
		++it){
		if(times[frame]-times[it->getFrame(it->length-1)] > p.loosingFrames
				|| it->length > p.maxFramePerTrajectory){
			trajectories.erase(it);
    }
  }
}
