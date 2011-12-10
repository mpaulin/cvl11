#include "trajectoryFiltering.hpp"

void filterTrajectories(list<Balls::Trajectory>& trajectories,
			const vector<double>& times,
			int frame,
			const TrajectoryFilteringParameters& p){
  
  vector<list<Balls::Trajectory>::iterator> toRemove;
  for(list<Balls::Trajectory>::iterator it = trajectories.begin();
      it!=trajectories.end();
      ++it){
    if(times[frame]-times[it->getFrame(it->length-1)] > p.loosingFrames){
      toRemove.push_back(it);
    }
  }
  for(vector<list<Balls::Trajectory>::iterator>::const_iterator it = toRemove.begin();
      it!=toRemove.end(); ++it){
    trajectories.erase(*it);
  }
}
