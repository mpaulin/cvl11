#include "trajectoryFiltering.hpp"


void filterTrajectories(list<Balls::Trajectory>& trajectories,
		const vector<double>& times,
		int frame,
		const TrajectoryFilteringParameters& p){
	for(list<Balls::Trajectory>::iterator it = trajectories.begin();
			it!=trajectories.end();
			++it){
		if(it->length>1 && (frame-(it->getFrame(it->length-1)) > p.loosingFrames
				|| it->length > p.maxFramePerTrajectory)){
			it=trajectories.erase(it);
			--it;
		}
	}
}
