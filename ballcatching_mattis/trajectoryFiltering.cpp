#include "trajectoryFiltering.hpp"



void filterTrajectories(Trajectories& trajectories,
			int time,
			const TrajectoryFilteringParameters& p){
  
  vector<list<vector<pair<Ellipse,int> > >::iterator> toRemove;
  for(list<vector<pair<Ellipse,int> > >::iterator it = trajectories.trajectories.begin();
      it!=trajectories.trajectories.end();it++){
    if(time-(*it)[it->size()-1].second > p.loosingFrames){
      toRemove.push_back(it);
    }
  }
  for(vector<list<vector<pair<Ellipse,int> > >::iterator>::const_iterator it = toRemove.begin();
      it!=toRemove.end(); ++it){
    trajectories.trajectories.erase(*it);
  }
}
