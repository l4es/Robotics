/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef SEARCHALGORITHMS_ASTAR
#define SEARCHALGORITHMS_ASTAR

#include "basesearchalgorithm.h"

namespace SearchAlgorithms {

template<typename UserStateClass, typename CostClass = double>
class AStar : public BaseSearchAlgorithm<UserStateClass, CostClass> {
public:
	AStar() { }
	~AStar() { }

protected:
	virtual BaseState<UserStateClass, CostClass>* popFirstAliveState() {
		if (BaseSearchAlgorithm<UserStateClass, CostClass>::aliveStates.empty())
			return 0;
		
		typename list<BaseState<UserStateClass, CostClass>*>::iterator bestIt, it;
		
		it = BaseSearchAlgorithm<UserStateClass, CostClass>::aliveStates.begin();
		bestIt = it;
		
		for (;it != BaseSearchAlgorithm<UserStateClass, CostClass>::aliveStates.end(); ++it) {
#if 0
			double coeff = 1;
			CostClass c1 = (*it)->costToCome;
			CostClass h1 = coeff*heuristicFunction((*it)->userState,
				BaseSearchAlgorithm<UserStateClass, CostClass>::userTargetState);
			CostClass c2 = (*bestIt)->costToCome;
			CostClass h2 = coeff*heuristicFunction((*bestIt)->userState,
				BaseSearchAlgorithm<UserStateClass, CostClass>::userTargetState);
			
			bool equal = costIsEqual(c1+h1,c2+h2);//fabs((c1+h1)-(c2+h2)) < 0.00001;
			
			if ((!equal & (c1 + h1 < c2 + h2))||( equal & (h1 < h2))) bestIt = it;
#endif
#if 1
			if ((*it)->costToCome + heuristicFunction((*it)->userState,
			BaseSearchAlgorithm<UserStateClass>::userTargetState)
			< (*bestIt)->costToCome + heuristicFunction((*bestIt)->userState,
			BaseSearchAlgorithm<UserStateClass>::userTargetState))
		 		bestIt = it;
#endif
		}
		
		BaseState<UserStateClass, CostClass>* best = *bestIt;
		BaseSearchAlgorithm<UserStateClass, CostClass>::aliveStates.erase(bestIt);
		return best;
	}
	
	/// you have to implement these in concrete classes
	/// virtual vector<UserStateClass> getSuccessorsOf(const UserStateClass& state) = 0;
	/// virtual double computeCost(const UserStateClass& state1, const UserStateClass& state2) = 0;
	/// virtual bool isSameState(const UserStateClass& state1, const UserStateClass& state2) = 0;
	virtual double heuristicFunction(const UserStateClass& state1, const UserStateClass& state2) = 0;
};

} // namespace SearchAlgorithms

#endif
