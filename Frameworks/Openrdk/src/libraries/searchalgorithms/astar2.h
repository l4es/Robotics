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

#ifndef SEARCH_ALGORITHMS_A_STAR_2
#define SEARCH_ALGORITHMS_A_STAR_2

#include <float.h>
#include <queue>

namespace SearchAlgorithms {

using namespace std;

template<typename UserStateClass>
struct LabeledState {
	LabeledState(const UserStateClass& userState) : parent(0), costToCome(DBL_MAX), estimatedCostToGo(DBL_MAX), userState(userState) { }

	LabeledState<UserStateClass>* parent;
	double costToCome;
	double estimatedCostToGo;
	UserStateClass userState;
};

template<typename UserStateClass>
class LStateComparison : public std::binary_function<LabeledState<UserStateClass>*, LabeledState<UserStateClass>*, bool> {
public:
	bool operator()(LabeledState<UserStateClass>*& state1, LabeledState<UserStateClass>*& state2) const
	{
		// note: for a priority queue, "top" is "back" (damned std)
		return (state1->costToCome + state1->estimatedCostToGo) > (state2->costToCome + state2->estimatedCostToGo);
	}
};

template<typename UserStateClass>
class AStar2 {
protected:
	typedef LabeledState<UserStateClass> LState;
	priority_queue<LState*, vector<LState*>, LStateComparison<UserStateClass> > aliveLStates;
	vector<LState*> visitedLStates;

	virtual bool isTargetState(const UserStateClass& userState, const UserStateClass& userTargetState) = 0;
	virtual vector<UserStateClass> getSuccessorsOf(const UserStateClass& userState) = 0;
	virtual double computeCost(const UserStateClass& state1, const UserStateClass& state2) = 0;
	virtual double heuristicFunction(const UserStateClass& userState, const UserStateClass& userTargetState) = 0;
	virtual void markExpanded(const UserStateClass& userState) = 0;
	virtual bool isExpanded(const UserStateClass& userState) = 0;
	virtual void markEnqueued(const UserStateClass& userState) = 0;
	virtual bool isEnqueued(const UserStateClass& userState) = 0;

	virtual void printState(const UserStateClass& /*state*/) { }
	virtual void debugSuccessorStates(const vector<UserStateClass>& /*successors*/) { }

	size_t uniqueSearchedStatesInLastSearch, searchedStatesInLastSearch;

public:
	AStar2() { }
	virtual ~AStar2() { }

	list<UserStateClass> search(const UserStateClass& start, const UserStateClass& target);
	inline size_t getUniqueSearchedStatesInLastSearch() { return uniqueSearchedStatesInLastSearch; }
	inline size_t getSearchedStatesInLastSearch() { return searchedStatesInLastSearch; }
};

} // namespace

#include "astar2.hpp"

#endif
