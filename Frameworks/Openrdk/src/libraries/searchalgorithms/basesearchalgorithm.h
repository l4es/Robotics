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

#ifndef SEARCHALGORITHMS_BASESEARCHALGORITHM
#define SEARCHALGORITHMS_BASESEARCHALGORITHM

/**

Search algorithms

This classes implements some widely-used search algorithms, i.e. algorithms 
that look for a plan, or a path, made of (discrete) states in the state space.

In order to use them, you have to implement a subclass of BestFirst, Dijkstra 
or AStar classes and implement the pure virtual functions. You can use whatever
class you want as state and use it in the instantiation of these search classes, e.g.:

class MyAStarClass : public Dijkstra<MyStateClass> {
...
};

the pure virtual functions you have to implement are:

virtual vector<UserStateClass> getSuccessorsOf(const UserStateClass& state);
This function has to return a vector of states that are the successors of 
state \param state . You have to implement it in every algorithms.

virtual double computeCost(const UserStateClass& state1, const UserStateClass& state2);
This function should return the cost to go from state1 to state2.
You have to implement it only for Dijkstra and AStar subclasses.

virtual bool isSameState(const UserStateClass& state1, const UserStateClass& state2) = 0;
This function has to return true if the two states are the same, false otherwise.
You have to implement it in every algorithms.

For the UserStateClass, you can use an object or you can use a pointer, if you
don't want too many copies of your own states. This is the reason for the 
isSameState function (you have to compare the object pointed
by the pointers, instead of the pointers themselves!).

If you choose to use objects instead of pointers, the objects' class need to
implement operator<.

The main function of these algorithms is search, to which you have to provide 
the start state and target state and returns a list of states that is the 
(best for AStar and Dijkstra) plan to reach the target from the start state.

*/

#include <list>
#include <map>
#include <vector>
#include <stdexcept>
#include <float.h>	// for DBL_MAX
#include <sys/types.h>
#include <iostream>

namespace SearchAlgorithms {

using namespace std;

template<typename UserStateClass, typename CostClass = double>
struct BaseState {
	BaseState(const UserStateClass& userState) :
		parent(0), costToCome(DBL_MAX), userState(userState) { }
	BaseState<UserStateClass, CostClass>* parent;
	CostClass costToCome;
	UserStateClass userState;
};

	bool equal(double a, double b);
	
	bool lessThan(double a, double b);
	
template<typename UserStateClass, typename CostClass = double>
class BaseSearchAlgorithm {
public:
	virtual ~BaseSearchAlgorithm() { }
	
protected:
	typedef BaseState<UserStateClass, CostClass> State;
	
	/// All visited states
	list<pair<UserStateClass, State*> > visitedStates;

	/// Currenty active states
	list<State*> aliveStates;

protected:
	UserStateClass userTargetState;
	void freeStates();
	list<UserStateClass> makePlanAndFreeStates(State* targetState);

protected:
	/// You have to implement this for new search algorithms
	virtual State* popFirstAliveState() = 0;
	
	/// You have to implement these in concrete classes
	virtual vector<UserStateClass> getSuccessorsOf(const UserStateClass& state) = 0;
	virtual CostClass computeCost(const UserStateClass& state1, const UserStateClass& state2) = 0;
	bool isDominated(const UserStateClass& uStateStart, CostClass costToCome);
	void eraseAliveDominatedBy(const UserStateClass& s, CostClass costToCome);

	virtual bool isSameState(const UserStateClass& state1, const UserStateClass& state2) = 0;
	virtual bool isTargetState(const UserStateClass& state);

	virtual bool isDominatedBy(const UserStateClass& /*state1*/, const UserStateClass& /*state2*/) { return false; }

	inline virtual bool costIsBetter(const CostClass& cost1, const CostClass& cost2) { return cost1 < cost2; }
	virtual CostClass sumCosts(const CostClass& cost1, const CostClass& cost2) { return cost1 + cost2; }

public:
	list<UserStateClass> search(const UserStateClass& start, const UserStateClass& target);
	list<UserStateClass> search(const UserStateClass& start);
	
	uint getSearchedStatesInLastSearch() { return searchedStates; }
	uint getUniqueSearchedStatesInLastSearch() { return uniqueSearchedStates; }

protected:
	uint searchedStates;
	uint uniqueSearchedStates;
};

template<typename T>
bool costIsEqual(const T& a, const T& b);

template<>
bool costIsEqual<double>(const double& a, const double& b);

} // namespace SearchAlgorithms

#include "basesearchalgorithm.hpp"

#endif
