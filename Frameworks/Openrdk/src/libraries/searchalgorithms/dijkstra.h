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

#ifndef SEARCHALGORITHMS_DIJKSTRA
#define SEARCHALGORITHMS_DIJKSTRA

#include "astar.h"

namespace SearchAlgorithms {

template<typename UserStateClass, typename CostClass = double>
class Dijkstra : public AStar<UserStateClass, CostClass> {
public:
	~Dijkstra() { }
	
protected:
	inline virtual double heuristicFunction(const UserStateClass&, const UserStateClass&) { return 0.; }
	
	/// you have to implement these in concrete classes
	/// virtual vector<UserStateClass> getSuccessorsOf(const UserStateClass& state) = 0;
	/// virtual double computeCost(const UserStateClass& state1, const UserStateClass& state2) = 0;
	/// virtual bool isSameState(const UserStateClass& state1, const UserStateClass& state2) = 0;
};

} // namespace SearchAlgorithms

#endif
