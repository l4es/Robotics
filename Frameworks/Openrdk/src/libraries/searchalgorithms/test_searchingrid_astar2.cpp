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

#include <stdio.h>
#include <cstdlib>
#include <vector>
#include <list>
#include <math.h>
#include <string>
#include <sstream>
using namespace std;

#include "astar2.h"

struct Point {
	Point(int x = 0, int y = 0) : x(x), y(y) { }
	int x, y;
	inline bool operator==(const Point& p) const { return (p.x == x && p.y == y); }
	inline bool operator<(const Point& p) const { return (p.x < x || (p.x == x && p.y < y)); }
	inline string toString() const { ostringstream oss; oss << "(" << x << "," << y << ")"; return oss.str(); }
};

ostream& operator<<(ostream&os, const Point&p) {
	return os << "("<<p.x<<","<<p.y<<")";	
}

class AStarWithGrid : public SearchAlgorithms::AStar2<Point> {
public:
	int w, h;
	
public:
	bool** grid;
	int** visited;

public:
	AStarWithGrid(int w, int h) : SearchAlgorithms::AStar2<Point>(), w(w), h(h)
	{
		grid = new bool*[w];
		for (int i = 0; i < w; i++) grid[i] = new bool[h];
		for (int x = 0; x < w; x++)
			for (int y = 0; y < h; y++)
				grid[x][y] = (rand() % 5 == 0);
		visited = new int*[w];
		for (int i = 0; i < w; i++) visited[i] = new int[h];
		for (int x = 0; x < w; x++)
			for (int y = 0; y < h; y++)
				visited[x][y] = 0;	// 0 means nothing, 1 means enqueued, 2 means visited
	}
	
	~AStarWithGrid()
	{
		for (int i = 0; i < w; i++) delete[] grid[i];
		delete[] grid;
		for (int i = 0; i < w; i++) delete[] visited[i];
		delete[] visited;
	}

	inline bool isTargetState(const Point& state, const Point& target) { return state == target; }
	vector<Point> getSuccessorsOf(const Point& state);
	double computeCost(const Point& state1, const Point& state2);
	inline double heuristicFunction(const Point& state1, const Point& state2) { return computeCost(state1, state2); }
	inline void markExpanded(const Point& state) { visited[state.x][state.y] = 2; }
	inline bool isExpanded(const Point& state) { return visited[state.x][state.y] == 2; }
	inline void markEnqueued(const Point& state) { visited[state.x][state.y] = 1; }
	inline bool isEnqueued(const Point& state) { return visited[state.x][state.y] == 1; }
};

vector<Point> AStarWithGrid::getSuccessorsOf(const Point& p)
{
	vector<Point> r;
	// XXX tremendamente inefficiente
	#define ROBA(x,y) if (x >= 0 && y >= 0 && x < w && y < h && grid[x][y] == false) r.push_back(Point(x, y));
	ROBA(p.x-1, p.y)
	ROBA(p.x-1, p.y-1)
	ROBA(p.x-1, p.y+1)
	ROBA(p.x, p.y-1)
	ROBA(p.x, p.y+1)
	ROBA(p.x+1, p.y)
	ROBA(p.x+1, p.y-1)
	ROBA(p.x+1, p.y+1)
	return r;
}

double AStarWithGrid::computeCost(const Point& state1, const Point& state2)
{
	double x2 = state1.x - state2.x, y2 = state1.y - state2.y;
	return sqrt(x2 * x2 + y2 * y2);
}

int main(int, char**)
{
	AStarWithGrid dijkstra(40, 40);
	list<Point> path = dijkstra.search(Point(2, 2), Point(28, 30));
	for (int x = 0; x < dijkstra.w; x++) {
		for (int y = 0; y < dijkstra.h; y++) {
			bool found = false;
			for (list<Point>::iterator it = path.begin(); it != path.end(); ++it) {
				if (it->x == x && it->y == y) {
					printf("@ ");
					found = true;
					break;
				}
			}
			if (!found) printf("%s ", (dijkstra.grid[x][y] ? "#" : "."));
		}
		printf("\n");
	}
	printf("Path length: %zd, Searched states: %zd (unique: %zd)\n", path.size(),
		dijkstra.getSearchedStatesInLastSearch(),
		dijkstra.getUniqueSearchedStatesInLastSearch());
	return 0;
}
