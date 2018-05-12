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

#include <cmath>
using namespace std;

#include "basesearchalgorithm.h"

namespace SearchAlgorithms {

template<>
bool costIsEqual<double>(const double& a, const double& b)
{
	return fabs(a - b) < 0.0001;
}

#define EPS 0.0001
	bool equal(double a, double b) {
		return fabs(a-b)/((a+b)/2) < EPS;
	}
	
	bool lessThan(double a, double b) {
		return a + EPS < b;
	}

}
