/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007  Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
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

#include <iostream>
#include "pipequeue.h"

	
using namespace PosixQueues;
using namespace PosixConstructs;
using namespace std;


PipeQueue<int> intQueue("que");

int main()
{
	PipeQueueConsumer<int>* q = intQueue.createConsumer("io", HERE);

	int a = 0;
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 10; j++) {
			int* k = new int(a++);
			intQueue.push(k, HERE);
			intQueue.gc(HERE);
		}
		vector<const int*> v = q->freeze();
		cout << "Got " << v.size() << " elements" << endl;
		v = q->freeze();
		cout << "Freezed again, got " << v.size() << " elements" << endl;
	}

}
