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

#include "pipequeue.h"

	
using namespace PosixQueues;
using namespace PosixConstructs;
using namespace std;


PipeQueue<int> intQueue("queue");

void* thread(void *) {
	PosixSemaphore mysem;
	PipeQueueConsumer<int> * c = intQueue.createConsumer("io",HERE);
	
	intQueue.setSignalSemaphore(c, &mysem);
	
	while(1) {
		mysem.wait();
		vector<const int*> results = c->freeze();
		
		sleep(3);
		if(0) {
			intQueue.deleteConsumer(c, HERE);
			//sem.signal();
			return 0;
		}
	}

}

int main() {
	cerr << "Here the consumer reads only every three seconds, therefore objects accumulate in the buffer." << endl;
	
	pthread_t p;
	
	pthread_create(&p,0,thread,NULL);
	
	sleep(5);
	
	for(int count=0;;count++) {
		int * i = new int(count);
		cout << "Pushing " << count << " " << i << endl;
		if(0==intQueue.push(i, HERE)) return 0;
		count++;
		intQueue.gc(HERE);
	}
	
}
