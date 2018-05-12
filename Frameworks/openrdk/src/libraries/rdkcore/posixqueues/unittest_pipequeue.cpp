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

#define N 500

	PipeQueue<int> intQueue("que");
	PosixSemaphore sem;

void* thread(void *) {
	PosixSemaphore mysem;
	PipeQueueConsumer<int> * c = intQueue.createConsumer("io",HERE);
	
	intQueue.setSignalSemaphore(c, &mysem);
	
	int id = 1;
	int num = id % N;
	int max = 100+num * (num&3);
	cout << id << " max= " << max << endl;
	while(1) {
		cout << id << " waiting "<<  endl;
		
		mysem.wait();
		
		cout << id << " waited "<<  endl;
		
		vector<const int*> results = c->freeze();
		
		for(size_t i=0;i<results.size();i++) {
			const int * p = results[i];
			cout << "Hi " << num << " saw " << p << " = " << *p << endl;
		
		}
		
		max -= results.size();
		if(max<0) {
			intQueue.deleteConsumer(c, HERE);
			sem.signal();
			return 0;
		}
	}

}

int main() {
	pthread_t p;
	
	for(int a=0;a<N;a++)
		pthread_create(&p,0,thread,NULL);
	
	sleep(5);
	cerr << "starting push" << endl;
	
	for(int count=0;;count++) {
		int * i = new int(count);
		cout << "Pushing " << count << " " << i << endl;
		if(0==intQueue.push(i, HERE)) return 0;
		count++;
		sleep(1);
		intQueue.gc(HERE);
	}
	
	for(int a=0;a<N;a++) {
	}
	
	for(int a=0;a<N;a++) {
		sem.wait();
	}
	intQueue.gc(HERE);
}
