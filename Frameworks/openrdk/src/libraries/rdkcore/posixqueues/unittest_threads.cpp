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
#include <unistd.h>
#include "onemutexmasterqueue.h"


using namespace PosixQueues;
using namespace PosixConstructs;
using namespace std;

#define N 100

OneMutexMasterQueue<int> intQueue;
PosixSemaphore sem;

void* thread(void *) {
	
	int num = intQueue.getNewId();
	int max = 100+num * (num&3);
	
	ConsumerQueueInterface<int> * q = intQueue.getQueue(num); 
	while(1) {
		q->wait();	
		LOCK(q, "reading");
		if(q->size()) {
			cout << "Hi " << num << " saw " << *q->first() << endl;
			q->discardFirst();
		} else {
			cerr << "size=0? I'm thread " << num << " max " << max << endl;
		}
		q->unlock();
		
		if(!max--) {
			intQueue.deleteQueue(num);
			sem.signal();
			return 0;
		}
		
	}
}

int main() {
	pthread_t p;
	
	for(int a=0;a<N;a++)
		pthread_create(&p,0,thread,NULL);
	
	sleep(1);
	int count = 0;
	while(1) {
		cout << "Pushing " << count << endl;
		if(!intQueue.push_back(new int(count))) return 0;
		count++;
	}
	
	for(int a=0;a<N;a++)
		sem.wait();
}
