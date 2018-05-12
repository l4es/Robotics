#define ac_contains(v, x) (find(v.begin(),v.end(),x)!=v.end())

#define ptr2str(p) p

	template<class T>
		void PipeQueueConsumer<T>::debug_vector(const char*m, const vector<const T*>&v)
	{
		std::ostringstream oss;
		oss  << ":this=" << this << ": " << m << " [";
		for(size_t i=0;i<v.size();i++)
			oss << v[i] << ",";
		oss <<"]";
		logIt(oss.str());
	}

	template<class T>
		void PipeQueueConsumer<T>::debug_deque(const char*m, const deque<const T*>&v)
	{
		std::ostringstream oss;
		oss << ":this=" << this << ": " << m << " [";
		for(typename deque<const T*>::const_iterator i=v.begin(); i!=v.end();i++)
			oss << *i << ",";
		oss <<"]";

		logIt(oss.str());
	}

	template<class T>
		void PipeQueueConsumer<T>::debug_pointer(const char*m, const T*p) {
		std::ostringstream oss;
		oss  << ":this=" << this << ": " << m << " pointer= " << p;
		logIt(oss.str());
	}


	template<class T>
	void PipeQueueConsumer<T>::keep(const T*t) {
		debug_pointer("keep(t):t= ", t);
		debug_vector("keep(): current", current);
		debug_pointer("keep(): last", lastObject);
		debug_vector("keep(): toKeep", toKeep);

		if (!ac_contains(current, t)) {
			debug_pointer("keep(): current does not contain this pointer: ", t);
		} else
		if(ac_contains(toKeep, t)) {
			debug_pointer("keep(): keep already contains this pointer: ", t);
		} else {
			toKeep.push_back(t);
		}

		debug_vector("keep() - after: toKeep", toKeep);
	}

	template<class T>
	vector<const T*> PipeQueueConsumer<T>::freeze() {
		debug_vector("freeze(): current", current);
		debug_pointer("freeze(): last", lastObject);
		debug_vector("freeze(): toKeep", toKeep);

		vector<const T*> newObjects = downloadNew();

		debug_vector("freeze(): newobjects", newObjects);

		if(newObjects.size()) {
			// se ci sono nuovi oggetti

			// vengono flushati, anche last, ma non quelli in keep
			vector<const T*> toFlush;
			for(size_t i=0;i<current.size();i++)
				if( (!ac_contains(toKeep, current[i])) && (current[i]!=lastObject))
					toFlush.push_back(current[i]);

			if(lastObject && (!ac_contains(toKeep,lastObject)) && (!ac_contains(toKeep,lastObject)))
				toFlush.push_back(lastObject);


			debug_vector("freeze(): toFlush", toFlush);

			flushOld(toFlush);


			// il nuovo lastObjects è l'ultimo dei newObjects
			lastObject = newObjects[newObjects.size()-1];

		} else {
			// se non ci sono nuovi oggetti:
			// last non viene cambiato
			// vengono flushati tutti tranne last e quelli in keep
			vector<const T*> toFlush;
			for(size_t i=0;i<current.size();i++)
				if( (!ac_contains(toKeep, current[i])) && (current[i]!=lastObject))
					toFlush.push_back(current[i]);

			debug_vector("freeze(): toFlush", toFlush);

			flushOld(toFlush);

		}

		// il nuovo current è keep + newObjects
		current = toKeep;
		for(size_t i=0;i<newObjects.size();i++)
			if (!ac_contains(current, newObjects[i])) current.push_back(newObjects[i]);

		// il nuovo keep è vuoto
		toKeep.clear();

		debug_vector("freeze() - after: current", current);
		debug_pointer("freeze() - after: last", lastObject);
		debug_vector("freeze() - after: toKeep", toKeep);


		return current;
	}

	template<class T>
	vector<const T*>  PipeQueueConsumer<T>::content() { return current; }


	template<class T>
	const T*  PipeQueueConsumer<T>::last() {
		return lastObject;
	}

	template<class T>
	void PipeQueueConsumer<T>::flushOld(const vector<const T*>&v) {
		debug_vector("flushOld(): toFlush", v);
		debug_deque("flushOld(): pipe_buffer", pipe_buffer);
		for(size_t i=0;i<v.size();++i)
			pipe_buffer.push_back(v[i]);

		debug_deque("flushOld(): pipe_buffer before flushPipeBuffer ", pipe_buffer);
		flushPipeBuffer();
		debug_deque("flushOld(): pipe_buffer after flushPipeBuffer ", pipe_buffer);
	}

	template<class T>
	void PipeQueueConsumer<T>::flushPipeBuffer() {
		// While we have stuff to send
		while( pipe_buffer.size() > 0 ) {
			// Consider the first in list
			const T * first = pipe_buffer.front();
			// Try to write
			if(!writePointer(fd_out, (void*)first)) {
				// TODO: warn only after a certain size
				// TODO: implement a callback to warn the thread?
				cerr << this->name << ": My return pipe is full: current buffer size: " <<
					pipe_buffer.size() << ". Do you call gc() on the master queue every once in a while?" << endl;
				break;
			}
			// Okay, first is gone
			pipe_buffer.pop_front();
		}
	}

	template<class T>
	vector<const T*> PipeQueueConsumer<T>::downloadNew() {
		vector<const T*> v;
		void * p = 0;
		while(readPointer(fd_in,&p))
			if (!ac_contains(v, p)) v.push_back((const T*)p);
		return v;
	}



	template<class T>
	void PipeQueue<T>::
	setSignalSemaphore(PipeQueueConsumer<T>*c, PosixConstructs::PosixSemaphore* semaphore){
		mutex.lock(HERE);
			typename Con2sub::iterator i = subscriptions.find(c);
			if (i==subscriptions.end()) {
				// error
			} else {
				i->second.semaphore = semaphore;
			}
		mutex.unlock();
	}

	template<class T>
	bool PipeQueue<T>::
	hasSignalSemaphore(PipeQueueConsumer<T>*c) {
		bool ret = false;
		mutex.lock(HERE);
			typename Con2sub::iterator it = subscriptions.find(c);
			if (it == subscriptions.end()) {
				// error
			}
			else {
				ret = (it->second.semaphore != 0);
			}
		mutex.unlock();
		return ret;
	}

	template<class T>
	void PipeQueue<T>::
	setInterests(PipeQueueConsumer<T>*c, MyInterests<T>*interests) {
		mutex.lock(HERE);
			typename Con2sub::iterator i = subscriptions.find(c);
			if(i==subscriptions.end()) {
				// error
			} else {
				i->second.interests = interests;
			}
		mutex.unlock();
	}

	template<class T>
	void PipeQueue<T>::deleteConsumer(PipeQueueConsumer<T>*c, Stack w) {
		mutex.lock(w+HERE);
			typename Con2sub::iterator i = subscriptions.find(c);
			if(i==subscriptions.end()) {
				// error
			} else {
				close(i->second.fd_in);
				close(i->second.fd_out);
				subscriptions.erase(i);
			}
		delete c;
		mutex.unlock();
	}


	template<class T>
	PipeQueueConsumer<T> *
	PipeQueue<T>::createConsumer(const std::string&name, Stack w) {
		mutex.lock(w+HERE);
			// From master to consumer
			struct { int read; int write; } m2c;
			int res = pipe((int*)&m2c);
			if(0!=res) { mutex.unlock(); return 0; }

			// From consumer to master
			struct { int read; int write; } c2m;
			res = pipe((int*)&c2m);
			if(0!=res) { mutex.unlock(); return 0; } // should deallocate other pipe

			if(PIPEQUEUE_NOT_BLOCKING) {
				setNonBlocking(m2c.write);
				setNonBlocking(c2m.write);
			}

			Subscription s;
				s.fd_in = c2m.read;
				s.fd_out = m2c.write;
				s.interests = 0;
				s.semaphore = 0;
				s.name = name;
			string n = name + "(M=" + this->name + ")";
			PipeQueueConsumer<T>*c=new PipeQueueConsumer<T>(m2c.read,c2m.write,n);
			subscriptions[c]=s;
		mutex.unlock();
		return c;
	}

	template<class T>
	size_t PipeQueue<T>::push(T*ob, Stack w) {
    		mutex.lock(w+HERE);
			if(ptr2ref.find(ob)!=ptr2ref.end()) {
				logIt("ERROR!! pushing again");
				mutex.unlock();
				return 0;
			}			// first put in the buffers
			size_t copies = 0;
			for(typename Con2sub::iterator i=subscriptions.begin(); i!=subscriptions.end();++i)
				if(! i->second.interests ||
				    i->second.interests->areYouInterested(ob))
				{
					copies++;
					i->second.buffer.push_back(ob);
				}

			if(copies) {
				ptr2ref[ob] = copies;
			} else {
				delete ob;
			}

			// then flush the buffers
			for(typename Con2sub::iterator i=subscriptions.begin(); i!=subscriptions.end();++i)
			{
				std::deque<const T*> &buffer = i->second.buffer;
				int fd_out = i->second.fd_out;

				// While we have stuff to send
				while( buffer.size() > 0 ) {
					// Consider the first in list
					const T * first = buffer.front();
					// Try to write
					if(!writePointer(fd_out, (void*)first)) {
						// TODO: warn only after a certain size
						// TODO: implement a callback to warn the thread?
						cerr << this->name << ": This is not an error, but pay attention. "
						"One pipe is full: " <<
							i->second.name << " current buffer size: " <<
							i->second.buffer.size() << endl;
						break;
					} else {
						// Okay, first is gone
						buffer.pop_front();
						// Call the callback
						push_callback(i->first);
						// Signal the thread
						if(i->second.semaphore)
							i->second.semaphore->signal();

					}

				}
			}

		mutex.unlock();
		logIt(string("MASTER this=")+toString(this)+": pushed object "+
			toString(ob) + " with copies " + toString(copies));

		return copies;
	}

	template<class T>
	size_t  PipeQueue<T>::size() {
		mutex.lock(HERE);
		size_t s = ptr2ref.size();
		mutex.unlock();
		return s;
	}

	template<class T>
	void PipeQueue<T>::gc(Stack w) {
		mutex.lock(w+HERE);
		//T * ob = 0;
		void* ob = 0;

		int n=0;
		for(typename Con2sub::iterator i=subscriptions.begin(); i!=subscriptions.end();++i) {
			n++;
			while(readPointer(i->second.fd_in, /*(void**)*/&ob)) {
				logIt(string("MASTER this=")+toString(this)+": gc from client "
					+ toString(n) + " read pointer " + toString(ob));

				typename Ptr2ref::iterator j = ptr2ref.find((T*)ob);
				if(j==ptr2ref.end()) {
					logIt(string("MASTER this=")+toString(this)+": BUG error, pointer "+
						toString(ob) + " is not present in ptr2ref");

					for(typename Ptr2ref::iterator k=ptr2ref.begin();k!=ptr2ref.end();++k) {
						logIt(string("  object ") + toString(k->first) +
						 	" ref. = " + toString(k->second));
					}
				}
				else {
					if (ac_contains(i->first->toKeep, ob)) {
						logIt(string("MASTER this=")+toString(this)+": trying to -- a kept object");
					}
					else j->second--;

					logIt(string("MASTER this=")+toString(this)+": gc -- ref: "+toString(j->second)+
						 	string(" obj=")+toString(ob));

					if(0==j->second) {
						logIt(string("MASTER this=")+toString(this)+": gc -- deleting "
							+toString(ob));
						ptr2ref.erase(j);
						delete (T*) ob;
					}
				}
			}
		}
		mutex.unlock();
	}
