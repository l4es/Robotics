
template <typename T>
const T* Session::getObjectAsLC(cstr url)
	throw (NoSuchProperty, InvalidOperation, ValueNotSet, WrongType)
{
	checkStarted();
	const RDK2::Object* obj = getObjectLC(url);
	if (!obj) throw ValueNotSet("Property '" + url + "' is not set (3)");
	const T* t = dynamic_cast<const T*>(obj);
	if (!t) {
		T a;
		throw WrongType("Wrong object class for property '" +
			url + "' (it is " + obj->getClassName() + ", you want " + a.getClassName() + ")");
	}
	return t;
}

template <typename T>
T* Session::getObjectAsL(cstr url)
	throw (NoSuchProperty, InvalidOperation, ValueNotSet, WrongType)
{
	checkStarted();
	RDK2::Object* obj = getObjectL(url);
	if (!obj) throw ValueNotSet("Property '" + url + "' is not set (1)");
	T* t = dynamic_cast<T*>(obj);
	if (!t) {
		T a;
/*		if (obj->getClassName() == a.getClassName()) {
			// FIXME!!!!!!!!!!!!!!
			t = static_cast<T*>(obj);
		}
		else { */
			throw WrongType("Wrong object class for property '" +
				url + "' (it is " + obj->getClassName() + ", you want " + a.getClassName() + " (1))");
// 		}
	}
	return t;
}

template <typename T>
T* Session::getObjectCloneAs(cstr url)
	throw (NoSuchProperty, InvalidOperation, ValueNotSet, WrongType)
{
	checkStarted();
	RDK2::Object* obj = getObjectClone(url);
	if (!obj) throw ValueNotSet("Property '" + url + "' is not set (2)");
	T* t = dynamic_cast<T*>(obj);
	if (!t) {
		T a;
		if (obj->getClassName() == a.getClassName()) {
			// FIXME!!!!!!!!!!!!!!
			t = static_cast<T*>(obj);
		}
		else {
			throw WrongType("Wrong object class for property '" +
				url + "' (it is " + obj->getClassName() + ", you want " + a.getClassName() + " (2))");
 		}
	}
	return t;
}

template<typename T>
vector<const T*> Session::queueFreezeAs(CUrl url)
	throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	vector<const T*> r;
	const vector<const RDK2::Object*> v = queueFreeze(url);
	for (uint i = 0; i < v.size(); i++) {
		const T* t = dynamic_cast<const T*>(v[i]);
		if (t) r.push_back(t);
	}
	return r;
}

template<typename T>
vector<const T*> Session::queueContentAs(CUrl url)
	throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	vector<const T*> r;
	vector<const RDK2::Object*> v = queueContent(url);
	for (uint i = 0; i < v.size(); i++) {
		const T* t = dynamic_cast<const T*>(v[i]);
		if (t) r.push_back(t);
	}
	return r;
}

template<typename T>
const T* Session::queueLastAs(CUrl url)
	throw (NoSuchProperty, InvalidOperation, WrongType)
{
	checkStarted();
	const RDK2::Object* obj = queueLast(url);
	if (!obj) throw InvalidOperation("Queue had a null object (boh)");
	const T* t = dynamic_cast<const T*>(obj);
	if (!t) {
		T a;
		throw WrongType("Wrong object class for property '" +
			url + "' (it is " + obj->getClassName() + ", you want " + a.getClassName() + " (5))");
	}
	return t;
}

template<typename SimpleType, typename RType>
SimpleType Session::getValue(CUrl url)
	throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet)
{
	Property* p = getProperty(url);	// this throws InvalidOperation and NoSuchProperty
	Profiler::lock((sessionName + ":" + url).c_str());
	p->lock(HERE);
	if (!p->getObjectL()) {
		p->unlock();
		Profiler::unlock((sessionName + ":" + url).c_str());
		throw ValueNotSet("Property '" + url + "' is not set.");
	}
	const RType* rt = 0;
	try {
		rt = p->getObjectAsLC<RType>();	// this throws WrongType
	}
	catch (const SessionException& e) {
		p->unlock();
		Profiler::unlock((sessionName + ":" + url).c_str());
		throw(e);
	}
	if (!rt) {
		p->unlock();
		Profiler::unlock((sessionName + ":" + url).c_str());
		throw InvalidOperation("Something strange in getting the object of property '"
			+ url + "'");
	}
	SimpleType st = (SimpleType) *rt;
	p->unlock();
	Profiler::unlock((sessionName + ":" + url).c_str());
	return st;
}
