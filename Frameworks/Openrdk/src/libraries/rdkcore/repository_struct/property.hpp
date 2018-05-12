template <typename T>
const T* Property::getObjectAsLC() throw (ValueNotSet, WrongType)
{
	const RDK2::Object* obj = getObjectLC();
	if (!obj) throw ValueNotSet("Property is not set");
	const T* t = dynamic_cast<const T*>(obj);
	if (!t) {
		T t2;
		throw WrongType("Property has an object of type '" + obj->getClassName() + "', you asked for '" + t2.getClassName() + "'");
	}
	return t;
}

template <typename T>
T* Property::getObjectAsL() throw (ValueNotSet, WrongType)
{
	RDK2::Object* obj = getObjectL();
	if (!obj) throw ValueNotSet("Property is not set");
	T* t = dynamic_cast<T*>(obj);
	if (!t) {
		T t2;
		throw WrongType("Property has an object of type '" + obj->getClassName() + "', you asked for '" + t2.getClassName() + "'");
	}
	return t;
}
