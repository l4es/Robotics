#include "colorobjecttable.h"
#include <algorithm>
#include <functional>

RObjectInfo::RObjectInfo(const ObjectInfo& o):
	ObjectInfo(o)
{
}

	void
RObjectInfo::read ( RDK2::Reader* r ) throw (RDK2::ReadingException)
{
	r->startReading(getClassName());
		readImpl(r);
	r->doneReading();
	return ;
}		/* -----  end of method RObjectInfo::read  ----- */

	void
RObjectInfo::write ( RDK2::Writer* w ) const throw (RDK2::WritingException)
{
	w->startWriting(getClassName());
		writeImpl(w);
	w->doneWriting();
	return ;
}		/* -----  end of method RObjectInfo::write  ----- */

	void
RObjectInfo::readImpl ( RDK2::Reader* r ) throw (RDK2::ReadingException)
{
	id     = r->readString();
	width  = r->read_f32();
	height = r->read_f32();
	depth  = r->read_f32();
	return ;
}		/* -----  end of method RObjectInfo::readImpl  ----- */

	void
RObjectInfo::writeImpl ( RDK2::Writer* w ) const throw (RDK2::WritingException)
{
	w->writeString(id);
	w->write_f32(width);
	w->write_f32(height);
	w->write_f32(depth);
	return ;
}		/* -----  end of method RObjectInfo::writeImpl  ----- */

//RDK2::Object* RColorObjectTable::clone()
//{
//  RColorObjectTable* rcot = new RColorObjectTable(*this);
//}

RColorObjectTable::RColorObjectTable(const ColorObjectTable& cot):
	ColorObjectTable(cot)
{}

void
RColorObjectTable::read ( RDK2::Reader* r ) throw (RDK2::ReadingException)
{
	r->startReading(getClassName());
	size_t s = r->read_i32();
	for (size_t i=0; i<s; ++i)
	{
		RgbColor c = r->read_i32();
		RObjectInfo o;
		o.readImpl(r);
		insert(std::make_pair(c,o));
	}
	r->doneReading();
	return ;
}		/* -----  end of method RColorObjectTable::read  ----- */

template<typename W, typename V>
struct writeobject: std::binary_function<W,V,void>
{
	void operator() (W w, V v) const
	{
		w->write_i32(v.first);
		RObjectInfo ro(v.second);
		ro.writeImpl(w);
	}
};

void 
RColorObjectTable::write ( RDK2::Writer* w ) const throw (RDK2::WritingException)
{
	w->startWriting(getClassName());
	w->write_i32(size());
	for_each(begin(),end(),std::bind1st(writeobject<RDK2::Writer*,ColorObjectTable::value_type>(),w));
	w->doneWriting();
	return ;
}		/* -----  end of method RColorObjectTable::write  ----- */


RDK2_FACTORY(RColorObjectTable)
