#include "rnaojoints.h"

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RNaoJoints"

using namespace std;

namespace RDK2 {

RDK2_FACTORY(RNaoJoints)

//Object* RNaoJoints::clone() const
//{
//  RNaoJoints* rnj = new RNaoJoints(*this);
//  return rnj;
//}

void RNaoJoints::read(RDK2::Serialization::Reader*r) throw (ReadingException)
{
	r->startReading(getClassName());
		
		status = r->read_i32();
		value.reserve(NAO_JOINTS_COUNT);
		//value.clear();
		for (size_t i=0; i<NAO_JOINTS_COUNT; ++i)
		{
			value[i] = r->read_f64();
		}
r->doneReading();
}

void RNaoJoints::write(RDK2::Serialization::Writer*w) const throw (WritingException)
{
	w->startWriting(getClassName());
		w->write_i32(status);
		for (size_t i=0; i<NAO_JOINTS_COUNT; ++i)
		{
			w->write_f64(value[i]);
		}
	w->doneWriting();
}

std::string RNaoJoints::getStringRepresentation() const
{
	std::ostringstream oss;
	for (size_t i=0; i<NAO_JOINTS_COUNT; ++i)
	{
		oss << " " << value[i];
	}
	return oss.str();
}

} // namespace
