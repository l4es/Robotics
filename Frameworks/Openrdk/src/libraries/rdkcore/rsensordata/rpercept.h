#ifndef  RPERCEPT_INC
#define  RPERCEPT_INC

#include <rdkcore/object/object.h>
#include <rdkcore/sensordata/percept.h>
#include <rdkcore/container/container.h>


namespace RDK2 { namespace RSensorData {


	struct RPercept: RDK2::Object, RDK2::SensorData::Percept
	{
		RDK2_DEFAULT_CLONE(RPercept);
		RPercept(const std::string& s="PCPTGEN"):
			Percept(s)
		{}

		virtual void read(Reader* r) throw (ReadingException);
		virtual void write(Writer* w) const throw (WritingException);
		virtual std::string getLogLine() const;
		virtual bool loadFromLogLine(cstr line);
	};				/* ----------  end of struct RPercept  ---------- */


	typedef RDK2::Containers::Vector<RPercept> RPerceptVector;

}}

#endif   /* ----- #ifndef RPERCEPT_INC  ----- */
