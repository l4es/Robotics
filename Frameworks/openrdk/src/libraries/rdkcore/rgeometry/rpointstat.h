
#ifndef  RPOINTSTAT_INC
#define  RPOINTSTAT_INC

#include <rdkcore/object/object.h>
#include <rdkcore/geometry/pointstat.h>

namespace RDK2 { namespace RGeometry {
struct RPoint2dCov: Object, RDK2::Geometry::Point2dCov
{
	RDK2_DEFAULT_CLONE(RPoint2dCov);
	RPoint2dCov(double x, double y, double xx, double yy, double xy);
	RPoint2dCov(const RDK2::Geometry::Point2dCov& p=RDK2::Geometry::Point2dCov());
	virtual void read(Reader*r) throw (ReadingException);
	virtual void write(Writer*w) const throw (WritingException);

	bool hasStringRepresentation() const { return true; }
	std::string getStringRepresentation() const;
	bool loadFromStringRepresentation(const std::string&);

	bool hasStringForVisualization() const { return true; }
	std::string getStringForVisualization() const;

};

struct RPolarPointCov: Object, RDK2::Geometry::PolarPointCov
{
	RDK2_DEFAULT_CLONE(RPolarPointCov);
	RPolarPointCov(double theta, double rho, double tt, double rr, double tr);
	RPolarPointCov(const RDK2::Geometry::PolarPointCov& p=RDK2::Geometry::PolarPointCov());
	virtual void read(Reader*r) throw (ReadingException);
	virtual void write(Writer*w) const throw (WritingException);

	bool hasStringRepresentation() const { return true; }
	std::string getStringRepresentation() const;
	bool loadFromStringRepresentation(const std::string&);

	bool hasStringForVisualization() const { return true; }
	std::string getStringForVisualization() const;

};

struct RPoint2odCov: Object, RDK2::Geometry::Point2odCov
{
	RDK2_DEFAULT_CLONE(RPoint2odCov);
	RPoint2odCov(double x, double y, double theta, double xx, double yy, double xy, double tt, double xt, double yt);
	RPoint2odCov(const RDK2::Geometry::Point2odCov& p=RDK2::Geometry::Point2odCov());
	virtual void read(Reader*r) throw (ReadingException);
	virtual void write(Writer*w) const throw (WritingException);

	bool hasStringRepresentation() const { return true; }
	std::string getStringRepresentation() const;
	bool loadFromStringRepresentation(const std::string&);

	bool hasStringForVisualization() const { return true; }
	std::string getStringForVisualization() const;

};

}}
#endif   // ----- #ifndef RPOINTSTAT_INC  -----
