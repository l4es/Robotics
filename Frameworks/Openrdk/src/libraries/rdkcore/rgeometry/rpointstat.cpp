#include "rpointstat.h"
#include <iomanip>
#include <rdkcore/geometry/angle.h>
#include <float.h>

using namespace RDK2::Geometry;

namespace RDK2 { namespace RGeometry {

	using RDK2::Geometry::Point2dCov;
	using RDK2::Geometry::Point2odCov;
	using RDK2::Geometry::PolarPointCov;
	using RDK2::Geometry::Covariance2;
	using RDK2::Geometry::Covariance3;

	RDK2_FACTORY(RPoint2dCov);
	RDK2_FACTORY(RPolarPointCov);
	RDK2_FACTORY(RPoint2odCov);

	RPoint2dCov::RPoint2dCov(double x, double y, double xx, double yy, double xy):
		Point2dCov(RDK2::Geometry::Point2d(x,y), Covariance2(xx, yy, xy))
	{
	}

	RPoint2dCov::RPoint2dCov(const Point2dCov& p):
		Point2dCov(p)
	{
	}

	void RPoint2dCov::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
		x = r->read_f32();
		y = r->read_f32();
		cov.xx = r->read_f32();
		cov.yy = r->read_f32();
		cov.xy = r->read_f32();
		r->doneReading();
	}

	void RPoint2dCov::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
		w->write_f32(x);
		w->write_f32(y);
		w->write_f32(cov.xx);
		w->write_f32(cov.yy);
		w->write_f32(cov.xy);
		w->doneWriting();
	}

	std::string RPoint2dCov::getStringRepresentation() const
	{
		ostringstream oss;
		oss << static_cast<Point2d>(*this);
		return oss.str();
	}

	bool RPoint2dCov::loadFromStringRepresentation(const std::string& s)
	{
		istringstream iss(s);
		iss >> x >> y >> cov;
		return iss;
	}

	std::string RPoint2dCov::getStringForVisualization() const
	{
		ostringstream oss;
		oss << setiosflags(ios::fixed) << setprecision(1);
		oss << static_cast<RDK2::Geometry::Point2d>(*this);
		return oss.str();
	}

	RPolarPointCov::RPolarPointCov(double theta, double rho, double tt, double rr, double tr):
		PolarPointCov(PolarPoint(theta, rho), Covariance2(tt, rr, tr))
	{
	}

	RPolarPointCov::RPolarPointCov(const PolarPointCov& p):
		PolarPointCov(p)
	{
	}

	void RPolarPointCov::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
		rho = r->read_f32();
		theta = r->read_f32();
		cov.xx = r->read_f32();
		cov.yy = r->read_f32();
		cov.xy = r->read_f32();
		r->doneReading();
	}

	void RPolarPointCov::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
		w->write_f32(rho);
		w->write_f32(theta);
		w->write_f32(cov.xx);
		w->write_f32(cov.yy);
		w->write_f32(cov.xy);
		w->doneWriting();
	}

	std::string RPolarPointCov::getStringRepresentation() const
	{
		ostringstream oss;
		oss << rad2deg(theta) << " " << rho;
		return oss.str();
	}

	bool RPolarPointCov::loadFromStringRepresentation(const std::string& s)
	{
		istringstream iss(s);
		string a;
		iss >> a >> rho;
		if (a == "inf") theta = DBL_MAX;
		else theta = deg2rad(atof(a.c_str()));
		return iss;
	}

	std::string RPolarPointCov::getStringForVisualization() const
	{
		ostringstream oss;
		oss << setiosflags(ios::fixed) << setprecision(1);
		oss << rad2deg(theta) << " " << rho;
		return oss.str();
	}

	RPoint2odCov::RPoint2odCov(double x, double y, double theta, double xx, double yy, double xy, double tt, double xt, double yt):
		Point2odCov(RDK2::Geometry::Point2od(x,y,theta), Covariance3(xx, yy, xy, tt, xt, yt))
	{
	}

	RPoint2odCov::RPoint2odCov(const Point2odCov& p):
		Point2odCov(p)
	{
	}

	void RPoint2odCov::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
		x = r->read_f32();
		y = r->read_f32();
		theta = r->read_f32();
		cov.xx = r->read_f32();
		cov.yy = r->read_f32();
		cov.xy = r->read_f32();
		cov.tt = r->read_f32();
		cov.xt = r->read_f32();
		cov.yt = r->read_f32();
		r->doneReading();
	}

	void RPoint2odCov::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
		w->write_f32(x);
		w->write_f32(y);
		w->write_f32(theta);
		w->write_f32(cov.xx);
		w->write_f32(cov.yy);
		w->write_f32(cov.xy);
		w->write_f32(cov.tt);
		w->write_f32(cov.xt);
		w->write_f32(cov.yt);
		w->doneWriting();
	}

	std::string RPoint2odCov::getStringRepresentation() const
	{
		ostringstream oss;
		oss << x << " " << y << " " << rad2deg(theta);
		return oss.str();
	}

	bool RPoint2odCov::loadFromStringRepresentation(const std::string& s)
	{
		istringstream iss(s);
		string a;
		iss >> x >> y >> a;
		if (a == "inf") theta = DBL_MAX;
		else theta = deg2rad(atof(a.c_str()));
		iss >> cov;
		return iss;
	}

	std::string RPoint2odCov::getStringForVisualization() const
	{
		ostringstream oss;
		oss << setiosflags(ios::fixed) << setprecision(1);
		oss << static_cast<Point2od>(*this);
		return oss.str();
	}

}}
