#ifndef STAT_H
#define STAT_H

#include <vector>
#include <rdkcore/geometry/point.h>
#include <rdkcore/geometry/otherpoints.h>

/**
 * This is taken from GMapping distribution
 * plus some OpenRDK adjustment
 * */

/**stupid utility function for drawing particles form a zero mean, sigma variance normal distribution
probably it should not go there*/
//double sampleGaussian(double sigma,unsigned long int S=0);

namespace RDK2 { namespace Geometry {
struct Covariance2
{
	Covariance2();
	Covariance2(double xx, double yy, double xy);
	Covariance2 operator + (const Covariance2 & cov) const;
	Covariance2 operator += (const Covariance2 & cov);
	static const Covariance2 zero;
	double xx, yy, xy;
};

struct EigenCovariance2
{
	EigenCovariance2();
	EigenCovariance2(const Covariance2& c);
	EigenCovariance2 rotate(double angle) const;
	RDK2::Geometry::Point2d sample() const;
	double eval[2];
	double evec[2][2];
};

struct Gaussian2
{
	RDK2::Geometry::Point2d mean;
	EigenCovariance2 covariance;
	Covariance2 cov;
	double eval(const RDK2::Geometry::Point2d& p) const;
};

struct Covariance3: public Covariance2
{
	Covariance3();
	Covariance3(
			double xx, double yy, double xy,
			double tt, double xt, double yt
			);
	Covariance3 operator + (const Covariance3 & cov) const;
	Covariance3 operator += (const Covariance3 & cov);
	static const Covariance3 zero;
	double tt, xt, yt;
};

struct EigenCovariance3
{
	EigenCovariance3();
	EigenCovariance3(const Covariance3& c);
	EigenCovariance3 rotate(double angle) const;
	RDK2::Geometry::Point2od sample() const;
	double eval[3];
	double evec[3][3];
};

struct Gaussian3
{
	RDK2::Geometry::Point2od mean;
	EigenCovariance3 covariance;
	Covariance3 cov;
	double eval(const RDK2::Geometry::Point2od& p) const;
	//void computeFromSamples(const std::vector<RDK2::Geometry::Point2od> & poses);
	//void computeFromSamples(const std::vector<RDK2::Geometry::Point2od> & poses, const std::vector<double>& weights );
};

template<typename Point, typename Covariance>
struct pointCov: public Point
{
	pointCov():
		Point(),
		cov()
	{}
	
	pointCov(const Point& p, const Covariance& c):
		Point(p),
		cov(c)
	{}
	
	Covariance cov;
};

typedef pointCov<Point2d,Covariance2> Point2dCov;
typedef pointCov<PolarPoint,Covariance2> PolarPointCov;
typedef pointCov<Point2od,Covariance3> Point2odCov;


std::ostream& operator<<(std::ostream& os, const Covariance2& p);
std::ostream& operator<<(std::ostream& os, const Covariance3& p);
std::istream& operator>>(std::istream& is, Covariance2& p);
std::istream& operator>>(std::istream& is, Covariance3& p);


template <typename Point, typename Covariance>
std::ostream& operator<<(std::ostream& os, const pointCov<Point,Covariance>& p)
{
	return os << " " << static_cast<Point>(p) << " " << p.cov << " ";
}

template <typename Point, typename Covariance>
std::istream& operator>>(std::istream& is, pointCov<Point,Covariance>& p)
{
	is >> static_cast<Point>(p);
	is >> p.cov;
		
	return is;
}

//template<typename PointIterator, typename WeightIterator>
//Gaussian3 computeGaussianFromSamples(PointIterator& pointBegin, PointIterator& pointEnd, WeightIterator& weightBegin, WeightIterator& weightEnd){
//  Gaussian3 gaussian;
//  RDK2::Geometry::Point2od mean=RDK2::Geometry::Point2od(0,0,0);
//  double wcum=0;
//  double s=0, c=0;
//  WeightIterator wt=weightBegin;
//  double *w=new double();
//  RDK2::Geometry::Point2od *p=new RDK2::Geometry::Point2od();
//  for (PointIterator pt=pointBegin; pt!=pointEnd; pt++){
//    *w=*wt;
//    *p=*pt;
//    s+=*w*sin(p->theta);
//    c+=*w*cos(p->theta);
//    mean.x+=*w*p->x;
//    mean.y+=*w*p->y;
//    wcum+=*w;
//    wt++;
//  }
//  mean.x/=wcum;
//  mean.y/=wcum;
//  s/=wcum;
//  c/=wcum;
//  mean.theta=atan2(s,c);

//  Covariance3 cov=Covariance3::zero;
//  wt=weightBegin;
//  for (PointIterator pt=pointBegin; pt!=pointEnd; pt++){
//    *w=*wt;
//    *p=*pt;
//    RDK2::Geometry::Point2od delta=(*p)-mean;
//    delta.theta=atan2(sin(delta.theta),cos(delta.theta));
//    cov.xx+=*w*delta.x*delta.x;
//    cov.yy+=*w*delta.y*delta.y;
//    cov.tt+=*w*delta.theta*delta.theta;
//    cov.xy+=*w*delta.x*delta.y;
//    cov.yt+=*w*delta.y*delta.theta;
//    cov.xt+=*w*delta.x*delta.theta;
//    wt++;
//  }
//  cov.xx/=wcum;
//  cov.yy/=wcum;
//  cov.tt/=wcum;
//  cov.xy/=wcum;
//  cov.yt/=wcum;
//  cov.xt/=wcum;
//  EigenCovariance3 ecov(cov);
//  gaussian.mean=mean;
//  gaussian.covariance=ecov;
//  gaussian.cov=cov;
//  delete w;
//  delete p;
//  return gaussian;
//}

//template<typename PointIterator>
//Gaussian3 computeGaussianFromSamples(PointIterator& pointBegin, PointIterator& pointEnd){
//  Gaussian3 gaussian;
//  RDK2::Geometry::Point2od mean=RDK2::Geometry::Point2od(0,0,0);
//  double wcum=0;
//  double s=0, c=0;
//  RDK2::Geometry::Point2od *p=new RDK2::Geometry::Point2od();
//  for (PointIterator pt=pointBegin; pt!=pointEnd; pt++){
//    *p=*pt;
//    s+=sin(p->theta);
//    c+=cos(p->theta);
//    mean.x+=p->x;
//    mean.y+=p->y;
//    wcum+=1.;
//  }
//  mean.x/=wcum;
//  mean.y/=wcum;
//  s/=wcum;
//  c/=wcum;
//  mean.theta=atan2(s,c);

//  Covariance3 cov=Covariance3::zero;
//  for (PointIterator pt=pointBegin; pt!=pointEnd; pt++){
//    *p=*pt;
//    RDK2::Geometry::Point2od delta=(*p)-mean;
//    delta.theta=atan2(sin(delta.theta),cos(delta.theta));
//    cov.xx+=delta.x*delta.x;
//    cov.yy+=delta.y*delta.y;
//    cov.tt+=delta.theta*delta.theta;
//    cov.xy+=delta.x*delta.y;
//    cov.yt+=delta.y*delta.theta;
//    cov.xt+=delta.x*delta.theta;
//  }
//  cov.xx/=wcum;
//  cov.yy/=wcum;
//  cov.tt/=wcum;
//  cov.xy/=wcum;
//  cov.yt/=wcum;
//  cov.xt/=wcum;
//  EigenCovariance3 ecov(cov);
//  gaussian.mean=mean;
//  gaussian.covariance=ecov;
//  gaussian.cov=cov;
//  delete p;
//  return gaussian;
//}

}}

#endif

