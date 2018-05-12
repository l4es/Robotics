#include "pointstat.h"
#include <cstdlib>
#include <rdkcore/stat/stat.h>

#include <gsl/gsl_eigen.h>
#include <gsl/gsl_blas.h>

namespace RDK2 { namespace Geometry {

using namespace RDK2::Statistics;
using RDK2::Geometry::Point2d;
using RDK2::Geometry::Point2od;

Covariance2::Covariance2():
	xx(0.),
	yy(0.),
	xy(0.)
{
}

Covariance2::Covariance2(double _xx, double _yy, double _xy):
	xx(_xx),
	yy(_yy),
	xy(_xy)
{
}

const Covariance2 Covariance2::zero=Covariance2();

Covariance2 Covariance2::operator + (const Covariance2 & cov) const
{
	return Covariance2(*this) += cov;
}

Covariance2 Covariance2::operator += (const Covariance2 & cov)
{
	xx += cov.xx;
	yy += cov.yy;
	xy += cov.xy;
	return *this;
}

EigenCovariance2::EigenCovariance2()
{}

EigenCovariance2::EigenCovariance2(const Covariance2& cov)
{
	static gsl_eigen_symmv_workspace * m_eigenspace=NULL;
	static gsl_matrix * m_cmat=NULL;
	static gsl_matrix * m_evec=NULL;
	static gsl_vector * m_eval=NULL;
	static gsl_vector * m_noise=NULL;
	static gsl_vector * m_pnoise=NULL;

	if (m_eigenspace==NULL)
	{
		m_eigenspace=gsl_eigen_symmv_alloc(2);
		m_cmat=gsl_matrix_alloc(2,2);
		m_evec=gsl_matrix_alloc(2,2);
		m_eval=gsl_vector_alloc(2);
		m_noise=gsl_vector_alloc(2);
		m_pnoise=gsl_vector_alloc(2);
	}

	gsl_matrix_set(m_cmat,0,0,cov.xx); gsl_matrix_set(m_cmat,0,1,cov.xy);
	gsl_matrix_set(m_cmat,1,0,cov.xy); gsl_matrix_set(m_cmat,1,1,cov.yy);
	gsl_eigen_symmv (m_cmat, m_eval,  m_evec, m_eigenspace);
	for (int i=0; i<2; i++)
	{
		eval[i]=gsl_vector_get(m_eval,i);
		for (int j=0; j<2; j++)
			evec[i][j]=gsl_matrix_get(m_evec,i,j);
	}
	m_noise  = m_noise;
	m_pnoise = m_pnoise;
}

EigenCovariance2 EigenCovariance2::rotate(double angle) const
{
	static gsl_matrix * m_rmat=NULL;
	static gsl_matrix * m_vmat=NULL;
	static gsl_matrix * m_result=NULL;
	if (m_rmat==NULL)
	{
		m_rmat=gsl_matrix_alloc(2,2);
		m_vmat=gsl_matrix_alloc(2,2);
		m_result=gsl_matrix_alloc(2,2);
	}

	double c=cos(angle);
	double s=sin(angle);
	gsl_matrix_set(m_rmat,0,0, c ); gsl_matrix_set(m_rmat,0,1, -s);
	gsl_matrix_set(m_rmat,1,0, s ); gsl_matrix_set(m_rmat,1,1,  c);

	for (unsigned int i=0; i<2; i++)
		for (unsigned int j=0; j<2; j++)
			gsl_matrix_set(m_vmat,i,j,evec[i][j]);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1., m_rmat, m_vmat, 0., m_result);
	EigenCovariance2 ecov(*this);
	for (int i=0; i<2; i++){
		for (int j=0; j<2; j++)
			ecov.evec[i][j]=gsl_matrix_get(m_result,i,j);
	}
	return ecov;
}

Point2d EigenCovariance2::sample() const
{
	static gsl_matrix * m_evec=NULL;
	static gsl_vector * m_noise=NULL;
	static gsl_vector * m_pnoise=NULL;
	if (m_evec==NULL){
		m_evec=gsl_matrix_alloc(2,2);
		m_noise=gsl_vector_alloc(2);
		m_pnoise=gsl_vector_alloc(2);
	}
	for (int i=0; i<2; i++){
		for (int j=0; j<2; j++)
			gsl_matrix_set(m_evec,i,j, evec[i][j]);
	}
	for (int i=0; i<2; i++){
		double v=sampleGaussianSigma(sqrt(eval[i]));
		if(isnan(v))
			v=0;
		gsl_vector_set(m_pnoise,i, v);
	}
	gsl_blas_dgemv (CblasNoTrans, 1., m_evec, m_pnoise, 0, m_noise);
	Point2d ret(gsl_vector_get(m_noise,0),gsl_vector_get(m_noise,1));
	m_noise  = m_noise;
	m_pnoise = m_pnoise;
	return ret;
}


double Gaussian2::eval(const Point2d& p) const
{
	Point2d q=p-mean;
	double v1,v2;
	v1 = covariance.evec[0][0]*q.x+covariance.evec[1][0]*q.y;
	v2 = covariance.evec[0][1]*q.x+covariance.evec[1][1]*q.y;
	return logGaussian(0,covariance.eval[0], sqrt(v1))+logGaussian(0, covariance.eval[1], sqrt(v2));
}

Covariance3::Covariance3():
	Covariance2(),
	tt(0.),
	xt(0.),
	yt(0.)
{
}

Covariance3::Covariance3(double _xx, double _yy, double _xy,
		double _tt, double _xt, double _yt):
	Covariance2(_xx,_yy,_xy),
	tt(_tt),
	xt(_xt),
	yt(_yt)
{
}

const Covariance3 Covariance3::zero=Covariance3();

Covariance3 Covariance3::operator + (const Covariance3 & cov) const
{
	return Covariance3(*this) += cov;
}

Covariance3 Covariance3::operator += (const Covariance3 & cov)
{
	Covariance2(*this) += cov;
	tt += cov.tt;
	xt += cov.xt;
	yt += cov.yt;
	return *this;
}

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
#if 0
double pf_ran_gaussian(double sigma)
{
	double x1, x2, w;
	double r;

	do
	{
		do { r = drand48(); } while (r == 0.0);
		x1 = 2.0 * r - 1.0;
		do { r = drand48(); } while (r == 0.0);
		x2 = 2.0 * drand48() - 1.0;
		w = x1*x1 + x2*x2;
	} while(w > 1.0 || w==0.0);

		return(sigma * x2 * sqrt(-2.0*log(w)/w));
}

double sampleGaussian(double sigma, unsigned long int S) {
	/*
		 static gsl_rng * r = NULL;
		 if(r==NULL) {
		 gsl_rng_env_setup();
		 r = gsl_rng_alloc (gsl_rng_default);
		 }
		 */
	if (S!=0)
	{
		//gsl_rng_set(r, S);
		srand48(S);
	}
	if (sigma==0)
		return 0;
	//return gsl_ran_gaussian (r,sigma);
	return pf_ran_gaussian (sigma);
}
#endif

EigenCovariance3::EigenCovariance3()
{}

EigenCovariance3::EigenCovariance3(const Covariance3& cov)
{
	static gsl_eigen_symmv_workspace * m_eigenspace=NULL;
	static gsl_matrix * m_cmat=NULL;
	static gsl_matrix * m_evec=NULL;
	static gsl_vector * m_eval=NULL;
	static gsl_vector * m_noise=NULL;
	static gsl_vector * m_pnoise=NULL;

	if (m_eigenspace==NULL)
	{
		m_eigenspace=gsl_eigen_symmv_alloc(3);
		m_cmat=gsl_matrix_alloc(3,3);
		m_evec=gsl_matrix_alloc(3,3);
		m_eval=gsl_vector_alloc(3);
		m_noise=gsl_vector_alloc(3);
		m_pnoise=gsl_vector_alloc(3);
	}

	gsl_matrix_set(m_cmat,0,0,cov.xx); gsl_matrix_set(m_cmat,0,1,cov.xy); gsl_matrix_set(m_cmat,0,2,cov.xt);
	gsl_matrix_set(m_cmat,1,0,cov.xy); gsl_matrix_set(m_cmat,1,1,cov.yy); gsl_matrix_set(m_cmat,1,2,cov.yt);
	gsl_matrix_set(m_cmat,2,0,cov.xt); gsl_matrix_set(m_cmat,2,1,cov.yt); gsl_matrix_set(m_cmat,2,2,cov.tt);
	gsl_eigen_symmv (m_cmat, m_eval,  m_evec, m_eigenspace);
	for (int i=0; i<3; i++)
	{
		eval[i]=gsl_vector_get(m_eval,i);
		for (int j=0; j<3; j++)
			evec[i][j]=gsl_matrix_get(m_evec,i,j);
	}
	m_noise  = m_noise;
	m_pnoise = m_pnoise;
}

EigenCovariance3 EigenCovariance3::rotate(double angle) const
{
	static gsl_matrix * m_rmat=NULL;
	static gsl_matrix * m_vmat=NULL;
	static gsl_matrix * m_result=NULL;
	if (m_rmat==NULL)
	{
		m_rmat=gsl_matrix_alloc(3,3);
		m_vmat=gsl_matrix_alloc(3,3);
		m_result=gsl_matrix_alloc(3,3);
	}

	double c=cos(angle);
	double s=sin(angle);
	gsl_matrix_set(m_rmat,0,0, c ); gsl_matrix_set(m_rmat,0,1, -s); gsl_matrix_set(m_rmat,0,2, 0.);
	gsl_matrix_set(m_rmat,1,0, s ); gsl_matrix_set(m_rmat,1,1,  c); gsl_matrix_set(m_rmat,1,2, 0.);
	gsl_matrix_set(m_rmat,2,0, 0.); gsl_matrix_set(m_rmat,2,1, 0.); gsl_matrix_set(m_rmat,2,2, 1.);

	for (unsigned int i=0; i<3; i++)
		for (unsigned int j=0; j<3; j++)
			gsl_matrix_set(m_vmat,i,j,evec[i][j]);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1., m_rmat, m_vmat, 0., m_result);
	EigenCovariance3 ecov(*this);
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++)
			ecov.evec[i][j]=gsl_matrix_get(m_result,i,j);
	}
	return ecov;
}

Point2od EigenCovariance3::sample() const
{
	static gsl_matrix * m_evec=NULL;
	static gsl_vector * m_noise=NULL;
	static gsl_vector * m_pnoise=NULL;
	if (m_evec==NULL){
		m_evec=gsl_matrix_alloc(3,3);
		m_noise=gsl_vector_alloc(3);
		m_pnoise=gsl_vector_alloc(3);
	}
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++)
			gsl_matrix_set(m_evec,i,j, evec[i][j]);
	}
	for (int i=0; i<3; i++){
		double v=sampleGaussianSigma(sqrt(eval[i]));
		if(isnan(v))
			v=0;
		gsl_vector_set(m_pnoise,i, v);
	}
	gsl_blas_dgemv (CblasNoTrans, 1., m_evec, m_pnoise, 0, m_noise);
	Point2od ret(gsl_vector_get(m_noise,0),gsl_vector_get(m_noise,1),gsl_vector_get(m_noise,2));
	ret.theta=atan2(sin(ret.theta), cos(ret.theta));
	return ret;
}

#if 0
double Gaussian3::eval(const Point2od& p) const
{
	Point2od q=p-mean;
	q.theta=atan2(sin(p.theta-mean.theta),cos(p.theta-mean.theta));
	double v1,v2,v3;
	v1 = covariance.evec[0][0]*q.x+covariance.evec[1][0]*q.y+covariance.evec[2][0]*q.theta;
	v2 = covariance.evec[0][1]*q.x+covariance.evec[1][1]*q.y+covariance.evec[2][1]*q.theta;
	v3 = covariance.evec[0][2]*q.x+covariance.evec[1][2]*q.y+covariance.evec[2][2]*q.theta;
	return logGaussian(0,covariance.eval[0], sqrt(v1))+logGaussian(0, covariance.eval[1], sqrt(v2))+logGaussian(0,covariance.eval[2], sqrt(v3));
}
#endif

#if 0
void Gaussian3::computeFromSamples(const std::vector<Point2od> & poses, const std::vector<double>& weights ){
	Point2od mean=Point2od(0,0,0);
	double wcum=0;
	double s=0, c=0;
	std::vector<double>::const_iterator w=weights.begin();
	for (std::vector<Point2od>::const_iterator p=poses.begin(); p!=poses.end(); p++){
		s+=*w*sin(p->theta);
		c+=*w*cos(p->theta);
		mean.x+=*w*p->x;
		mean.y+=*w*p->y;
		wcum+=*w;
		w++;
	}
	mean.x/=wcum;
	mean.y/=wcum;
	s/=wcum;
	c/=wcum;
	mean.theta=atan2(s,c);

	Covariance3 cov=Covariance3::zero;
	w=weights.begin();
	for (std::vector<Point2od>::const_iterator p=poses.begin(); p!=poses.end(); p++){
		Point2od delta=(*p)-mean;
		delta.theta=atan2(sin(delta.theta),cos(delta.theta));
		cov.xx+=*w*delta.x*delta.x;
		cov.yy+=*w*delta.y*delta.y;
		cov.tt+=*w*delta.theta*delta.theta;
		cov.xy+=*w*delta.x*delta.y;
		cov.yt+=*w*delta.y*delta.theta;
		cov.xt+=*w*delta.x*delta.theta;
		w++;
	}
	cov.xx/=wcum;
	cov.yy/=wcum;
	cov.tt/=wcum;
	cov.xy/=wcum;
	cov.yt/=wcum;
	cov.xt/=wcum;
	EigenCovariance3 ecov(cov);
	this->mean=mean;
	this->covariance=ecov;
	this->cov=cov;
}

void Gaussian3::computeFromSamples(const std::vector<Point2od> & poses){
	Point2od mean=Point2od(0,0,0);
	double wcum=1;
	double s=0, c=0;
	for (std::vector<Point2od>::const_iterator p=poses.begin(); p!=poses.end(); p++){
		s+=sin(p->theta);
		c+=cos(p->theta);
		mean.x+=p->x;
		mean.y+=p->y;
		wcum+=1.;
	}
	mean.x/=wcum;
	mean.y/=wcum;
	s/=wcum;
	c/=wcum;
	mean.theta=atan2(s,c);

	Covariance3 cov=Covariance3::zero;
	for (std::vector<Point2od>::const_iterator p=poses.begin(); p!=poses.end(); p++){
		Point2od delta=(*p)-mean;
		delta.theta=atan2(sin(delta.theta),cos(delta.theta));
		cov.xx+=delta.x*delta.x;
		cov.yy+=delta.y*delta.y;
		cov.tt+=delta.theta*delta.theta;
		cov.xy+=delta.x*delta.y;
		cov.yt+=delta.y*delta.theta;
		cov.xt+=delta.x*delta.theta;
	}
	cov.xx/=wcum;
	cov.yy/=wcum;
	cov.tt/=wcum;
	cov.xy/=wcum;
	cov.yt/=wcum;
	cov.xt/=wcum;
	EigenCovariance3 ecov(cov);
	this->mean=mean;
	this->covariance=ecov;
	this->cov=cov;
}
#endif

std::ostream& operator<<(std::ostream& os, const Covariance2& p)
{
	return os << " " << p.xx << " " << p.yy << " " << p.xy << " ";
}

std::ostream& operator<<(std::ostream& os, const Covariance3& p)
{
	return os << static_cast<Covariance2>(p) << p.tt << " " << p.xt << " " << p.yt << " ";
}

std::istream& operator>>(std::istream& is, Covariance2& p)
{
	is >> p.xx >> p.yy >> p.xy;
	return is;
}

std::istream& operator>>(std::istream& is, Covariance3& p)
{
	is >> p.xx >> p.yy >> p.xy;
	is >> p.tt >> p.xt >> p.yt;
	return is;
}



}}
