/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <sys/time.h>
#include <cmath>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include "stat.h"

namespace RDK2 { namespace Statistics {

	gsl_rng * r = NULL;

	void initRNG() {
		if(r==NULL) {
			gsl_rng_env_setup();
			r = gsl_rng_alloc (gsl_rng_default);
		}
	}
	
	unsigned long int setRandomSeed() {
		struct timeval time;
		gettimeofday(&time,NULL);
		unsigned long int seed = (unsigned long int) time.tv_usec;
		setSeed(seed);
		return seed;
	}
	
	void setSeed(unsigned long int seed) {
		initRNG();
		gsl_rng_set(r, seed);
	}
	
	double sampleGaussianSigma(double sigma) {
		initRNG();
		return gsl_ran_gaussian (r,sigma);
	}
	
	double sampleUniform(double max) {
		initRNG();
		return max*gsl_rng_uniform (r);
	}
	
	double sampleUniform(double center, double range) {
		initRNG();
		return (2*range)*gsl_rng_uniform (r)+center-range;
	}
		
	double gaussian(double x, double mean, double sigma){
		return (1./sqrt(2.*M_PI*sigma*sigma))*exp(-((x-mean)*(x-mean))/(2*sigma*sigma));
	}

	double logGaussian(double x, double mean, double sigma){
		return -.5*log(2.*M_PI*sigma*sigma)-.5*(x-mean)*(x-mean)/(sigma*sigma);
	}

}}
