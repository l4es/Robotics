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

#ifndef RDK2_STAT_H
#define RDK2_STAT_H

/// @file
///
/// @brief This is a wrapper around GSL for simple statistics functions.

namespace RDK2 { namespace Statistics {

	/// @name Random number generators
	//@{

	/// Sets a random seed using time and returns it
	unsigned long int setRandomSeed();
		
	/** Sets seed for random number generator. */
	void setSeed(unsigned long seed);
	
	/** Samples from a gaussian N(0,sigma^2) */
	double sampleGaussianSigma(double sigma);

	/** Samples from a uniform (0,max) */
	double sampleUniform(double max);
	
	/** Samples from a uniform (center-range, center+range) */
	double sampleUniform(double center, double range);

	//@}

	/** Evaluates gaussian probability density function N(mean,sigma^2) at x */
	double gaussian(double x, double mean, double sigma);

	/** Evaluates log gaussian probability density function N(mean,sigma^2) at x */
	double logGaussian(double x, double mean, double sigma);

}} // namespace RDK2::Statistics

#endif
