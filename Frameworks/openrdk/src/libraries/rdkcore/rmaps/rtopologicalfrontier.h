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

/**
 * @file
 *
 * @brief This file defines the class RTopologicalFrontier
 */

#ifndef RDK2_RMAPS_RTOPOLOGICALFRONTIER
#define RDK2_RMAPS_RTOPOLOGICALFRONTIER

#include <rdkcore/object/object.h>
#include <rdkcore/geometry/point.h>
#include <rdkcore/container/container.h>
#include <rdkcore/rgraphics/color.h>

#define DEFAULT_FRONTIER_SIMILARITY_DIST 0.5

namespace RDK2 { namespace RMaps {
	
using namespace RDK2::Geometry;

/**
 * This class represents a topological frontier, that is a magic thing
 * used for coordination.
 */
struct RTopologicalFrontier : public RDK2::Object {
	Point2d point0, point1;
	/// The middle point has also information about the direction
	/// where the known empty space is
	Point2od midpoint;

	/**
	 * Every class should have a default constructor
	 */
	RTopologicalFrontier() { };
	/**
	 * Constructor: takes the coordinates of the segment's end
	 * points. Assumes that, going from x0 to y0, the empty,
	 * known space is on the left.
	 */
	RTopologicalFrontier(double x0, double y0, double x1, double y1);
	inline bool isSimilarTo(const RTopologicalFrontier* topolFront, 
				double frontierSimilarityDist = 
				DEFAULT_FRONTIER_SIMILARITY_DIST) const
		{ return topolFront->midpoint.distTo(midpoint) < 
			  frontierSimilarityDist; }

	/// Returns true because this object has a string that can be
	/// visualized.
	bool hasStringForVisualization() const { return true; }
	/// Returns a string to be used to visualize the object.
	std::string getStringForVisualization() const;

	/// @name Serialization
	//@{
	void read(Reader* r) throw (ReadingException);
	void write(Writer* w) const throw (WritingException);
	//@}
	RDK2_DEFAULT_CLONE(RTopologicalFrontier);
};

 typedef RDK2::Containers::Vector<RTopologicalFrontier> 
	 RTopologicalFrontierVector;

}} // namespace

#endif
