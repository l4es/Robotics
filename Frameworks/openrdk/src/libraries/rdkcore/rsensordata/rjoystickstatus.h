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

#ifndef RDK2_RSENSORS_RJOYSTICKSTATUS
#define RDK2_RSENSORS_RJOYSTICKSTATUS

#include <rdkcore/geometry/point.h>
#include <rdkcore/object/object.h>
#include <rdkcore/time/time.h>
#include <rdkcore/container/container.h>
#include <rdkcore/rprimitive/rdouble.h>
#include <rdkcore/rprimitive/rbool.h>
#include <string>

namespace RDK2 { namespace RSensorData {

using namespace RDK2::Containers;
using namespace RDK2::RPrimitive;

/**
 * This class contains the data read from a joystick
 *
 * @ingroup RDKLibraries
 */	
class RJoystickStatus : public RDK2::Object {
private:
	/// Axes' status. Each is a 16-bit signed value
	//Vector<RDouble>* axes;
	vector<double> axes;
	/// Buttons' status. True means pressed
	//Vector<RBool>* buttons;
	vector<bool> buttons;

public: 
	/// Number of axes
	unsigned int axesNumber;
	
	/// Number of buttons
	unsigned int buttonsNumber;

	/**
	 * Return axis value in range -1...1
	 *
	 * Throws a string in case of error
	 */
	double getAxis(unsigned int axis) const throw (string);
	
	/**
	 * Return button value. True means pressed.
	 *
	 * Throws a string in case of error
	 */
	bool getButton(unsigned int button) const throw(string);

	/**
	 * This constructor just creates the vectors
	 */
	RJoystickStatus();
	
	/**
	 * This constructor takes axes' and buttons' values
	 */
	RJoystickStatus(const vector<double> &axesData, 
			const vector<bool> &buttonsData);

	/// Destructor: frees vectors
	virtual ~RJoystickStatus();

	///@name Serialization
	//@{
	void read(Reader* r) throw (ReadingException);
	void write(Writer* w) const throw (WritingException);
	//@}

	/**
	 * @name String for visualization
	 */
	bool hasStringForVisualization() const { return true; }
	string getStringForVisualization() const;

	RDK2_DEFAULT_CLONE(RJoystickStatus);
};

}} // namespaces

#endif
