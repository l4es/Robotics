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

#include "rjoystickstatus.h"

#include <stdio.h>

namespace RDK2 { namespace RSensorData {

RJoystickStatus::RJoystickStatus() { }

RJoystickStatus::RJoystickStatus(const vector<double> &axesData, const vector<bool> &buttonsData)
{
	vector<double>::const_iterator axIt;
	vector<bool>::const_iterator btIt;
	axesNumber = axesData.size();
	buttonsNumber = buttonsData.size();
	/*axes = new Vector<RDouble>;
	buttons = new Vector<RBool>;
	for (axIt = axesData.begin(); axIt != axesData.end(); axIt++) {
		axes->push_back(new RDouble(*axIt));
	}
	for (btIt = buttonsData.begin(); btIt != buttonsData.end();
			btIt++) {
		buttons->push_back(new RBool(*btIt));
	}*/
	axes = axesData;
	buttons = buttonsData;
}

RJoystickStatus::~RJoystickStatus()
{
	/*if (axes) axes->clear();
	if (buttons) buttons->clear();
	delete axes;
	delete buttons;*/
}

double RJoystickStatus::getAxis(unsigned int axis) const throw (string)
{
	double retval;
	if (axis <= axesNumber) {
		//retval = (*axes)[axis]->value;	// ((double)(*axes)[axis]->value / maxAxisValue);
		retval = axes[axis];
	} else {
		ostringstream oss;
		oss << "Invalid axis number: " << axis;
		throw oss.str();
	}
	return retval;
}

bool RJoystickStatus::getButton(unsigned int button) const throw (string)
{
	bool retval;
	if (button <= buttonsNumber) {
		//retval = (*buttons)[button]->value;
		retval = buttons[button];
	} else {
		ostringstream oss;
		oss << "Invalid button number: " << button;
		throw oss.str();
	}
	return retval;
}

void RJoystickStatus::read(Reader* r) throw (ReadingException)
{
	r->startReading(getClassName());
	axesNumber = r->read_i8();
	buttonsNumber = r->read_i8();
	//axes = (Vector<RDouble> *)r->readObject();
	//buttons = (Vector<RBool> *)r->readObject();
	r->doneReading();
}

void RJoystickStatus::write(Writer* w) const throw (WritingException)
{
	w->startWriting(getClassName());
	w->write_i8(axesNumber);
	w->write_i8(buttonsNumber);
	//w->writeObject(false, axes);
	//w->writeObject(false, buttons);
	w->doneWriting();
}

string RJoystickStatus::getStringForVisualization() const
{
	ostringstream oss;
	unsigned int i;
	oss << "Axes: (";
	for (i = 0; i < axesNumber; i++) {
		//oss << (*axes)[i]->value << ", ";
		oss << axes[i] << ",";
	}
	oss << ")  Buttons: (";
	for (i = 0; i < buttonsNumber; i++) {
		//oss << (*buttons)[i]->value << ", ";
		oss << buttons[i] << ",";
	}
	oss << ")";
	return oss.str();
}

RDK2_FACTORY(RJoystickStatus);
	
}} // namespace

