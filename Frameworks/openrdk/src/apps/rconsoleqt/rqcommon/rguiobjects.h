/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi (daniele.calisi@dis.uniroma1.it)
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

#ifndef RDK2_RQ_RGUIOBJECTS
#define RDK2_RQ_RGUIOBJECTS

#include <rdkcore/container/container.h>
#include <rdkcore/rgeometry/rpoint2i.h>
#include <rdkcore/container/container.h>
#include <rdkcore/rprimitive/rstring.h>

#include <qtabwidget.h>

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::Containers;
using namespace RDK2::RPrimitive;
using namespace RDK2::RGeometry;

class RqTabWidget;

struct RGuiTab: public RDK2::Object {
	int id;
	string title;
	Vector<RString> moduleNames;
	QWidget* qTab;
	
	RGuiTab();
	~RGuiTab();
	void read(Reader*r) throw (ReadingException);
	void write(Writer*w) const throw (WritingException);
	RDK2::Object* clone() const;
};

struct RGuiWindow : public RDK2::Object {
	int id;
	string title;
	RPoint2i position, dimension;
	Vector<RGuiTab> tabs;
	RqTabWidget* qWindow;
	bool resizeMe;
	
	RGuiWindow();
	~RGuiWindow();
	void read(Reader*r) throw (ReadingException);
	void write(Writer*w) const throw (WritingException);
	RDK2::Object* clone() const;
};

struct RGuiState: public RDK2::Object {
	Vector<RGuiWindow> windows;
	
	RGuiState() { }
	~RGuiState() { windows.clear(true); }
	void read(Reader*r) throw (ReadingException);
	void write(Writer*w) const throw (WritingException);
	RDK2_DEFAULT_CLONE(RGuiState);
};

}}

#endif
