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

#include "rguiobjects.h"

namespace RDK2 { namespace RConsoleQt {

RDK2_FACTORY(RGuiState);
RDK2_FACTORY(RGuiWindow);
RDK2_FACTORY(RGuiTab);

int guiObjId = 0;

int getGuiObjectId() {
	return guiObjId++;
}

void RGuiState::read(Reader*r) throw (ReadingException)
{
	r->startReading("RGuiState");
	r->readObject(&windows, "windows");
	r->doneReading();
}

void RGuiState::write(Writer*w) const throw (WritingException)
{
	w->startWriting("RGuiState");
	w->writeObject(true, &windows, "windows");
	w->doneWriting();
}

RGuiWindow::RGuiWindow() : id(getGuiObjectId()), qWindow(0), resizeMe(false) { }
RGuiWindow::~RGuiWindow() { tabs.clear(true); }

RDK2::Object* RGuiWindow::clone() const
{
	RGuiWindow* obj = new RGuiWindow(*this);
	obj->id = getGuiObjectId();
	return obj;
}

void RGuiWindow::read(Reader*r) throw (ReadingException)
{
	r->startReading("RGuiWindow");
	title = r->readString("title");
	r->readObject(&dimension, "dimension");
	r->readObject(&position, "position");
	r->readObject(&tabs, "tabs");		
	r->doneReading();
}

void RGuiWindow::write(Writer*w) const throw (WritingException)
{
	w->startWriting("RGuiWindow");
	w->writeString(title, "title");
	w->writeObject(true,&dimension, "dimension");
	w->writeObject(true,&position, "position");
	w->writeObject(true,&tabs, "tabs");
	w->doneWriting();
}

RGuiTab::RGuiTab() : id(getGuiObjectId()), qTab(0){ }
RGuiTab::~RGuiTab() { moduleNames.clear(true); }
	
RDK2::Object* RGuiTab::clone() const
{
	RGuiTab* obj = new RGuiTab(*this);
	obj->id = getGuiObjectId();
	return obj;
}

void RGuiTab::read(Reader*r) throw (ReadingException)
{
	r->startReading("RGuiTab");
	title = r->readString("title");
	r->readObject(&moduleNames, "moduleNames");
	r->doneReading();
}

void RGuiTab::write(Writer*w) const throw (WritingException)
{
	w->startWriting("RGuiTab");
	w->writeString(title, "title");
	w->writeObject(true, &moduleNames, "moduleNames");
	w->doneWriting();
}

}} // ns

