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

#ifndef RDK2_RQM_RQMODULE
#define RDK2_RQM_RQMODULE

#include <rdkcore/modules/module.h>

#include <qobject.h>

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RAgent;

class RqModule : public QObject, public Module {
public:
	RqModule() : widgetReady(false) { }

	/** put
	 *  if (!RqModule::init()) return false;
	 *  in the beginning of your init() function, if you reimplement it
	 */
	virtual bool init();
	
	/** put
	 *  WIDGET_GUARD
	 *  in the beginning of your exec() function, before the while(session->wait(), !exiting) loop,
	 *  and, in any case, before doing anything with the widget; you can also avoid to reimplement this,
	 *  leaving this empty (no-action) implementation
	 */
	virtual void exec();
	
	/** put
	 *  RqModule::cleanup();
	 *  in the end of your cleanup() function, if your reimplement it
	 */
	virtual void cleanup();

protected:
	virtual void customEvent(QCustomEvent* e);
	virtual QWidget* createWidget() = 0;

	volatile bool widgetReady;
};

}}

#endif
