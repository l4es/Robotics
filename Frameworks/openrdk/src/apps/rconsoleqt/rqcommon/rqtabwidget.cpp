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

#include "rqtabwidget.h"

#include <qdragobject.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RqTabWidget"

#include "rqcommon.h"

namespace RDK2 { namespace RConsoleQt {

void RqTabWidget::dragEnterEvent(QDragEnterEvent* event)
{
	event->accept(QTextDrag::canDecode(event));
}

void RqTabWidget::dropEvent(QDropEvent* event)
{
	QString qsUrl;
	if (QTextDrag::decode(event, qsUrl)) {
		Session* guiSession = RqCommon::getGuiSession("");
		int winId = -1;
		SESSION_TRY_START(guiSession)
		guiSession->lock(RqCommon::getGuiStateUrl(), HERE);
		RGuiState* gs = guiSession->getObjectAsL<RGuiState>(RqCommon::getGuiStateUrl());
		for (Vector<RGuiWindow>::iterator wi = gs->windows.begin(); wi != gs->windows.end(); ++wi) {
			RGuiWindow& w = *(*wi);
			if (w.qWindow == this) { winId = w.id; break; }
		}
		guiSession->unlock(RqCommon::getGuiStateUrl());
		SESSION_END_CATCH_TERMINATE(guiSession)

		RqCommon::createDefaultViewerFor(qsUrl.latin1(), event->pos(), winId);
	}
}

}}
