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

#include <vector>
#include <qpopupmenu.h>
#include <qapplication.h>

#include <rdkcore/textutils/textutils.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "TreeViewerWidget"

#include "../../rqcommon/rqcommon.h"
#include "../../rqcommon/rqpropertydragobject.h"
#include "treeviewerwidget.h"

#define POPUP_ENABLE_MODULE 0
#define POPUP_VIEWERS_START 1

namespace RDK2 { namespace RConsoleQt {

using namespace std;
using namespace RDK2::TextUtils;

TreeViewerWidget::TreeViewerWidget(QWidget* parent, string agentName, WFlags f) :
	RqPropertyListView(parent, agentName.c_str(), f), agentName(agentName)
{
	//addColumn(QString("name"));	// this is added by the RqPropertyListView class
	addColumn(QString("class"));
	addColumn(QString("description"));
	addColumn(QString("link"));
	setRootIsDecorated(true);
	setAllColumnsShowFocus(true);
	
	QObject::connect(this, SIGNAL(rightButtonClicked(QListViewItem*,const QPoint&,int)),
		this, SLOT(slot_rightButtonClicked(QListViewItem*,const QPoint&,int)));
	QObject::connect(this, SIGNAL(doubleClicked(QListViewItem*,const QPoint&,int)),
			this, SLOT(slot_listViewDoubleClicked(QListViewItem*,const QPoint&,int)));

}

TreeViewerWidget::~TreeViewerWidget() { }

void TreeViewerWidget::slot_listViewDoubleClicked(QListViewItem* item, const QPoint& p, int)
{
	bool exiting = false; QT_THREAD_GUARD()
	
	Url completeUrl = listItemToUrl(item);
	if (completeUrl == "") return;
	if (!completeUrl.isComplete()) completeUrl = completeUrl.contextualize("rdk://" + agentName);

	Session* guiSession = RqCommon::getGuiSession("");
	
	bool propertyExists = false;
	SESSION_TRY_START(guiSession)
		propertyExists = guiSession->propertyExists(completeUrl);
	SESSION_END_CATCH_TERMINATE_NOMSG(guiSession)
			
	if (propertyExists) {
		RqMainWindow* mw = RqCommon::getMainWindow();
		QPoint pp = mw->mapFromGlobal(p);
		RqCommon::createDefaultViewerFor(completeUrl, pp);
	}
}

void TreeViewerWidget::slot_rightButtonClicked(QListViewItem* item, const QPoint& p,int)
{
	if (!item) return;
	bool exiting = false; QT_THREAD_GUARD()
	
	Url completeUrl = listItemToUrl(item);
	if (completeUrl == "") return;
	if (!completeUrl.isComplete()) completeUrl = completeUrl.contextualize("rdk://" + agentName);
	
	popupPropertyUrl = completeUrl;
	popupX = p.x();
	popupY = p.y();
	
	Session* guiSession = RqCommon::getGuiSession("");
	
	string propertyClass = "";
	SESSION_TRY_START(guiSession)
			propertyClass = guiSession->getPropertyClassName(completeUrl);
	// if the property doesn't exists, it can be a "container"
	SESSION_END_CATCH_TERMINATE_NOMSG(guiSession)
	
	if (propertyClass != "") {
		set<RqCommon::RegisteredViewer> viewers = RqCommon::getViewersForClass(propertyClass);
		if (!viewers.empty()) {
			QPopupMenu* m = new QPopupMenu(this);
			QObject::connect(m, SIGNAL(activated(int)), this, SLOT(slot_popupActivated(int)));
			
			int a = POPUP_VIEWERS_START;
			for (set<RqCommon::RegisteredViewer>::iterator it = viewers.begin(); it != viewers.end(); ++it) {
				m->insertItem(it->menuCaption, a++);
			}
			m->popup(p);
		}
	}
	else if (completeUrl.getLevel() == 1) {
		QPopupMenu* m = 0;
		SESSION_TRY_START(guiSession)
		string s = "Enable module";
		if (guiSession->getBool(completeUrl + "/enabled")) s = "Disable module";
		m = new QPopupMenu(this);
		QObject::connect(m, SIGNAL(activated(int)), this, SLOT(slot_popupActivated(int)));
		m->insertItem(s, POPUP_ENABLE_MODULE);
		SESSION_END_CATCH_TERMINATE_NOMSG(guiSession)
		if (m) m->popup(p);
	}
}

void TreeViewerWidget::slot_popupActivated(int id)
{
	bool exiting = false; QT_THREAD_GUARD()
	if (id == POPUP_ENABLE_MODULE) {
		popupPropertyUrl = popupPropertyUrl + "/enabled";
		Session* guiSession = RqCommon::getGuiSession("");
		SESSION_TRY_START(guiSession)
		guiSession->setBool(popupPropertyUrl, !guiSession->getBool(popupPropertyUrl));
		SESSION_END_CATCH_TERMINATE(guiSession)
	}
	else {
		string propertyClass = "";
		Session* guiSession = RqCommon::getGuiSession("");
		SESSION_TRY_START(guiSession)
				propertyClass = guiSession->getPropertyClassName(popupPropertyUrl);
		SESSION_END_CATCH_TERMINATE(guiSession)
		if (propertyClass == "") return;
		
		set<RqCommon::RegisteredViewer> viewers = RqCommon::getViewersForClass(propertyClass);
		if (!viewers.empty()) {
			set<RqCommon::RegisteredViewer>::iterator it = viewers.begin();
			for (int a = POPUP_VIEWERS_START; a < id; a++) {
				++it;
				if (it == viewers.end()) break;
			}
			if (it != viewers.end()) RqCommon::createViewerFor(popupPropertyUrl, it->libraryName, QPoint(popupX, popupY));
		}
	}
}

void TreeViewerWidget::buildTree(const RPropertyDefVector* propertyDefs)
{
	// this will be called NOT from QT thread
	if (!propertyDefs) {
		RDK_ERROR_PRINTF("Passing a null property definition vector");
		return;
	}
	for (RPropertyDefVector::const_iterator it = propertyDefs->begin();
	it != propertyDefs->end(); ++it) {
		addProperty((*it)->completeUrl.getPath(),
				(*it)->getObjectClassName(),
				(*it)->description,
				(*it)->linkTo);
	}
}

void TreeViewerWidget::addProperty(const string& absoluteUrl, const string& className, const string& description, const string& linkTo)
{
	// this will be called NOT from QT thread
	PropertyToAdd pta;
	pta.absoluteUrl = absoluteUrl;
	pta.className = className;
	pta.description = description;
	pta.linkTo = linkTo;
	mutex.lock(HERE);
	propertiesToAdd.push_back(pta);
	mutex.unlock();
}

void TreeViewerWidget::deleteProperty(const string& absoluteUrl)
{
	// this will be called NOT from QT thread
	mutex.lock(HERE);
	propertiesToDelete.push_back(absoluteUrl);
	mutex.unlock();
}
								   
void TreeViewerWidget::customEvent(QCustomEvent* e)
{
	bool exiting = false; QT_THREAD_GUARD()
	if (e->type() == EVENT_REFRESH_TREE) {
		mutex.lock(HERE);
		for (list<PropertyToAdd>::iterator it = propertiesToAdd.begin(); it != propertiesToAdd.end(); ++it) {
			QListViewItem* item = qtAddProperty(it->absoluteUrl);
			item->setText(1, it->className);
			item->setText(2, it->description);
			item->setText(3, it->linkTo);
		}
		for (list<string>::iterator it = propertiesToDelete.begin(); it != propertiesToDelete.end(); ++it) {
			qtDeleteProperty(*it);
		}
		mutex.unlock();
	}
}

void TreeViewerWidget::postRefresh()
{
	QApplication::postEvent(this, new QCustomEvent(EVENT_REFRESH_TREE));
}		

QDragObject* TreeViewerWidget::dragObject()
{
	bool exiting = false; QT_THREAD_GUARD(0)
	QListViewItem* element = selectedItem();
	
	// If none selected no drag is taken
	if (!element) return 0;

	Url completeUrl = listItemToUrl(element);
	if (completeUrl == "") return 0;
	if (!completeUrl.isComplete()) completeUrl = completeUrl.contextualize("rdk://" + agentName);

	RqPropertyDragObject* dragged = new RqPropertyDragObject(completeUrl, this);

	return dragged;
}

}} // namespaces
