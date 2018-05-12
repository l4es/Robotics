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

#ifndef RDK2_RCONSOLEN_TREEVIEWERWIDGET
#define RDK2_RCONSOLEN_TREEVIEWERWIDGET

#include <string>
#include <rdkcore/textutils/textutils.h>
#include <rdkcore/repository_struct/rpropertydefvector.h>
#include <rdkcore/repository_struct/url.h>
#include <rdkcore/posixconstructs/posixmutex.h>

#include <qlistview.h>

#include "../../rqcommon/rqpropertylistview.h"

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RepositoryNS;
using namespace RDK2::TextUtils;
using namespace PosixConstructs;

class TreeViewerWidget : public RqPropertyListView {
Q_OBJECT
public:
	static const int EVENT_REFRESH_TREE = QEvent::User + 1;
	
	TreeViewerWidget(QWidget* parent, string agentName, WFlags f = 0);
	virtual ~TreeViewerWidget();

	void buildTree(const RDK2::RepositoryNS::RPropertyDefVector* propertyDefs);
	void addProperty(const string& absoluteUrl, const string& className, const string& description, const string& linkTo);
	void deleteProperty(const string& absoluteUrl);
	void postRefresh();
	
public slots:
	void slot_listViewDoubleClicked(QListViewItem* item, const QPoint& p, int);
	void slot_rightButtonClicked(QListViewItem*,const QPoint&,int);
	void slot_popupActivated(int);
	
private:
	virtual QDragObject* dragObject();
	string agentName;
	
	void customEvent(QCustomEvent* e);
	
	struct PropertyToAdd {
		string absoluteUrl;
		string className;
		string description;
		string linkTo;
	};
	
	PosixMutex mutex;
	list<PropertyToAdd> propertiesToAdd;
	list<string> propertiesToDelete;
	
	string popupPropertyUrl;
	int popupX, popupY;
};

}} // namespaces

#endif
