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

#include "rqpropertylistview.h"

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RqPropertyListView"

#include "rqcommon.h"

namespace RDK2 { namespace RConsoleQt {

RqPropertyListView::RqPropertyListView(QWidget* parent, const char* name, WFlags f) :
	QListView(parent, name, f), useCheckBoxes(false)
{
	addColumn(QString("name"));
	setRootIsDecorated(true);
	setAllColumnsShowFocus(true);
}

RqPropertyListView::~RqPropertyListView() { }

string lastUrlPart(string u)
{
	vector<string> v = RDK2::TextUtils::tokenize(u, "/");
	return (v.size() ? v[v.size()-1] : "ERROR");
}

QListViewItem* RqPropertyListView::checkTreeParentsFor(cstr absoluteUrl)
{
	bool exiting = false;
	QT_THREAD_GUARD(0)
	vector<string> v = RDK2::TextUtils::tokenize(absoluteUrl, "/");
	if (v.size() < 2) return 0;
	string guessedParent;
	for (size_t i = 0; i < v.size() - 1; i++) {
		guessedParent += "/" + v[i];
	}
	map<Url, QListViewItem*>::iterator it = map_ni.find(guessedParent);
	if (it != map_ni.end()) return it->second;
	QListViewItem* parent = checkTreeParentsFor(guessedParent);
	QListViewItem* item =
		(parent == 0) ?
			new QListViewItem(this, QString(lastUrlPart(guessedParent).c_str()))
		:  new QListViewItem(parent, QString(lastUrlPart(guessedParent).c_str()));
	
	item->setDragEnabled(true);
	mutex.lock(HERE);
	map_ni.insert(make_pair(guessedParent, item));
	map_in.insert(make_pair(item, guessedParent));
	mutex.unlock();
	return item;
}

QListViewItem* RqPropertyListView::qtAddProperty(cstr aurl)
{
	bool exiting = false;
	QT_THREAD_GUARD(0)
	Url url(aurl);
	string absoluteUrl;
	if (url.isComplete()) absoluteUrl = url.getPath();
	else absoluteUrl = url;
	mutex.lock(HERE);
	map<Url, QListViewItem*>::iterator eIt = map_ni.find(absoluteUrl);
	if (eIt != map_ni.end()) {
		mutex.unlock();
		// the property is already here, we return it
		return eIt->second;
	}
	else {
		// this is new, so we add it
		QListViewItem* parent = checkTreeParentsFor(absoluteUrl);
	        std::string name = absoluteUrl;
        	QListViewItem* item;
		if (parent == 0) item = (useCheckBoxes ?
			new QCheckListItem(this, lastUrlPart(name).c_str(),
				QCheckListItem::CheckBoxController) :
			new QListViewItem(this, lastUrlPart(name).c_str()));
		else item = (useCheckBoxes ?
			new QCheckListItem(parent, lastUrlPart(name).c_str(),
				QCheckListItem::CheckBoxController) :
			new QListViewItem(parent, lastUrlPart(name).c_str()));
		if (useCheckBoxes) {
			((QCheckListItem*)item)->setTristate(false);
		}
		map_ni.insert(make_pair(absoluteUrl, item));
		map_in.insert(make_pair(item, absoluteUrl));
		mutex.unlock();
		item->setDragEnabled(true);
		return item;
	}
}

void RqPropertyListView::qtDeleteProperty(cstr absoluteUrl)
{
	bool exiting = false;
	QT_THREAD_GUARD()
	mutex.lock(HERE);
	map<Url, QListViewItem*>::iterator hhmm = map_ni.find(absoluteUrl);
	if (hhmm != map_ni.end()) map_ni.erase(hhmm);
	else { RDK_ERROR_PRINTF("Cannot find property '%s'", absoluteUrl.c_str()); }
	for (map<QListViewItem*, Url>::iterator it = map_in.begin(); it != map_in.end(); ++it) {
		if (it->second == absoluteUrl) {
			QListViewItem* parent = it->first->parent();
			delete it->first;
			map_in.erase(it);
			if (parent && parent->text(1) == "" && parent->childCount() == 0) {
				map<QListViewItem*, Url>::iterator itt = map_in.find(parent);
				if (itt != map_in.end()) qtDeleteProperty(itt->second);
			}
			break;
		}
	}
	mutex.unlock();
}

QListViewItem* RqPropertyListView::urlToListItem(CUrl url)
{
	bool exiting = false;
	QT_THREAD_GUARD(0)
	mutex.lock(HERE);
	QListViewItem* ret;
	map<Url, QListViewItem*>::iterator it = map_ni.find(url);
	if (it == map_ni.end()) ret = 0;
	else ret = it->second;
	mutex.unlock();
	return ret;
}

Url RqPropertyListView::listItemToUrl(QListViewItem* item)
{
	// FIXME
	if (!RqCommon::checkQtThread()) { RDK_ERROR_PRINTF("Not in QT thread (%p)!!", pthread_self()); return ""; }

	mutex.lock(HERE);
	Url url("");
	map<QListViewItem*, Url>::iterator it = map_in.find(item);
	if (it == map_in.end()) url = "";
	else url = it->second;
	mutex.unlock();
	return url;
}

}} // namespaces
