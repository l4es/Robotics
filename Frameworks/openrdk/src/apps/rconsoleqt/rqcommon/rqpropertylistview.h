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

#ifndef RDK2_RQ_RQPROPERTYLISTVIEW
#define RDK2_RQ_RQPROPERTYLISTVIEW

#include <map>
#include <qlistview.h>

#include <rdkcore/repository_struct/url.h>
#include <rdkcore/textutils/textutils.h>
#include <rdkcore/posixconstructs/posixmutex.h>

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RepositoryNS;
using namespace RDK2::TextUtils;
using namespace PosixConstructs;

class RqPropertyListView : public QListView {
Q_OBJECT
public:
	RqPropertyListView(QWidget* parent = 0, const char* name = 0, WFlags f = 0);
	~RqPropertyListView();

	QListViewItem* qtAddProperty(cstr absoluteUrl);
	void qtDeleteProperty(cstr absoluteUrl);

	void setUseCheckBoxes(bool use) { useCheckBoxes = use; }

private:
	QListViewItem* checkTreeParentsFor(cstr absoluteUrl);

protected:
	QListViewItem* urlToListItem(CUrl url);
	Url listItemToUrl(QListViewItem* item);

	PosixConstructs::PosixMutex mutex;
		std::map<Url, QListViewItem*> map_ni;
		std::map<QListViewItem*, Url> map_in;

	bool useCheckBoxes;
};

}} // namespaces

#endif
