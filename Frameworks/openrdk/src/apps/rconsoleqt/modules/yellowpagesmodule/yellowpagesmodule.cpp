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

#include "yellowpagesmodule.h"
#include <qtooltip.h>
#include <sstream>

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "YPViewer"

#include <rdkcore/rnetobjects/ryellowpages.h>


namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::Geometry;

QWidget* YellowPagesModule::createWidget()
{
	string myUrl = "<error>";
	vHost.reserve(10);
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession);
	myUrl = guiSession->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
	guiSession->lock(myUrl, HERE);
	RDK2::RNetObjects::RYellowPages* objVal = guiSession->getObjectAsL<RDK2::RNetObjects::RYellowPages>(myUrl);
	const RDK2::RNetObjects::RYellowPages::YPHostMap& hostMap = objVal->getMap();

	for (RDK2::RNetObjects::RYellowPages::YPHostMap::const_iterator it = hostMap.begin();
								it != hostMap.end();
								++it)
	{
		for (std::map<std::string, Network::SocketAddress>::const_iterator it2 = it->second.begin();
					it2 != it->second.end();
					++it2)
		{
			Host q;
			q.protocol = netProtocolToString(it->first);
			q.hostname = it2->first;
			std::stringstream address;
			InetAddress ia(it2->second);
			address << ia.getIPAddress() << ":" << int(ia.getPort());
			q.address = address.str();
			vHost.push_back(q);
		}
	}
	guiSession->unlock(myUrl);
	SESSION_END_CATCH_TERMINATE(guiSession);

	QWidget* widget = new QWidget();
	QVBoxLayout* mainLayout = new QVBoxLayout(widget);
	QLabel* label = new QLabel(SMALL_URL_CAPTION(myUrl.c_str()), widget);
	QWidget* editPanel = new QWidget(widget);
	editPanel->setPaletteForegroundColor(Qt::red);
	mainLayout->addWidget(label);
	mainLayout->addWidget(editPanel);

	QGridLayout* editLayout = new QGridLayout(mainLayout, vHost.size(), 3);

	int c=0;
	for (std::vector<Host>::iterator it=vHost.begin();
				it != vHost.end();
				++it, ++c)
	{
		editLayout->addWidget(it->leProtocol = new QLineEdit(widget), c, 1);
		editLayout->addWidget(it->leHostname = new QLineEdit(widget), c, 2);
		editLayout->addWidget(it->leAddress = new QLineEdit(widget), c, 3);
	}
	return widget;
}

RDK2::Object* YellowPagesModule::buildRObject()
{
	RDK2::Object* obj=new RDK2::RNetObjects::RYellowPages();
	RDK_ERROR_STREAM("Warning YPViewer::buildRObject() not implemented! Fix it if you need!");
	//for (std::vector<Host>::iterator it=vHost.begin();
				//it != vHost.end();
				//++it)
	//{
	//}

	return obj;
}

void YellowPagesModule::refreshWidgetFields()
{
	QT_THREAD_GUARD()
	for (std::vector<Host>::iterator vit = vHost.begin();
						vit != vHost.end();
						++vit)
	{
		QString qs;
		vit->leProtocol->setText(qs.sprintf("%s",vit->protocol.c_str()));
		vit->leHostname->setText(qs.sprintf("%s",vit->hostname.c_str()));
		vit->leAddress->setText(qs.sprintf("%s",vit->address.c_str()));
	}
}

MODULE_FACTORY(YellowPagesModule);

}} // ns
