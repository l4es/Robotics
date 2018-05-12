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

#include "fourarrowsmodule.h"

#include "../../rqcommon/rqcommon.h"

#include <rdkcore/geometry/angle.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "FourArrowsModule"

#include <qlabel.h>
#include <qapplication.h>

#define PROPERTY_DESTINATION "destination"
#define PROPERTY_SPEED_INCREMENT "speedIncrement"
#define PROPERTY_SPEED_LIMIT "speedLimit"
#define PROPERTY_JOG_INCREMENT "jogIncrement"
#define PROPERTY_JOG_LIMIT "jogLimit"
#define PROPERTY_SEND_VIA_UDP "sendViaUdp"

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::Geometry;

bool FourArrowsModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		session->createString(PROPERTY_DESTINATION, "Destination", "");
		session->createDouble(PROPERTY_SPEED_INCREMENT, "Linear speed increment", RDouble::M_SEC, 0.02);
		session->createDouble(PROPERTY_SPEED_LIMIT, "Linear speed limit", RDouble::M_SEC, 0.500);
		session->createDouble(PROPERTY_JOG_INCREMENT, "Jog increment", RDouble::RAD_SEC, deg2rad(3));
		session->createDouble(PROPERTY_JOG_LIMIT, "Jog limit", RDouble::RAD_SEC, deg2rad(15.));
		session->createBool(PROPERTY_SEND_VIA_UDP, "Send command via UDP", true);
	SESSION_END_CATCH_TERMINATE(session)
	return true;
}

QWidget* FourArrowsModule::createWidget()
{
	QT_THREAD_GUARD(0)
	QWidget* widget = new QWidget();
	QVBoxLayout* vbl = new QVBoxLayout(widget);
	vbl->addWidget(new QLabel("<small>Destination:</small>", widget));
	vbl->addWidget(tboxDestination = new QLineEdit(widget));

	QGridLayout* layout = new QGridLayout(vbl, 3, 3);

	layout->addWidget(upB = new QPushButton("^", widget),    0, 1);
	layout->addWidget(downB = new QPushButton("v", widget),  2, 1);
	layout->addWidget(rightB = new QPushButton(">", widget), 1, 2);
	layout->addWidget(leftB = new QPushButton("<", widget),  1, 0);
	layout->addWidget(stopB = new QPushButton("x", widget),  1, 1);

	upB->setAccel(QKeySequence(Key_Up));
	downB->setAccel(QKeySequence(Key_Down));
	leftB->setAccel(QKeySequence(Key_Left));
	rightB->setAccel(QKeySequence(Key_Right));
	stopB->setAccel(QKeySequence(Key_Space));

	connect( upB,    SIGNAL( pressed() ), this, SLOT( slot_goFront() ) );
	connect( downB,  SIGNAL( pressed() ), this, SLOT( slot_goBack()  ) );
	connect( leftB,  SIGNAL( pressed() ), this, SLOT( slot_goLeft()  ) );
	connect( rightB, SIGNAL( pressed() ), this, SLOT( slot_goRight() ) );
	connect( stopB,  SIGNAL( pressed() ), this, SLOT( slot_stop()    ) );
	return widget;
}

void FourArrowsModule::slot_goFront()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		speed += guiSession->getDouble(PROPERTY_SPEED_INCREMENT);
		if (speed > guiSession->getDouble(PROPERTY_SPEED_LIMIT))
			speed = guiSession->getDouble(PROPERTY_SPEED_LIMIT);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void FourArrowsModule::slot_goBack()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		speed -= guiSession->getDouble(PROPERTY_SPEED_INCREMENT);
		if (speed < -guiSession->getDouble(PROPERTY_SPEED_LIMIT))
			speed = -guiSession->getDouble(PROPERTY_SPEED_LIMIT);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void FourArrowsModule::slot_goRight()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		jog -= guiSession->getDouble(PROPERTY_JOG_INCREMENT);
		if (jog < -guiSession->getDouble(PROPERTY_JOG_LIMIT))
			jog = -guiSession->getDouble(PROPERTY_JOG_LIMIT);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void FourArrowsModule::slot_goLeft()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		jog += guiSession->getDouble(PROPERTY_JOG_INCREMENT);
		if (jog > guiSession->getDouble(PROPERTY_JOG_LIMIT))
			jog = guiSession->getDouble(PROPERTY_JOG_LIMIT);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void FourArrowsModule::slot_stop()
{
	speed = jog = 0;
	sendCommand();
}

void FourArrowsModule::sendCommand()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		ostringstream cmdss; cmdss << "setSpeedAndJog " << speed << " " << jog;
		string dest = guiSession->getString("destination");
		if (dest == "") dest = guiSession->getRepositoryName();
		guiSession->queuePush(PROPERTY_OUTBOX, 
			new RNetMessage(dest, guiSession->getRepositoryName(), RNetMessage::AGENT_CMD,
			guiSession->getBool(PROPERTY_SEND_VIA_UDP) ? Network::UDP_IP : Network::TCP_IP,
			new RString(cmdss.str())));
	SESSION_END_CATCH_TERMINATE(guiSession)
}

MODULE_FACTORY(FourArrowsModule);

}} // ns
