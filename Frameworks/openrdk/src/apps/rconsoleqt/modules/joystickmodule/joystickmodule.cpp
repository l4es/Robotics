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

#include "joystickmodule.h"

#include "../../rqcommon/rqcommon.h"

#include <rdkcore/geometry/angle.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "JoystickModule"

#include <qlabel.h>
#include <qapplication.h>

#define PROPERTY_DESTINATION "destination"
#define PROPERTY_REMOTE_URL "remoteUrl"
#define PROPERTY_BUTTON_UP "up"
#define PROPERTY_BUTTON_DOWN "down"
#define PROPERTY_BUTTON_LEFT "left"
#define PROPERTY_BUTTON_RIGHT "right"
#define PROPERTY_BUTTON_STOP "stop"
#define PROPERTY_BUTTON_L "buttonL"
#define PROPERTY_BUTTON_R "buttonR"
#define PROPERTY_BUTTON_J "buttonJ"
#define PROPERTY_BUTTON_K "buttonK"
#define PROPERTY_BUTTON_A "buttonA"
#define PROPERTY_BUTTON_B "buttonB"
#define PROPERTY_BUTTON_C "buttonC"
#define PROPERTY_BUTTON_D "buttonD"
#define PROPERTY_LAST_COMMAND "lastCommand"

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::Geometry;

bool JoystickModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		session->createString(PROPERTY_DESTINATION, "Destination", "");
		session->createString(PROPERTY_REMOTE_URL, "Remote Property Url", "");
		session->createString(PROPERTY_BUTTON_UP, "Command for UP key", "");
		session->createString(PROPERTY_BUTTON_DOWN, "Command for DOWN key", "");
		session->createString(PROPERTY_BUTTON_LEFT, "Command for LEFT key", "");
		session->createString(PROPERTY_BUTTON_RIGHT, "Command for RIGHT key", "");
		session->createString(PROPERTY_BUTTON_STOP, "Command for STOP key", "");

		session->createString(PROPERTY_BUTTON_L, "Command for L key", "");
		session->createString(PROPERTY_BUTTON_R, "Command for R key", "");
		session->createString(PROPERTY_BUTTON_J, "Command for J key", "");
		session->createString(PROPERTY_BUTTON_K, "Command for K key", "");
		session->createString(PROPERTY_BUTTON_A, "Command for A key", "");
		session->createString(PROPERTY_BUTTON_B, "Command for B key", "");
		session->createString(PROPERTY_BUTTON_C, "Command for C key", "");
		session->createString(PROPERTY_BUTTON_D, "Command for D key", "");

		session->createString(PROPERTY_LAST_COMMAND, "Last Command Sent", "");
		cmd = session->getString(PROPERTY_BUTTON_STOP);
	SESSION_END_CATCH_TERMINATE(session)
	return true;
}

QWidget* JoystickModule::createWidget()
{
	QT_THREAD_GUARD(0)
	QWidget* widget = new QWidget();
	QVBoxLayout* vbl = new QVBoxLayout(widget);
	vbl->addWidget(new QLabel("<small>Destination:</small>", widget));
	vbl->addWidget(tboxDestination = new QLineEdit(widget));
	vbl->addWidget(new QLabel("<small>RemotePropertyUrl:</small>", widget));
	vbl->addWidget(tboxPropertyUrl = new QLineEdit(widget));

	QGridLayout* layout = new QGridLayout(vbl, 4, 4);

	layout->addWidget(upB = new QPushButton("^", widget),    0, 1);
	layout->addWidget(downB = new QPushButton("v", widget),  2, 1);
	layout->addWidget(rightB = new QPushButton(">", widget), 1, 2);
	layout->addWidget(leftB = new QPushButton("<", widget),  1, 0);
	layout->addWidget(stopB = new QPushButton("x", widget),  1, 1);

	layout->addWidget(lB = new QPushButton("L", widget),    0, 0);
	layout->addWidget(rB = new QPushButton("R", widget),    0, 2);
	layout->addWidget(jB = new QPushButton("J", widget),    2, 0);
	layout->addWidget(kB = new QPushButton("K", widget),    2, 2);

	layout->addWidget(aB = new QPushButton("A", widget),    0, 3);
	layout->addWidget(bB = new QPushButton("B", widget),    1, 3);
	layout->addWidget(cB = new QPushButton("C", widget),    2, 3);
	layout->addWidget(dB = new QPushButton("D", widget),    3, 3);

	upB->setAccel(QKeySequence(Key_Up));
	downB->setAccel(QKeySequence(Key_Down));
	leftB->setAccel(QKeySequence(Key_Left));
	rightB->setAccel(QKeySequence(Key_Right));
	stopB->setAccel(QKeySequence(Key_Space));

	lB->setAccel(QKeySequence(Key_L));
	rB->setAccel(QKeySequence(Key_R));
	jB->setAccel(QKeySequence(Key_J));
	kB->setAccel(QKeySequence(Key_K));

	aB->setAccel(QKeySequence(Key_A));
	bB->setAccel(QKeySequence(Key_B));
	cB->setAccel(QKeySequence(Key_C));
	dB->setAccel(QKeySequence(Key_D));

	connect( upB,    SIGNAL( pressed() ), this, SLOT( slot_up() ) );
	connect( downB,  SIGNAL( pressed() ), this, SLOT( slot_down()  ) );
	connect( leftB,  SIGNAL( pressed() ), this, SLOT( slot_left()  ) );
	connect( rightB, SIGNAL( pressed() ), this, SLOT( slot_right() ) );
	connect( stopB,  SIGNAL( pressed() ), this, SLOT( slot_stop()    ) );

	connect( lB,    SIGNAL( pressed() ), this, SLOT( slot_l() ) );
	connect( rB,    SIGNAL( pressed() ), this, SLOT( slot_r() ) );
	connect( jB,    SIGNAL( pressed() ), this, SLOT( slot_j() ) );
	connect( kB,    SIGNAL( pressed() ), this, SLOT( slot_k() ) );

	connect( aB,    SIGNAL( pressed() ), this, SLOT( slot_a() ) );
	connect( bB,    SIGNAL( pressed() ), this, SLOT( slot_b() ) );
	connect( cB,    SIGNAL( pressed() ), this, SLOT( slot_c() ) );
	connect( dB,    SIGNAL( pressed() ), this, SLOT( slot_d() ) );
	
	return widget;
}

void JoystickModule::slot_up()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		RDK_DEBUG_STREAM("Pressed up");
		cmd = guiSession->getString(PROPERTY_BUTTON_UP);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void JoystickModule::slot_down()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		RDK_DEBUG_STREAM("Pressed down");
		cmd = guiSession->getString(PROPERTY_BUTTON_DOWN);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void JoystickModule::slot_right()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		RDK_DEBUG_STREAM("Pressed right");
		cmd = guiSession->getString(PROPERTY_BUTTON_RIGHT);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void JoystickModule::slot_left()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		RDK_DEBUG_STREAM("Pressed left");
		cmd = guiSession->getString(PROPERTY_BUTTON_LEFT);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void JoystickModule::slot_l()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		RDK_DEBUG_STREAM("Pressed L");
		cmd = guiSession->getString(PROPERTY_BUTTON_L);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void JoystickModule::slot_r()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		RDK_DEBUG_STREAM("Pressed R");
		cmd = guiSession->getString(PROPERTY_BUTTON_R);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void JoystickModule::slot_j()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		RDK_DEBUG_STREAM("Pressed J");
		cmd = guiSession->getString(PROPERTY_BUTTON_J);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void JoystickModule::slot_k()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		RDK_DEBUG_STREAM("Pressed K");
		cmd = guiSession->getString(PROPERTY_BUTTON_K);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void JoystickModule::slot_a()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		RDK_DEBUG_STREAM("Pressed A");
		cmd = guiSession->getString(PROPERTY_BUTTON_A);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void JoystickModule::slot_b()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		RDK_DEBUG_STREAM("Pressed B");
		cmd = guiSession->getString(PROPERTY_BUTTON_B);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void JoystickModule::slot_c()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		RDK_DEBUG_STREAM("Pressed C");
		cmd = guiSession->getString(PROPERTY_BUTTON_C);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void JoystickModule::slot_d()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		RDK_DEBUG_STREAM("Pressed D");
		cmd = guiSession->getString(PROPERTY_BUTTON_D);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void JoystickModule::slot_stop()
{
	RDK_DEBUG_STREAM("Pressed stop");
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		cmd = guiSession->getString(PROPERTY_BUTTON_STOP);
	SESSION_END_CATCH_TERMINATE(guiSession)
	sendCommand();
}

void JoystickModule::sendCommand()
{
	cerr << "try this " << endl;
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		cerr << "2try this " << endl;
		ostringstream cmdss;
		cmdss << "rdk://";
		string dest = guiSession->getString(PROPERTY_DESTINATION);
		RDK_INFO_STREAM(dest);
		cerr << "3try this " << dest << endl;

		if (dest == "")
			dest = guiSession->getRepositoryName();
		cmdss << dest << "/";
		dest = guiSession->getString(PROPERTY_REMOTE_URL);
		RDK_INFO_STREAM(dest);
		cerr << "try " << dest << endl;
		if (dest != "")
		{
			cmdss << dest;
			guiSession->setString(cmdss.str(),cmd);
			guiSession->setString(PROPERTY_LAST_COMMAND,cmd);
		}

		//ostringstream cmdss; cmdss << "setSpeedAndJog " << speed << " " << jog;
		//string dest = guiSession->getString("destination");
		//if (dest == "") dest = guiSession->getRepositoryName();
		//guiSession->queuePush(PROPERTY_OUTBOX, 
		//  new RNetMessage(dest, guiSession->getRepositoryName(), RNetMessage::AGENT_CMD,
		//  guiSession->getBool(PROPERTY_SEND_VIA_UDP) ? Network::UDP_IP : Network::TCP_IP,
		//  new RString(cmdss.str())));
	SESSION_END_CATCH_TERMINATE(guiSession)
}

MODULE_FACTORY(JoystickModule);

}} // ns
