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

#ifndef RDK2_RQM_JOYSTICKMODULE
#define RDK2_RQM_JOYSTICKMODULE

#include "../../rqcommon/rqmodule.h"

#include <rdkcore/modules/module.h>

#include <qlineedit.h>
#include <qpushbutton.h>
#include <qlayout.h>
#include <qcheckbox.h>

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RepositoryNS;
using namespace RDK2::RAgent;

class JoystickModule : public RqModule {
Q_OBJECT
private:
	QLineEdit* tboxDestination;
	QLineEdit* tboxPropertyUrl;
	QPushButton *upB, *downB, *leftB, *rightB, *stopB,;
	QPushButton *aB, *bB, *cB, *dB, *lB, *rB, *jB, *kB;
	QCheckBox* chkUseUdp;
	
	void sendCommand();
	string cmd;

	QWidget* createWidget();
	
public slots:
	void slot_up();
	void slot_down();
	void slot_left();
	void slot_right();
	void slot_stop();
	void slot_l();
	void slot_r();
	void slot_j();
	void slot_k();
	void slot_a();
	void slot_b();
	void slot_c();
	void slot_d();

public:
	JoystickModule() { }

	bool initConfigurationProperties();
};

}} // ns

#endif
