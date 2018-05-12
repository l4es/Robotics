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

#ifndef RDK2_RQM_FOURARROWSMODULE
#define RDK2_RQM_FOURARROWSMODULE

#include "../../rqcommon/rqmodule.h"

#include <rdkcore/modules/module.h>

#include <qlineedit.h>
#include <qpushbutton.h>
#include <qlayout.h>
#include <qcheckbox.h>

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RepositoryNS;
using namespace RDK2::RAgent;

class FourArrowsModule : public RqModule {
Q_OBJECT
private:
	QLineEdit* tboxDestination;
	QPushButton *upB, *downB, *leftB, *rightB, *stopB;
	QCheckBox* chkUseUdp;
	
	double speed, jog;
	void sendCommand();

	QWidget* createWidget();
	
public slots:
	void slot_goFront();
	void slot_goBack();
	void slot_goRight();
	void slot_goLeft();
	void slot_stop();

public:
	FourArrowsModule() : speed(0.), jog(0.) { }

	bool initConfigurationProperties();
};

}} // ns

#endif
