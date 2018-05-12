/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
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


#ifndef  RDK_MOTIONSHM_INC
#define  RDK_MOTIONSHM_INC

#include "sharedmemory_traits.h"

struct RdkMotionData
{
	char cmd[64]; // tipo di comando con stringa che indica l'operazione da fare... In motionserver::send() è necessario identificare il comando per avviare una funzione di locomozione. I giunti, come nel caso del calcio, devono essere precaricati, ossia, una volta definiti i parametri è inutile calcolare ad ogni passo la matrice, basta memorizzarli e usarli come se fossero costanti.
	float targetJointsValues[JOINTS_VALUES_SIZE];
	float executionTime;
	int withSpeedValue;
	unsigned int interpolationMode;
	bool change_stiffness;
	int  stiffness_chain; // see enum in naojoins.h  { None, Body, Head, LArm, LLeg, RLeg, RArm }
	double stiffness;
	bool shutdown;
	char cmdHead[64];
	char cmdBody[64];
	/*
	#ifdef USE_NEW_SHM
	char cmdHead[64];
	char cmdBody[64];
	#endif*/
};




#endif   /* ----- #ifndef MOTIONSHM_INC  ----- */
