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

#ifndef  RDK_SENSORDATASHM_INC
#define  RDK_SENSORDATASHM_INC

#include "sharedmemory_traits.h"

//#include <rdkcore/config.h>

#include "naocameraparams.h"

enum BallDistance
{
	VERY_NEAR_BALL = 0,
	NEAR_BALL,
	MIDDLE_DISTANCE_BALL,
	FAR_BALL,
	LOST_BALL
};

enum GameState
{
	GAME_INITIAL = 0,
	GAME_READY,
	GAME_SET,
	GAME_PLAY,
	GAME_FINISH,
	GAME_PENALIZED
};

enum PlayerRole
{
	GOALIE = 0,
	DEFENDER,
	ATTACKER,
	SUPPORTER,
	NO_ROLE
};

enum TeamColor
{
	TEAM_RED = 0,
	TEAM_BLUE,
	TEAM_UNKNOWN
};

struct RdkSensorData
{
	struct Battery
	{
		float charge;
		float current;
		float temperature;
	};
	
	struct Gyrometer
	{
		float x;
		float y;
		float ref;
	};

	struct Accelerometer
	{
		float x;
		float y;
		float z;
	};

	struct InertialAngle
	{
		float x;
		float y;
	};

	struct UltraSound
	{
		float left;
		float right;
	};

	struct FootForceSensor
	{
		float x;
		float y;
		float weight;
	};
	
	struct ForceSensors
	{
		FootForceSensor left;
		FootForceSensor right;
	};

	struct FootBumper
	{
		float left;
		float right;
	};
	
	struct Bumpers
	{
		FootBumper left;
		FootBumper right;
	};
	
	struct FeedbackSensor
	{
		char motionModuleTaskByNaoMotionModule[20];
		bool isHeadCommandExecutingByNaoMotionModule;
		double odometryXByNaoMotionModule;
		double odometryYByNaoMotionModule;
		double odometryThetaByNaoMotionModule;
		
		char gamestateByXSentinel[20];
		bool isBlueTeamByXSentinel;
	};

	double timestamp;
	Battery battery;
	float jointsValues[JOINTS_VALUES_SIZE];
	Gyrometer gyr;
	Accelerometer acc;
	InertialAngle angle;
	UltraSound us;
	ForceSensors fsr;
	Bumpers bumpers;
	FeedbackSensor feedbackSensor;
};

struct RdkAudioData
{
	char text[256];
	int volume; // 0-100
};

struct RdkLedsData
{
	BallDistance ballDistance;
	GameState gameState;
	PlayerRole playerRole;
	TeamColor teamColor;
	float batteryLevel;
	bool changed;
	bool goalBlueSeen;
	bool goalMineSeen;
	bool goalOpponentSeen;
	bool goalYellowSeen;
	bool kickOff;
};

struct RdkImageData
{
	unsigned char data[640*480*3]; // maximum allowed image size
	int width;
	int height;
	int bpp;
	bool changeParams;
	NaoCameraParams cameraParams[NUMBEROFCAMERAS];
	int selectedCameraID;
	bool fastSwitchCamera;
};

struct RdkAllSensorData
{
	RdkSensorData sensors;
	RdkImageData vision;
};


#endif   /* ----- #ifndef SENSORDATASHM_INC  ----- */
