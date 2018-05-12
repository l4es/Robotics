#include "rledsserver.h"
#include <nao-objects/sensordatashm.h>
#include <string>

using namespace AL;
using namespace std;

#define BLUE 0x000000FF
#define GREEN 0x0000FF00
#define YELLOW 0x00FFFF00
#define PURPLE 0xC062FF
#define RED 0x00FF0000
#define WHITE 0x00FFFFFF

RLedsServer::RLedsServer(ALPtr<ALBroker> broker, const string& name) : ALModule(broker,name)
{
	setModuleDescription("This module manage the leds on the robot.");
	
	functionName("isPlugged",getName(),"Check if the Shared Memory is correctly plugged in");
	setReturn("isPlugged","True if the shared memory is connected");
	
	BIND_METHOD(RLedsServer::isPlugged);
}

RLedsServer::~RLedsServer()
{
	if (shm_led != NULL)
	{
		delete shm_led;
	}
	
	shm_led = NULL;
}

void RLedsServer::init()
{
	// Create a proxy to the logger module
	// If this fails, we throw an exception and the module will not be registered.
	try
	{
		logger_proxy = getParentBroker()->getLoggerProxy();
	}
	catch (const ALError& e)
	{
		throw ALError(getName(),"RLedsServer()","Fail to create a proxy to ALLogger module. Msg " + e.toString() + "\nModule will abort.");
	}
	
	// Create a proxy to the led module
	// If this fails, we throw an exception and the module will not be registered.
	try
	{
		led_proxy = getParentBroker()->getProxy("ALLeds");
	}
	catch (ALError& e)
	{
#ifdef OpenRDK_ARCH_GENERIC
		logger_proxy->warn(getName(),"generic platform detected: Could not create a proxy to ALLeds module");
#else
		throw ALError(getName(),"RLedsServer()","Fail to create a proxy to ALLeds module. Msg " + e.toString() + "\nModule will abort.");
#endif
	}
	
	shm_led = new RDK2::SharedMemory(RDK2::SharedMemory::LEDS,sizeof(RdkLedsData));
	
	if (shm_led->pluggingSuccessful())
	{
		logger_proxy->lowInfo(getName(),"Succesfully plugged in Shared Memory for LEDS");
	}
	else
	{
		logger_proxy->error(getName(),"Cannot create ShM LEDS");
	}
	
	pthread_create(&leds_manager_thread_id,NULL,(void*(*)(void*))ledsManagerThread,this);
	
	logger_proxy->info(getName()," Init LEDS done.");
}

void RLedsServer::ledsManager()
{
	int ball_distance, game_state, player_role, team_color;
	bool goal_blue_seen, goal_yellow_seen, goal_mine_seen, goal_opponent_seen, kick_off;
	
	while (true)
	{
		shm_led->wait(NAOQI);
#ifdef OpenRDK_ARCH_GENERIC
		logger_proxy->warn(getName(),"generic platform detected: Led lights cannot be set");
#else
		RdkLedsData* command = static_cast<RdkLedsData*>(shm_led->getEntry());
		
		bool& changed = command->changed;
		
		if (changed)
		{
			game_state = command->game_state;
			kick_off = command->kick_off;
			team_color = command->team_color;
			goal_blue_seen = command->goal_blue_seen;
			goal_yellow_seen = command->goal_yellow_seen;
			goal_mine_seen = command->goal_mine_seen;
			goal_opponent_seen = command->goal_opponent_seen;
			ball_distance = command->ball_distance;
			player_role = command->player_role;
			
			setGameState(game_state);
			setKickOff(kick_off,game_state);
			setTeamColor(team_color);
			setGoalSeen(goal_blue_seen,goal_yellow_seen,goal_mine_seen,goal_opponent_seen);
			setBallSeen(ball_distance);
			setPlayerRole(player_role);
			
			changed = false;
		}
#endif
		
		shm_led->signal(RDK);
	}
}

void RLedsServer::setBallSeen(int ball_distance)
{
	string led;
	
	led = "LeftFaceLeds";
	
	if (ball_distance == VERY_NEAR_BALL)
	{
		// The robot has seen the ball very near and the "LeftFaceLeds" must be turned on with green color.
		led_proxy->callVoid("fadeRGB",led,GREEN,0.0f);
	}
	else if (ball_distance == NEAR_BALL)
	{
		// The robot has seen the ball near and the "LeftFaceLeds" must be turned on with yellow color.
		led_proxy->callVoid("fadeRGB",led,YELLOW,0.0f);
	}
	else if (ball_distance == MIDDLE_DISTANCE_BALL)
	{
		// The robot has seen the ball on the middle distance and the "LeftFaceLeds" must be turned on with purple color.
		led_proxy->callVoid("fadeRGB",led,PURPLE,0.0f);
	}
	else if (ball_distance == FAR_BALL)
	{
		// The robot has seen the ball far and the "LeftFaceLeds" must be turned on with red color.
		led_proxy->callVoid("fadeRGB",led,RED,0.0f);
	}
	else if (ball_distance == LOST_BALL)
	{
		// The robot has lost the ball and the "LeftFaceLeds" must be turned off.
		led_proxy->callVoid("off",led);
	}
	else
	{
		// In this case there was a mistake in the code and the "LeftFaceLeds" will be turned off.
		led_proxy->callVoid("off",led);
	}
}

void RLedsServer::setGameState(int game_state)
{
	string led;
	
	led = "ChestLeds";
	
	if ((game_state == GAME_INITIAL) || (game_state == GAME_FINISH))
	{
		// In this state of game the "ChestLeds" must be turned off.
		led_proxy->callVoid("off",led);
	}
	else if (game_state == GAME_READY)
	{
		// In this state of game the "ChestLeds" must be turned on with blue color.
		led_proxy->callVoid("fadeRGB",led,BLUE,0.0f);
	}
	else if (game_state == GAME_SET)
	{
		// In this state of game the "ChestLeds" must be turned on with yellow color.
		led_proxy->callVoid("fadeRGB",led,YELLOW,0.0f);
	}
	else if (game_state == GAME_PLAY)
	{
		// In this state of game the "ChestLeds" must be turned on with green color.
		led_proxy->callVoid("fadeRGB",led,GREEN,0.0f);
	}
	else if (game_state == GAME_PENALIZED)
	{
		// In this state of game the "ChestLeds" must be turned on with red color.
		led_proxy->callVoid("fadeRGB",led,RED,0.0f);
	}
}

void RLedsServer::setGoalSeen(bool goal_blue_seen, bool goal_yellow_seen, bool goal_mine_seen, bool goal_opponent_seen)
{
	string led;
	
	led = "FaceLedsRight";
	
	if (goal_yellow_seen && goal_blue_seen)
	{
		// The robot has seen the blue goal and the yellow goal and the "FaceLedsRightExternal" and "FaceLedsRightTop" must be turned on with purple color.
		led_proxy->callVoid("fadeRGB",led + "External",PURPLE,0.0f);
		led_proxy->callVoid("fadeRGB",led + "Top",PURPLE,0.0f);
	}
	else if (goal_blue_seen)
	{
		// The robot has seen the blue goal and the "FaceLedsRightExternal" and "FaceLedsRightTop" must be turned on with blue color.
		led_proxy->callVoid("fadeRGB",led + "External",BLUE,0.0f);
		led_proxy->callVoid("fadeRGB",led + "Top",BLUE,0.0f);
	}
	else if (goal_yellow_seen)
	{
		// The robot has seen the yellow goal and the "FaceLedsRightExternal" and "FaceLedsRightTop" must be turned on with yellow color.
		led_proxy->callVoid("fadeRGB",led + "External",YELLOW,0.0f);
		led_proxy->callVoid("fadeRGB",led + "Top",YELLOW,0.0f);
	}
	else
	{
		// The robot doesn't seen either the blue goal or the yellow goal and the "FaceLedsRightExternal" and "FaceLedsRightTop" must be turned off.
		led_proxy->callVoid("off",led + "External");
		led_proxy->callVoid("off",led + "Top");
	}
	
	if (goal_mine_seen && goal_opponent_seen)
	{
		// The robot has seen the mine goal and the opponent goal and the "FaceLedsRightInternal" and "FaceLedsRightBottom" must be turned on with purple color.
		led_proxy->callVoid("fadeRGB",led + "Internal",PURPLE,0.0f);
		led_proxy->callVoid("fadeRGB",led + "Bottom",PURPLE,0.0f);
	}
	else if (goal_mine_seen)
	{
		// The robot has seen the mine goal and the "FaceLedsRightInternal" and "FaceLedsRightBottom" must be turned on with red color.
		led_proxy->callVoid("fadeRGB",led + "Internal",RED,0.0f);
		led_proxy->callVoid("fadeRGB",led + "Bottom",RED,0.0f);
	}
	else if (goal_opponent_seen)
	{
		// The robot has seen the opponent goal and the "FaceLedsRightInternal" and "FaceLedsRightBottom" must be turned on with green color.
		led_proxy->callVoid("fadeRGB",led + "Internal",GREEN,0.0f);
		led_proxy->callVoid("fadeRGB",led + "Bottom",GREEN,0.0f);
	}
	else
	{
		// The robot doesn't seen either the mine goal or the opponent goal and the "FaceLedsRightInternal" and "FaceLedsRightBottom" must be turned off.
		led_proxy->callVoid("off",led + "Internal");
		led_proxy->callVoid("off",led + "Bottom");
	}
}

void RLedsServer::setKickOff(bool is_kick_off, int game_state)
{
	string led;
	
	led = "RightFootLeds";
	
	if (is_kick_off && ((game_state == GAME_INITIAL) || (game_state == GAME_READY) || (game_state == GAME_SET)))
	{
		// In this case the "RightFootLeds" must be turned on with white color.
		led_proxy->callVoid("fadeRGB",led,WHITE,0.0f);
	}
	else
	{
		// In this case the "RightFootLeds" must be turned off.
		led_proxy->callVoid("off",led);
	}
}

void RLedsServer::setPlayerRole(int team_color)
{
	string led_after, led_before;
	
	led_before = "Ears/Led/Right/";
	led_after = "Deg/Actuator/Value";
	
	if (team_color == GOALIE)
	{
		// This is the configuration for goalie player.
		led_proxy->callVoid("off",led_before + "324" + led_after);
		led_proxy->callVoid("off",led_before + "288" + led_after);
		led_proxy->callVoid("off",led_before + "252" + led_after);
		led_proxy->callVoid("off",led_before + "216" + led_after);
		led_proxy->callVoid("off",led_before + "180" + led_after);
		led_proxy->callVoid("on",led_before + "144" + led_after);
		led_proxy->callVoid("on",led_before + "108" + led_after);
		led_proxy->callVoid("on",led_before + "72" + led_after);
		led_proxy->callVoid("on",led_before + "36" + led_after);
		led_proxy->callVoid("on",led_before + "0" + led_after);
	}
	else if (team_color == DEFENDER)
	{
		// This is the configuration for defender player.
		led_proxy->callVoid("off",led_before + "324" + led_after);
		led_proxy->callVoid("off",led_before + "288" + led_after);
		led_proxy->callVoid("on",led_before + "252" + led_after);
		led_proxy->callVoid("off",led_before + "216" + led_after);
		led_proxy->callVoid("off",led_before + "180" + led_after);
		led_proxy->callVoid("off",led_before + "144" + led_after);
		led_proxy->callVoid("on",led_before + "108" + led_after);
		led_proxy->callVoid("on",led_before + "72" + led_after);
		led_proxy->callVoid("on",led_before + "36" + led_after);
		led_proxy->callVoid("off",led_before + "0" + led_after);
	}
	else if (team_color == ATTACKER)
	{
		// This is the configuration for attacker player.
		led_proxy->callVoid("on",led_before + "324" + led_after);
		led_proxy->callVoid("on",led_before + "288" + led_after);
		led_proxy->callVoid("on",led_before + "252" + led_after);
		led_proxy->callVoid("on",led_before + "216" + led_after);
		led_proxy->callVoid("on",led_before + "180" + led_after);
		led_proxy->callVoid("off",led_before + "144" + led_after);
		led_proxy->callVoid("off",led_before + "108" + led_after);
		led_proxy->callVoid("off",led_before + "72" + led_after);
		led_proxy->callVoid("off",led_before + "36" + led_after);
		led_proxy->callVoid("off",led_before + "0" + led_after);
	}
	else if (team_color == SUPPORTER)
	{
		// This is the configuration for supporter player.
		led_proxy->callVoid("off",led_before + "324" + led_after);
		led_proxy->callVoid("on",led_before + "288" + led_after);
		led_proxy->callVoid("on",led_before + "252" + led_after);
		led_proxy->callVoid("on",led_before + "216" + led_after);
		led_proxy->callVoid("off",led_before + "180" + led_after);
		led_proxy->callVoid("off",led_before + "144" + led_after);
		led_proxy->callVoid("off",led_before + "108" + led_after);
		led_proxy->callVoid("on",led_before + "72" + led_after);
		led_proxy->callVoid("off",led_before + "36" + led_after);
		led_proxy->callVoid("off",led_before + "0" + led_after);
	}
	else
	{
		// This is the configuration for no role.
		led_proxy->callVoid("off",led_before + "324" + led_after);
		led_proxy->callVoid("off",led_before + "288" + led_after);
		led_proxy->callVoid("off",led_before + "252" + led_after);
		led_proxy->callVoid("off",led_before + "216" + led_after);
		led_proxy->callVoid("off",led_before + "180" + led_after);
		led_proxy->callVoid("off",led_before + "144" + led_after);
		led_proxy->callVoid("off",led_before + "108" + led_after);
		led_proxy->callVoid("off",led_before + "72" + led_after);
		led_proxy->callVoid("off",led_before + "36" + led_after);
		led_proxy->callVoid("off",led_before + "0" + led_after);
	}
}

void RLedsServer::setTeamColor(int team_color)
{
	string led;
	
	led = "LeftFootLeds";
	
	if (team_color == TEAM_RED)
	{
		// If the robot belongs to the red team then the "LeftFootLeds" must be turned on with red color.
		led_proxy->callVoid("fadeRGB",led,RED,0.0f);
	}
	else if (team_color == TEAM_BLUE)
	{
		// If the robot belongs to the blue team then the "LeftFootLeds" must be turned on with blue color.
		led_proxy->callVoid("fadeRGB",led,BLUE,0.0f);
	}
	else if (team_color == TEAM_UNKNOWN)
	{
		// If the robot doesn't belong to some team then the "LeftFootLeds" must be turned off.
		led_proxy->callVoid("off",led);
	}
}
