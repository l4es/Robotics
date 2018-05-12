#define LOGGING_MODULE MODULE_NAME

#include "naoqiledsmodule.h"
#include "naoqiledsmodule_names.h"
#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/sharedmemory/sensordatashm.h>
#include <rdkcore/rgeometry/rpointstat.h>

namespace RDK2
{
	namespace RAgent
	{
		bool NaoQiLedsModule::initConfigurationProperties()
		{
			SESSION_TRY_START(session)
				Common::createDefaultProperties(session,true);
				
				session->createString(PROPERTY_GAME_CONTROLLER,"Status of the game (initial, ready, set, play, finish)","play");
				session->createInt(PROPERTY_PENALIZED,"If the value is equals to 0 then the robot has no penalization, otherwise yes",0);
				session->createBool(PROPERTY_KICK_OFF,"The value of kick off",false);
				session->createBool(PROPERTY_BALL_SEEN,"If the value is equals to true then the robot has seen the ball, otherwise no",false);
				session->createStorage("RPolarPointCov",PROPERTY_COORDINATE_BALL_FIELD,"Coordinate of the ball in the field");
				session->createInt(PROPERTY_TEAM_COLOR,"The color of the team",TEAM_UNKNOWN);
				session->createDouble(PROPERTY_BALL_NEAR,"The value that will consider as maximum distance for near ball",RDouble::REAL,0.40);
				session->createDouble(PROPERTY_BALL_MIDDLE_DISTANCE,"The value that will consider as maximum distance for middle distance of the ball",RDouble::REAL,0.80);
				
				session->setVolatile(PROPERTY_GAME_CONTROLLER);
				session->setVolatile(PROPERTY_PENALIZED);
				session->setVolatile(PROPERTY_KICK_OFF);
				session->setVolatile(PROPERTY_BALL_SEEN);
				session->setVolatile(PROPERTY_COORDINATE_BALL_FIELD);
				session->setVolatile(PROPERTY_TEAM_COLOR);
			SESSION_END(session)
				return true;
			SESSION_CATCH_TERMINATE(session)
				return false;
		}
		
		bool NaoQiLedsModule::init()
		{
			SESSION_TRY_START(session)
				shmem = new SharedMemory(SharedMemory::LEDS,sizeof(RdkLedsData));
				
				session->listenToTimer(100.);
				session->listen(PROPERTY_BALL_SEEN);
				session->listen(PROPERTY_GAME_CONTROLLER);
				session->listen(PROPERTY_KICK_OFF);
				session->listen(PROPERTY_PENALIZED);
				session->listen(PROPERTY_TEAM_COLOR);
				
				session->registerTimerEventHandler(SESSION_EVENT(timerEvent));
				session->registerPropertyUpdateEventHandler(SESSION_EVENT(propertyEventPropertyBallSeen));
				session->registerPropertyUpdateEventHandler(SESSION_EVENT(propertyEventPropertyKickOff));
				session->registerPropertyUpdateEventHandler(SESSION_EVENT(propertyEventPropertyGameController));
				session->registerPropertyUpdateEventHandler(SESSION_EVENT(propertyEventPropertyPenalized));
				session->registerPropertyUpdateEventHandler(SESSION_EVENT(propertyEventPropertyTeamColor));
			SESSION_END(session)
				return true;
			SESSION_CATCH_TERMINATE(session)
				return false;
		}
		
		void NaoQiLedsModule::exec()
		{
			while (session->wait(), !exiting)
			{
				SESSION_TRY_START(session)
					session->processEvents();
				SESSION_END_CATCH_TERMINATE(session)
			}
		}
		
		bool NaoQiLedsModule::propertyEventPropertyBallSeen(const Event*)
		{
			RPolarPointCov* ball_position_field;
			
			RdkLedsData* command = static_cast<RdkLedsData*>(shmem->getEntry());
			
			if (!session->getBool(PROPERTY_BALL_SEEN))
			{
				command->ball_distance = BALL_LOST;
				command->changed = true;
			}
			else
			{
				session->lock(PROPERTY_COORDINATE_BALL_FIELD,HERE);
				ball_position_field = session->getObjectAsL<RPolarPointCov>(PROPERTY_COORDINATE_BALL_FIELD);
				
				if (ball_position_field->rho < session->getDouble(PROPERTY_BALL_NEAR))
				{
					command->ball_distance = BALL_NEAR;
					command->changed = true;
				}
				else if (ball_position_field->rho < session->getDouble(PROPERTY_BALL_MIDDLE_DISTANCE))
				{
					command->ball_distance = BALL_MIDDLE_DISTANCE;
					command->changed = true;
				}
				else
				{
					command->ball_distance = BALL_FAR;
					command->changed = true;
				}
				
				session->unlock(PROPERTY_COORDINATE_BALL_FIELD);
			}
			
			shmem->signal(NAOQI);
			
			return true;
		}
		
		bool NaoQiLedsModule::propertyEventPropertyGameController(const Event*)
		{
			string game_state;
			int penalized;
			
			penalized = session->getInt(PROPERTY_PENALIZED);
			
			if (penalized == 0)
			{
				game_state = session->getString(PROPERTY_GAME_CONTROLLER);
				
				RdkLedsData* command = static_cast<RdkLedsData*>(shmem->getEntry());
				
				if (game_state == "initial")
				{
					command->game_state = GAME_INITIAL;
					command->changed = true;
				}
				else if (game_state == "ready")
				{
					command->game_state = GAME_READY;
					command->changed = true;
				}
				else if (game_state == "set")
				{
					command->game_state = GAME_SET;
					command->changed = true;
				}
				else if (game_state == "play")
				{
					command->game_state = GAME_PLAY;
					command->changed = true;
				}
				else if (game_state == "finish")
				{
					command->game_state = GAME_FINISH;
					command->changed = true;
				}
				
				shmem->signal(NAOQI);
			}
			
			return true;
		}
		
		bool NaoQiLedsModule::propertyEventPropertyKickOff(const Event*)
		{
			RdkLedsData* command = static_cast<RdkLedsData*>(shmem->getEntry());
			
			command->kick_off = session->getBool(PROPERTY_KICK_OFF);
			command->changed = true;
			
			shmem->signal(NAOQI);
			
			return true;
		}
		
		bool NaoQiLedsModule::propertyEventPropertyPenalized(const Event*)
		{
			int penalized;
			
			penalized = session->getInt(PROPERTY_PENALIZED);
			
			RdkLedsData* command = static_cast<RdkLedsData*>(shmem->getEntry());
			
			if (penalized != 0)
			{
				command->game_state = GAME_PENALIZED;
				command->changed = true;
			}
			
			shmem->signal(NAOQI);
			
			return true;
		}
		
		bool NaoQiLedsModule::propertyEventPropertyTeamColor(const Event*)
		{
			int team_color;
			
			team_color = session->getInt(PROPERTY_TEAM_COLOR);
			
			RdkLedsData* command = static_cast<RdkLedsData*>(shmem->getEntry());
			
			if (team_color == 0) command->team_color = TEAM_BLUE;
			else if (team_color == 1) command->team_color = TEAM_RED;
			else command->team_color = TEAM_UNKNOWN;
			
			command->changed = true;
			
			shmem->signal(NAOQI);
			
			return true;
		}
		
		bool NaoQiLedsModule::timerEvent(const Event*)
		{
			return true;
		}
		
		MODULE_FACTORY(NaoQiLedsModule);
	}
}
