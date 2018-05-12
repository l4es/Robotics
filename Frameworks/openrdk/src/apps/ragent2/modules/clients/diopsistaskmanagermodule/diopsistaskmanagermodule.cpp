#define MODULE_NAME "DiopsisTaskmanagerModule"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/ns.h>
#define LOGGING_MODULE MODULE_NAME

#include "diopsistaskmanagermodule.h"
#include "diopsistaskmanagermodule_names.h"

#include "dsp/DSPAbstract_ResourceManager.h"


namespace RDK2 { namespace RAgent {

	bool DiopsisTaskmanagerModule::initConfigurationProperties()
	{
		SESSION_TRY_START(session)
			// put false in the second parameter if you want the module disabled for default
			// add a third parameter valued false if you don't want the state property
			Common::createDefaultProperties(session, true);
		// here you declare the properties of your module
		// in the repository they have the name of this module as prefix, e.g. /yourModuleName/maxSpeed
		// session->createString(PROPERTY_MY_BEAUTIFUL_STRING, "My beautiful string", "Default value");
		// session->createDouble(PROPERTY_MAX_SPEED, "Maximum speed", RDouble::MM_SEC, 100.);
		// // example of enum creation
		// ENUM_CREATE(whichStalled);
		// ENUM_ITEM(whichStalled, NONE, "None", "No wheel stalled");
		// ENUM_ITEM(whichStalled, LEFT, "Left", "Left wheels stalled");
		// ENUM_ITEM(whichStalled, RIGHT, "Right", "Right wheels stalled");
		// ENUM_ITEM(whichStalled, BOTH, "Both", "Full robot stall");
		// session->createEnum(PROPERTY_ROBOT_WHICH_STALLED, "Which robot wheel stalled", whichStalled, NONE, INFO);
		// // example of image creation
		// session->createImage(PROPERTY_MY_IMAGE, "My pretty image", width, height, type);
		// // example of map creation
		// session->createMap(PROPERTY_MY_MAP, "My ugly map", x, y, theta, realWidth, bitmapWidth, bitmapHeight);
		// // example of vector creation
		// session->createVector<RType>(PROPERTY_MY_VECTOR, "My vector");
		session->createBool(PROPERTY_CMD_START, "Start demo", false);
		session->setVolatile(PROPERTY_CMD_START);

		session->createString(PROPERTY_STATUS, "Status of task manager", DTM_IDLE);
		session->setVolatile(PROPERTY_STATUS);

		session->createString(PROPERTY_PARAMS_TASK1_NAME, "Name for task1 (not implemented yet)", "");
		session->createInt(PROPERTY_PARAMS_TASK1_ADDRESS, "Memory Address for task1", 5000);
		session->createInt(PROPERTY_PARAMS_TASK1_PRIORITY, "Priority for task1",DA_PRIORITY_LOW);

		session->createString(PROPERTY_PARAMS_TASK2_NAME, "Name for task (not implemented yet)", "");
		session->createInt(PROPERTY_PARAMS_TASK2_ADDRESS, "Memory Address for task2", 6000);
		session->createInt(PROPERTY_PARAMS_TASK2_PRIORITY, "Priority for task2", DA_PRIORITY_LOW);

		session->createDouble(PROPERTY_PARAMS_TASK1_VARIABLE, "Dummy variable for task1", RDouble::OTHER, 0.);
		session->createDouble(PROPERTY_PARAMS_TASK2_VARIABLE, "Dummy variable for task2", RDouble::OTHER, 0.);

		SESSION_END(session)
			return true;
		SESSION_CATCH_TERMINATE(session)
			return false;
	}

	// bool DiopsisTaskmanagerModule::initInterfaceProperties() { }

	bool DiopsisTaskmanagerModule::init()
	{
		// in the init() function you should initialize everything your module needs (drivers initialization,
		// sockets, and so on); all modules wait for all init() before they start
		SESSION_TRY_START(session)
			// here you can declare the events you are waiting in the main exec() loop,
			// for example:
		session->listenToTimer(.100);
		session->listen(PROPERTY_PARAMS_TASK1_VARIABLE);
		session->listen(PROPERTY_PARAMS_TASK2_VARIABLE);

		session->registerPropertyUpdateEventHandler(SESSION_EVENT(propertyEventVariable),PROPERTY_PARAMS_TASK1_VARIABLE);
		session->registerPropertyUpdateEventHandler(SESSION_EVENT(propertyEventVariable),PROPERTY_PARAMS_TASK2_VARIABLE);

#ifndef __PC__
		DSPAbstractRM_Init();
#endif
		SESSION_END(session)
			return true;
		SESSION_CATCH_TERMINATE(session)
			return false;
	}

	void DiopsisTaskmanagerModule::exec()
	{
		/* :WORKAROUND:10/21/2009 03:19:08 PM:lm: 16000 is the address in mAgicV memory */
		magicParams = reinterpret_cast<DA_TLParams*>(16000);
		RDK_INFO_STREAM("Please start magic from the debugger...\n");
		bool running = false;
		phase = DTM_START;
		currentTaskBasename = TASK1_BASENAME;
		task1Var = reinterpret_cast<DA_double*>(_M_STRUCTF(DA_double*, (DA_double*)(16208)));
		task2Var = reinterpret_cast<DA_double*>(_M_STRUCTF(DA_double*, (DA_double*)(16210)));
		while (session->wait(), !exiting)
		{
			SESSION_TRY_START(session)
				if (session->getBool(PROPERTY_MODULE_ENABLED))
				{
					session->processEvents();
					if (session->getBool(PROPERTY_CMD_START))
					{
						running = doInit();
						if (running)
						{
							checkTasks();
						}
					}
				}
				else
				{
					session->unlisten(PROPERTY_PARAMS_TASK1_VARIABLE);
					session->unlisten(PROPERTY_PARAMS_TASK2_VARIABLE);
				}
			SESSION_END_CATCH_TERMINATE(session)
		}
	}

	bool DiopsisTaskmanagerModule::doInit()
	{
		string status = session->getString(PROPERTY_STATUS);
		if (status == DTM_ALLOC_MEMORY)
		{
			if (phase == DTM_START)
			{
				RDK_TRACE_STREAM("Starting magicTask");
				if ((_M_STRUCTI(DA_TLParams, magicParams)->iRegisteredTasksNum))
				{
					magicTasks = (DA_TLTasks *)_M_STRUCTI(DA_TLParams, magicParams)->pTasksInfo;
				}
				DSPAbstractTM_GetMsg(magicTasks, &msg, timeout);
				//printf("Code: %d - Address: %d\n", msg.iMsg, msg.lArg);
				msg.iMsg = DA_TM_MSG_ALLOCMEMORY;
				msg.lArg = sizeof(DA_TLCreateTaskArgs);
				phase = DTM_SND;
			}
			else if (phase == DTM_SND)
			{
				if (DSPAbstractTM_PutMsg(magicTasks, &msg, timeout) == DA_SUCCESS)
				{
					phase = DTM_RCV;
				}
			}
			else if (phase == DTM_RCV && (DSPAbstractTM_GetMsg(magicTasks, &msg, timeout) != DA_SUCCESS))
			{
				if (msg.iMsg == DA_TM_MSG_ALLOCMEMORY_DONE)
				{
					phase = DTM_SND;
					session->setString(PROPERTY_STATUS,DTM_TASK_START);
				}
				else
				{
					phase = DTM_IDLE;
					session->setString(PROPERTY_STATUS,DTM_ALLOC_MEMORY_ERROR);
				}
			}
			else
			{
				RDK_ERROR_STREAM("Unknown phase [" << phase << "]");
			}
		}
		else if (status == DTM_TASK_START)
		{
			return startTask(TASK1_BASENAME) &&
				startTask(TASK2_BASENAME);
		}
		return false;
	}

	bool DiopsisTaskmanagerModule::startTask(string basename)
	{
		if (basename == currentTaskBasename)
		{
			if (phase == DTM_SND)
			{
				RDK_TRACE_STREAM("Starting task " << basename);
				msg.iMsg = DA_TM_MSG_CREATETASK;
				larg = msg.lArg;
				mArgs = reinterpret_cast<DA_TLCreateTaskArgs*>((_DBIOS_DM_I_mmapped_base + (DA_long)(larg)));
				mArgs->pName = 0; //session->getString(basename + "/" + TASK_NAME);
				mArgs->lAddress = session->getInt(basename + "/" + TASK_ADDRESS); // FIXME check *.map vTask or linker.bcf
				mArgs->iPriority = session->getInt(basename + "/" + TASK_PRIORITY);
				mArgs->iQueueElemNum = 1;
				if (DSPAbstractTM_PutMsg(magicTasks, &msg, timeout) == DA_SUCCESS)
				{
					phase = DTM_RCV;
				}
			}
			else if (phase == DTM_RCV)
			{
				DSPAbstractTM_GetMsg(magicTasks, &msg, timeout);
				if (basename == TASK1_BASENAME)
					currentTaskBasename = TASK2_BASENAME;
				else if (basename == TASK2_BASENAME)
				{
					currentTaskBasename = "none";
					session->setString(PROPERTY_STATUS,DTM_RUNNING);
				}
				return true;
			}
		}
		return false;
	}

	void DiopsisTaskmanagerModule::checkTasks()
	{
		session->setDouble(PROPERTY_PARAMS_TASK1_VARIABLE, *task1Var);
		session->setDouble(PROPERTY_PARAMS_TASK2_VARIABLE, *task2Var);
	}

	bool DiopsisTaskmanagerModule::propertyEventVariable(const Event* e)
	{
		const EventPropertyUpdate* event = dynamic_cast<const EventPropertyUpdate*>(e);
		if (event)
		{
			if (event->propertyUrl == PROPERTY_PARAMS_TASK1_VARIABLE)
			{
				*task1Var = session->getDouble(PROPERTY_PARAMS_TASK1_VARIABLE);
			}
			else if (event->propertyUrl == PROPERTY_PARAMS_TASK2_VARIABLE)
			{
				*task2Var = session->getDouble(PROPERTY_PARAMS_TASK2_VARIABLE);
			}
			else
			{
				return false;
			}
			return true;
		}
		return false;
	}
	// implement this if you need to force the exec to go out of the loop
	// if the thread may be waiting not for the session->wait() semaphore:
	// on closing, the main thread will set the exiting variables, call this exitRequested()
	// function and then signal the session semaphores
	// void DiopsisTaskmanagerModule::exitRequested() { }

	// implement this if you need to clean up things after the exec has exited
	// void DiopsisTaskmanagerModule::cleanup() { }

	// void DiopsisTaskmanagerModule::asyncAgentCmd(cstr cmd)
	// {
	//	SESSION_TRY_START(asyncSession)
	//	// here you can parse 'cmd'
	//	SESSION_END_CATCH_TERMINATE(asyncSession)
	// }

	MODULE_FACTORY(DiopsisTaskmanagerModule);

}} // namespace
