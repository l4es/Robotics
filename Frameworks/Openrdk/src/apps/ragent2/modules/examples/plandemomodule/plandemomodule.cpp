#define MODULE_NAME "PlanDemoModule"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE MODULE_NAME

#include "plandemomodule.h"

#include <nao/object/rnaojoints.h>
#include <nao/object/rnaosensors.h>

#define PROPERTY_IN_AVGINT   "in/avg_intensity"
#define PROPERTY_IN_SENSORS  "in/sensors"
#define PROPERTY_OUT_ACTI0N  "out/action"
#define PROPERTY_OUT_TEXT    "out/saytext"
#define PROPERTY_PARAM_THRESHOLD    "param/threshold"
#define PROPERTY_CMD_SEND    "cmd/send"

namespace RDK2 { namespace RAgent {

using namespace Nao;

bool PlanDemoModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		Common::createDefaultProperties(session, true);		
		session->createDouble(PROPERTY_IN_AVGINT, "Avg image intensity", RDouble::REAL, 0.0);
		session->createStorage("RNaoSensors", PROPERTY_IN_SENSORS, "Nao sensor values");
		session->setObject(PROPERTY_IN_SENSORS, new RNaoSensors());

		session->createString(PROPERTY_OUT_ACTI0N, "Action to execute", "NONE");
		session->createString(PROPERTY_OUT_TEXT, "Text to say");
		session->createDouble(PROPERTY_PARAM_THRESHOLD, "# Intensity threshold for LARM UP/DOWN actions", RDouble::REAL, 20.0);
		
		session->createString(PROPERTY_CMD_SEND,"Remote command to send", std::string(""));
		session->setVolatile(PROPERTY_CMD_SEND);

	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

// bool PlanDemoModule::initInterfaceProperties() { }

bool PlanDemoModule::init()
{
	// in the init() function you should initialize everything your module needs (drivers initialization,
	// sockets, and so on); all modules wait for all init() before they start
	SESSION_TRY_START(session)
		// here you can declare the events you are waiting in the main exec() loop,
		// for example:
		session->listenToTimer(500.);
	
		lastY=250; // larm down

	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void PlanDemoModule::exec()
{
	usleep(1000); state=0;

	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)

		double y = session->getDouble(PROPERTY_IN_AVGINT);

		// cout << "plan: intensity received " << y <<  endl;
		session->lock(PROPERTY_IN_SENSORS, HERE);
		RNaoSensors* rsensors = session->getObjectAsL<RNaoSensors>(PROPERTY_IN_SENSORS);
		//sensors->
		//cout << *sensors << endl;
		//float bumpers=0;
		//for (int i=0; i<4; i++) bumpers+=rsensors->sensors.bumpers[i];
		//// cout << "Bumpers " << bumpers << endl;
		
		//float force=0;
		//for (size_t i=0; i<FORCE_SENSOR_SIZE; i++) 
		//  force+=rsensors->sensors.forceSensor[i];
		// cout << "Force Sensor " << force << endl;
		
		session->unlock(PROPERTY_IN_SENSORS);
		
// FIXME : if this works, remove PROPERTY_OUT_ACTI0N
		
		double thr=session->getDouble(PROPERTY_PARAM_THRESHOLD);
		if (y==0.0) { // nothing to do
			session->setString(PROPERTY_CMD_SEND,"NONE");
		}

		if (y<thr && lastY>thr) { // swicth to dark image
			session->setString(PROPERTY_CMD_SEND,"LARM UP");
			session->setString(PROPERTY_OUT_TEXT,"I cannot see.");
		}
		else if (y>thr && lastY<thr) { // switch to bright
			session->setString(PROPERTY_CMD_SEND,"LARM DOWN");
			session->setString(PROPERTY_OUT_TEXT,"OK, now it's better.");
		}
		else { // nothing to do
			session->setString(PROPERTY_CMD_SEND,"NONE");
		}

		lastY=y;

		//if (force>20000 && state==2) state=3;
		
		//switch (state) {
		//  case 0: // wait
				
		//    if (bumpers>=2) state++;
		//    break;
		//  case 1:
		//    cout << "Start Walk" << endl; state++;
		//    session->setString(PROPERTY_CMD_SEND,"setSpeed 1 0");
		//    break;
		//  case 2: // exec walk
		//    // cout << "Walk" << endl;
		//    break;
		//  case 3:
		//    cout << "Stop" << endl; session->setString(PROPERTY_CMD_SEND,"setSpeed 0 0");
		//    state=0;
		//    break;
		//}
		
		SESSION_END_CATCH_TERMINATE(session)
	}
}

// implement this if you need to force the exec to go out of the loop
// if the thread may be waiting not for the session->wait() semaphore:
// on closing, the main thread will set the exiting variables, call this exitRequested()
// function and then signal the session semaphores
// void PlanDemoModule::exitRequested() { }

// implement this if you need to clean up things after the exec has exited
// void PlanDemoModule::cleanup() { }

// void PlanDemoModule::asyncAgentCmd(cstr cmd)
// {
//	SESSION_TRY_START(asyncSession)
//	// here you can parse 'cmd'
//	SESSION_END_CATCH_TERMINATE(asyncSession)
// }

MODULE_FACTORY(PlanDemoModule);

}} // namespace
