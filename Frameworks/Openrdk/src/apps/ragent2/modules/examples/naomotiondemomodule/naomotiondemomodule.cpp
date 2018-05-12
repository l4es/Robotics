#define MODULE_NAME "NaoMotionDemoModule"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE MODULE_NAME

#include "naomotiondemomodule.h"

#include <nao/object/rnaojoints.h>

#define PROPERTY_OUT_MOTION_JOINTS  "out/motionJoints"
#define PROPERTY_OUT_INTERPOLATION  "out/interpolation"
#define PROPERTY_IN_BODY_JOINTS     "in/bodyJoints"
#define PROPERTY_IN_ACTION          "in/action"


namespace RDK2 { namespace RAgent {

using namespace Nao;

bool NaoMotionDemoModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		// put false in the second parameter if you want the module disabled for default
		// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, true);
		session->createString(PROPERTY_IN_ACTION, "Action to execute", "NONE");
		
		//session->setVolatile(PROPERTY_IN_ACTION);
		// here you declare the properties of your module
		// in the repository they have the name of this module as prefix, e.g. /yourModuleName/maxSpeed
		session->createStorage("RNaoJoints", PROPERTY_OUT_MOTION_JOINTS, "Target Joints to send to the Nao");
		session->setObject(PROPERTY_OUT_MOTION_JOINTS, new RNaoJoints());
		session->createStorage("RNaoJoints", PROPERTY_IN_BODY_JOINTS, "Current Joints configuration of the Nao");
		session->setObject(PROPERTY_IN_BODY_JOINTS, new RNaoJoints());
		
		session->createInt(PROPERTY_OUT_INTERPOLATION, "Interpolation mode (0,1)", 0);
				
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

// bool NaoMotionDemoModule::initInterfaceProperties() { }

bool NaoMotionDemoModule::init()
{
	// in the init() function you should initialize everything your module needs (drivers initialization,
	// sockets, and so on); all modules wait for all init() before they start
	SESSION_TRY_START(session)
		// here you can declare the events you are waiting in the main exec() loop,
		// for example:
		session->listenToTimer(200.);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void NaoMotionDemoModule::exec()
{
	while (session->wait(), !exiting) {
		
		SESSION_TRY_START(session)

		session->lock(PROPERTY_OUT_MOTION_JOINTS, HERE);
		RNaoJoints* targetjoints = session->getObjectAsL<RNaoJoints>(PROPERTY_OUT_MOTION_JOINTS);
		RNaoJoints* currentjoints = session->getObjectCloneAs<RNaoJoints>(PROPERTY_IN_BODY_JOINTS);
		string action = session->getString(PROPERTY_IN_ACTION);

		if (action!="NONE") {
		
			RDK_DEBUG_STREAM("Executing " << action);

			if (action=="LARM UP") {		
				(*targetjoints)[NaoJoints::L_SHOULDER_ROLL] = M_PI/6.0;
				(*targetjoints)[NaoJoints::L_SHOULDER_PITCH] = -M_PI/2.0;
				(*targetjoints)[NaoJoints::L_ELBOW_ROLL] = -0.2;
				(*targetjoints)[NaoJoints::L_ELBOW_YAW] = 0.0;
			}
			else if (action=="LARM DOWN") {		
				(*targetjoints)[NaoJoints::L_SHOULDER_ROLL] = M_PI/6.0;
				(*targetjoints)[NaoJoints::L_SHOULDER_PITCH] = M_PI/8.0;
				(*targetjoints)[NaoJoints::L_ELBOW_ROLL] = -M_PI/4.0;
				(*targetjoints)[NaoJoints::L_ELBOW_YAW] = 0.0;
			}

			session->setString(PROPERTY_IN_ACTION,"NONE");
			session->setInt(PROPERTY_OUT_INTERPOLATION,INTERPOLATION_SMOOTH);
			session->valueChanged(PROPERTY_OUT_MOTION_JOINTS);

		} 
		
		delete currentjoints;

		session->unlock(PROPERTY_OUT_MOTION_JOINTS);

		SESSION_END_CATCH_TERMINATE(session)
				
	}
}

// implement this if you need to force the exec to go out of the loop
// if the thread may be waiting not for the session->wait() semaphore:
// on closing, the main thread will set the exiting variables, call this exitRequested()
// function and then signal the session semaphores
void NaoMotionDemoModule::exitRequested() { 
}

// implement this if you need to clean up things after the exec has exited
// void NaoMotionDemoModule::cleanup() { }

// void NaoMotionDemoModule::asyncAgentCmd(cstr cmd)
// {
//	SESSION_TRY_START(asyncSession)
//	// here you can parse 'cmd'
//	SESSION_END_CATCH_TERMINATE(asyncSession)
// }

MODULE_FACTORY(NaoMotionDemoModule);

}} // namespace
