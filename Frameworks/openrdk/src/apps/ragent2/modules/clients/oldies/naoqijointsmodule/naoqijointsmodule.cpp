#define MODULE_NAME "NaoQiJointsModule"

#include "naoqijointsmodule.h"
#include <stdio.h>
#include <string.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/robot/robotmodule.h>
#include <nao/object/rnaojoints.h>
#include <nao/object/rnaosensors.h>
#include <nao/object/rcameramatrix.h>

//added by gianluca. It would be right to define our costants command in some place, but we have to decide which is this place!!!!
//#include <nao/naoqi-modules/motionserver.h> CHIEDERE A LM

#include <sstream>
#include <iterator>
#define LOGGING_MODULE MODULE_NAME
#define NOP "no op"

#define PROPERTY_IN_MOTION_JOINTS "in/motionJoints"
#define PROPERTY_IN_INTERPOLATION "in/interpolation"
#define PROPERTY_OUT_SENSOR_TIMESTAMP  "out/sensorTimeStamp"

//added by gianluca, verificare se è in (letta da altri) o out (scritta da qst modulo)
#define PROPERTY_BODYMOTION_CMD "in/motionCommand" // è giusto prevedere una nuova proprietà per i giunti della testa???
#define PROPERTY_HEADMOTION_CMD "in/motionHeadCommand"
#define PROPERTY_OUT_ISKICKING "out/iskicking"

#define PROPERTY_OUT_BODY_JOINTS  "out/bodyJoints"
#define PROPERTY_OUT_SENSORS      "out/sensors"
#define PROPERTY_OUT_BATTERY      "out/batteryCharge"
#define PROPERTY_OUT_ALLDATA	  "out/alldata"
#define PROPERTY_OUT_POSE	  "out/pose"
// #define PROPERTY_OUT_ENDFRAME	  "out/endframe"
#define PROPERTY_GOTO_TIME_FRAME  "params/gotoTimeFrame"
#define PROPERTY_GOTO_WITH_SPEED  "params/gotoWithSpeedValue"
#define PROPERTY_STIFFNESS        "params/stiffness"
#define PROPERTY_STIFFNESS_CHAIN  "params/stiffnessChain"
#define PROPERTY_MOTIONENABLED    "params/directMotionEnabled"

#define PROPERTY_DEBUG_ENABLED "params/debugEnabled"

namespace RDK2 { namespace RAgent {

using namespace Nao;

bool NaoQiJointsModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)

	Common::createDefaultProperties(session, true);

	session->createDouble(PROPERTY_GOTO_TIME_FRAME, "Naoqi gotoBodyAngles time frame", RDouble::SEC, 0.1);
	session->createInt(PROPERTY_GOTO_WITH_SPEED, "Naoqi gotoBodyAnglesWithSpeed speed value (0-100)", 25);
	session->createDouble(PROPERTY_STIFFNESS, "Naoqi stiffness value (0.0-1.0)", RDouble::REAL, -1);
	session->createInt(PROPERTY_STIFFNESS_CHAIN, "Naoqi stiffness chain (0-6)", 0);
	// 0=None, 1=Body, 2=Head, 3=LArm, 4=LLeg, 5=RLeg, 6=RArm
	
	//added by gianluca, aggiungere nuova proprietà per i giunti della testa?
	session->createString(PROPERTY_BODYMOTION_CMD, "Robot motion bodycommand in string format for DCM", NOP);
	session->createString(PROPERTY_HEADMOTION_CMD, "Robot motion headcommand in string format for DCM", NOP);
	session->createBool(PROPERTY_OUT_ISKICKING, "If robot is kicking (only for remote controller)", false);
	
	session->createStorage("RNaoJoints", PROPERTY_IN_MOTION_JOINTS, "Target Joints to send to the Nao");
	session->createStorage("RNaoJoints", PROPERTY_OUT_BODY_JOINTS, "Current Joints configuration of the Nao");
	session->setObject(PROPERTY_IN_MOTION_JOINTS, new RNaoJoints());
	session->setObject(PROPERTY_OUT_BODY_JOINTS, new RNaoJoints());

	session->createBool(PROPERTY_MOTIONENABLED,"If direct motion is enabled",true);
	session->createDouble(PROPERTY_OUT_BATTERY, "Battery charge (0.0-1.0)", RDouble::REAL, -1);
	session->setVolatile(PROPERTY_OUT_BATTERY);
	// cout << "PROPERTY_GOTO_WITH_SPEED = " << session->getInt(PROPERTY_GOTO_WITH_SPEED)  << endl;

	session->createStorage("RNaoSensors", PROPERTY_OUT_SENSORS, "Nao sensor values");
	session->setObject(PROPERTY_OUT_SENSORS, new RNaoSensors());
	
	session->createDouble(PROPERTY_OUT_SENSOR_TIMESTAMP,"Sensor timestamp",RDouble::SEC, 0.0);
	session->setVolatile(PROPERTY_OUT_SENSOR_TIMESTAMP);

	session->createInt(PROPERTY_IN_INTERPOLATION, "Motion interpolation mode (0,1)", INTERPOLATION_SMOOTH);

	session->createString(PROPERTY_OUT_POSE, "Robot joint pose in string format", "N/A");
	session->createString(PROPERTY_OUT_ALLDATA, "All data in string format", "N/A");

	session->createBool(PROPERTY_DEBUG_ENABLED, "Enable some output debug", false);
	session->setVolatile(PROPERTY_DEBUG_ENABLED);

	SESSION_END(session);
	return true;
	SESSION_CATCH_TERMINATE(session);
	return false;
}

bool NaoQiJointsModule::init()
{
	SESSION_TRY_START(session)

	shmmotion = new SharedMemory(SharedMemory::MOTION, sizeof(RDKMOTIONDATA));
	if (!shmmotion->pluggingSuccessful())
	{
		RDK_ERROR_STREAM("Cannot creatt shmMOTION!!!");
	}

	shmsensor = new SharedMemory(SharedMemory::SENSOR, sizeof(RDKSENSORDATA));

	RdkMotionData* data = static_cast<RdkMotionData*>(shmmotion->getEntry());

	RNaoJoints initPose;
	
	for (int i = 0; i < 22; i++) data->targetJointsValues[i] = initPose[i];
	data->executionTime = 3.0;
	data->withSpeedValue = 10; // session->getInt(PROPERTY_GOTO_WITH_SPEED);
	data->interpolationMode = INTERPOLATION_SMOOTH;
	data->shutdown = false;
	data->change_stiffness = true;
	data->stiffness_chain = session->getInt(PROPERTY_STIFFNESS_CHAIN);
	data->stiffness = session->getDouble(PROPERTY_STIFFNESS);
	//added by gianluca
	strcpy(data->cmd,NOP); //nessuna operazione ---> verificare dove mettere il .h della definizione delle costanti di operazioni
	strcpy(data->cmdHead,NOP);
#if 1	
	// added by massimo
	oldStiffnessChain = session->getInt(PROPERTY_STIFFNESS_CHAIN);
	oldStiffness = session->getDouble(PROPERTY_STIFFNESS);
	//
#endif
	shmmotion->signal(NAOQI); shmsensor->signal(NAOQI);

	session->listenToTimer(session->getDouble(PROPERTY_GOTO_TIME_FRAME)*1000.0/*/5.0*/);
	//session->listenToTimer(100.); // cambiato qui per avere lo stesso valore di motioncontrolmodule by gianluca
	
	// cout << "Timer = " << session->getDouble(PROPERTY_GOTO_TIME_FRAME)*1000.0 << endl;
	//session->listen(PROPERTY_IN_MOTION_JOINTS);
#if 1
	// added by massimo
	session->listen(PROPERTY_STIFFNESS_CHAIN);
	session->listen(PROPERTY_STIFFNESS);
#endif	//
	//added by gianluca
	session->listen(PROPERTY_BODYMOTION_CMD);
	session->registerPropertyUpdateEventHandler(SESSION_EVENT(motionCmdUpdateEvent), PROPERTY_BODYMOTION_CMD);
	
	session->listen(PROPERTY_HEADMOTION_CMD);
	session->registerPropertyUpdateEventHandler(SESSION_EVENT(motionCmdUpdateEvent), PROPERTY_HEADMOTION_CMD);
	// cout << "*** NaoJointsModule init ***" << endl;
	wait_for_sensor_reading=0;
	
	SESSION_END(session);
	return true;
	SESSION_CATCH_TERMINATE(session);
	return false;
}

bool NaoQiJointsModule::motionCmdUpdateEvent(const Event* e)
{	
	//std::cout << "IN MOTION CMD UPDATE EVENT "<< std::endl;
	
	string moduleName = '/' + getModuleName() + '/';
	
	if (session->getBool(PROPERTY_MOTIONENABLED)) {
	const EventPropertyUpdate* event = dynamic_cast<const EventPropertyUpdate*>(e);
	std::cout  << "EVENT URL "<< event->propertyUrl.c_str() << std::endl;
	
		shmmotion->wait(RDK);
			RdkMotionData * data = static_cast<RdkMotionData*>(shmmotion->getEntry());
			
			string commandBody = session->getString(PROPERTY_BODYMOTION_CMD);
			string commandHead = session->getString(PROPERTY_HEADMOTION_CMD);
			
			if(event->propertyUrl == (moduleName + PROPERTY_BODYMOTION_CMD)) strcpy((data->cmd),commandBody.c_str());
			else if(event->propertyUrl == (moduleName + PROPERTY_HEADMOTION_CMD)) strcpy((data->cmdHead),commandHead.c_str());
			
			//session->setString(PROPERTY_MOTION_CMD,NOP)
			
		shmmotion->signal(NAOQI);
	}
	return true;
}

void NaoQiJointsModule::exec()
{
			
	while (session->dontWait(), !exiting) {

		SESSION_TRY_START(session)
		//std::cout << "IN EXEC" << std::endl;
		//bool wtimer = session->wokenByEvent("EventTimer"); //unused
		session->processEvents(); //added by gianluca
				
		if (session->getBool(PROPERTY_MOTIONENABLED)) {
			
		shmmotion->wait(RDK);
		timestamp.setToNow(); 
		unsigned long ts = timestamp.getMsFromMidnight();
		
		// cout << "***** NaoqiJointsModule sending data. ****" << endl;
		RNaoJoints* targetjoints = session->getObjectCloneAs<RNaoJoints>(PROPERTY_IN_MOTION_JOINTS);

		RdkMotionData * data = static_cast<RdkMotionData*>(shmmotion->getEntry());
		for (int i = 0; i < JOINTS_VALUES_SIZE; i++) {
			data->targetJointsValues[i] = targetjoints->getValues()[i];
		}
		#if 1 //by gianluca
		data->executionTime = session->getDouble(PROPERTY_GOTO_TIME_FRAME);
		data->withSpeedValue = session->getInt(PROPERTY_GOTO_WITH_SPEED);
		//RDK_INFO_STREAM(" speed value = "<<data->withSpeedValue );
		data->interpolationMode = session->getInt(PROPERTY_IN_INTERPOLATION);
		data->shutdown = false;
		//strcpy(data->cmd,"joints");
		//data->cmd = session->getString(PROPERTY_MOTION_CMD);


		// added by massimo ( substitute with "data->change_stiffness = false;")
		newStiffnessChain = session->getInt(PROPERTY_STIFFNESS_CHAIN);
		newStiffness = session->getDouble(PROPERTY_STIFFNESS);

		if (oldStiffnessChain != newStiffnessChain || oldStiffness!=newStiffness){

			data->change_stiffness = true;
			data->stiffness_chain = newStiffnessChain;
			data->stiffness = newStiffness;

			oldStiffnessChain = newStiffnessChain;
			oldStiffness = newStiffness;

			RDK_INFO_STREAM("THE STIFFNESS CHANGED!!!");

		}

		else 	data->change_stiffness = false;
		//


		//timestamp.setToNow(); unsigned long ts = timestamp.getMsFromMidnight(); excluded by gianluca
		//cout << ts << " " << (ts-lastts) << "  naoqijoints signal to NAOQI - woken by (" <<
		//		(wtimer?"Timer":"Property") << ")" << endl;
		//cout << *targetjoints << endl;
		/*
		double darete = targetjoints->getValues()[0];
		cout << "da rete = " << darete << endl;
		cout << "joint 0 = " << data->targetJointsValues[0] << endl;
		//data->targetJointsValues[0] = darete;
		cout << "joint 0 = " << data->targetJointsValues[0] << endl;
		cout << "===========================" << endl;*/
#endif				 
		lastts = ts; //excluded by gianluca
	/*	timestamp.setToNow();
		
		unsigned long ts2 = timestamp.getMsFromMidnight();
		unsigned long ts3 = ts2 - ts1;
		
		std::cout << "TIMESTAMP 1 " << ts1 << std::endl;
		std::cout << "TIMESTAMP 2 " << ts2 << std::endl;
		std::cout << "DIFFTIME " << ts3 << std::endl;*/
		
		shmmotion->signal(NAOQI);
		
		delete targetjoints;
		
		}
		
		//wait_for_sensor_reading++;
		
		if (1) { // wait_for_sensor_reading>=5)  { c'era 1
		
			//wait_for_sensor_reading=0;
			
			shmsensor->wait(RDK);
			
			session->lock(PROPERTY_OUT_BODY_JOINTS, HERE);
			RNaoJoints* currentjoints = session->getObjectAsL<RNaoJoints>(PROPERTY_OUT_BODY_JOINTS);
	
			RdkSensorData *sensors = static_cast<RdkSensorData*>(shmsensor->getEntry());
			for (int i = 0; i < 22; i++) (currentjoints->getValues())[i] = sensors->jointsValues[i];
			
	
			//cout << "Joints read " << sensors->timestamp << ": " << *currentjoints ;
			
			session->unlock(PROPERTY_OUT_BODY_JOINTS);
	
			//cout << "Battery value = " << sensors->battery << endl;
			session->setDouble(PROPERTY_OUT_BATTERY,sensors->battery);
	
			session->lock(PROPERTY_OUT_SENSORS, HERE);
			RNaoSensors* currentsensors = session->getObjectAsL<RNaoSensors>(PROPERTY_OUT_SENSORS);
			currentsensors->sensors = *sensors;
			#if USE_NEW_SHM
			session->setBool(PROPERTY_OUT_ISKICKING,sensors->kicking);
			#endif
			// cout << *currentsensors << endl;
			session->unlock(PROPERTY_OUT_SENSORS);
			session->valueChanged(PROPERTY_OUT_SENSORS);

			shmsensor->signal(NAOQI);
			
			ostringstream ss;
			ss << *currentsensors << endl;
			session->setString(PROPERTY_OUT_ALLDATA,ss.str());
			
			//cout << *currentsensors << endl;
			
			ostringstream ss2;
			for (int k=0; k<JOINTS_VALUES_SIZE; k++) { 
				ss2 << currentsensors->sensors.jointsValues[k] << " "; 
			}
			session->setString(PROPERTY_OUT_POSE,ss2.str());

			if (session->getBool(PROPERTY_DEBUG_ENABLED))
			{
				RDK_INFO_STREAM(currentsensors->getStringRepresentation());
				RDK_INFO_STREAM(currentjoints->getStringRepresentation());
			}
		
		}
		
		SESSION_END_CATCH_TERMINATE(session);

	} // while

	closing();
}

void NaoQiJointsModule::closing() { 

	SESSION_TRY_START(session)

	RdkMotionData* data;
	RNaoJoints* joints = new RNaoJoints(NaoJoints::getSafePose());
	
	shmmotion->wait(RDK);
	data = static_cast<RdkMotionData*>(shmmotion->getEntry());
	for (int i = 0; i < JOINTS_VALUES_SIZE; i++) data->targetJointsValues[i] = joints->getValues()[i];
	data->executionTime = 3.0;
	data->withSpeedValue = session->getInt(PROPERTY_GOTO_WITH_SPEED);
	data->interpolationMode = INTERPOLATION_SMOOTH;
	data->shutdown = true;
	data->change_stiffness = false;
	shmmotion->signal(NAOQI);

	int sec=5;
	while (sec>=0) {
		cout << getModuleName().c_str() << " Stiffness OFF in " << sec << " sec...." <<  endl;
		usleep(1e6);
		sec--;
	}

	shmmotion->wait(RDK);
	data->change_stiffness = true;
	
	//changed by massimo (substitute with the comment)	
	data->stiffness_chain = 1; //session->getInt(PROPERTY_STIFFNESS_CHAIN);
	//	

	data->stiffness = 0.0;
	shmmotion->signal(NAOQI);

	SESSION_END_CATCH_TERMINATE(session);
}

void NaoQiJointsModule::exitRequested() { 

} 

void NaoQiJointsModule::cleanup()
{


}

MODULE_FACTORY(NaoQiJointsModule);

}} // namespace
