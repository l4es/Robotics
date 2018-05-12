#define MODULE_NAME "NaoQiSensorsModule"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE MODULE_NAME

#include "naoqisensorsmodule.h"
#include "naoqisensorsmodule_names.h"
#include <rdkcore/sharedmemory/sensordatashm.h>
#include <nao/object/rnaojoints.h>
#include <nao/object/rnaosensors.h>

#include <cstring>

namespace RDK2 { namespace RAgent {

bool NaoQiSensorsModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)

	Common::createDefaultProperties(session, true);

	session->createStorage("RNaoJoints", PROPERTY_OUT_JOINTS, "Nao joints values");
	session->setObject(PROPERTY_OUT_JOINTS, new RNaoJoints());
	session->setVolatile(PROPERTY_OUT_JOINTS);

	session->createDouble(PROPERTY_OUT_BATTERY, "Battery charge (0.0-1.0)", RDouble::REAL, -1);
	session->setVolatile(PROPERTY_OUT_BATTERY);
	
	session->createStorage("RNaoSensors", PROPERTY_OUT_SENSORS, "Nao sensor values");
	session->setObject(PROPERTY_OUT_SENSORS, new RNaoSensors());
	session->setVolatile(PROPERTY_OUT_SENSORS);
	
	session->createString(PROPERTY_OUT_ALLDATA, "All data in string format", "N/A");
	session->setVolatile(PROPERTY_OUT_ALLDATA);
	session->createDouble(PROPERTY_OUT_SENSORS_TIMESTAMP,"Sensor timestamp",RDouble::SEC, 0.0);
	session->setVolatile(PROPERTY_OUT_SENSORS_TIMESTAMP);

	session->createString(PROPERTY_OUT_POSE, "Pose (body posture) string format", "");
	session->setVolatile(PROPERTY_OUT_POSE);

	session->createDouble(PROPERTY_PARAMS_PERIOD, "Enable sensors reading every X seconds", RDouble::SEC, 0.1);

	session->createBool(PROPERTY_DEBUG_ENABLED, "Enable debug output (use RDK_LOG=trace)", false);
	session->setVolatile(PROPERTY_DEBUG_ENABLED);

	SESSION_END(session);
	return true;
	SESSION_CATCH_TERMINATE(session);
	return false;
}

bool NaoQiSensorsModule::init()
{
	SESSION_TRY_START(session)

	shmsensor = new SharedMemory(SharedMemory::SENSOR, sizeof(RdkSensorData));

	shmsensor->signal(NAOQI);

	SESSION_END(session);
	return true;
	SESSION_CATCH_TERMINATE(session);
	return false;
}

void NaoQiSensorsModule::exec()
{
	while (session->dontWait(), !exiting)
	{
		SESSION_TRY_START(session)
			usleep(session->getDouble(PROPERTY_PARAMS_PERIOD)*1e6);
			shmsensor->wait(RDK);
#ifdef OPENRDK_ARCH_GENERIC
			usleep(100*10e6);
#else
			
			RdkSensorData *sensors = static_cast<RdkSensorData*>(shmsensor->getEntry());
			session->lock(PROPERTY_OUT_JOINTS, HERE);
			RNaoJoints* currentjoints = session->getObjectAsL<RNaoJoints>(PROPERTY_OUT_JOINTS);
			vector<float>& joints = currentjoints->getValues();
			for (size_t i = 0; i < joints.size(); i++)
				joints[i] = sensors->jointsValues[i];
			session->unlock(PROPERTY_OUT_JOINTS);
	
			session->setDouble(PROPERTY_OUT_BATTERY,sensors->battery.charge);

			session->lock(PROPERTY_OUT_SENSORS, HERE);
			RNaoSensors* currentsensors = session->getObjectAsL<RNaoSensors>(PROPERTY_OUT_SENSORS);
			currentsensors->sensors = *sensors;
			session->unlock(PROPERTY_OUT_SENSORS);
			session->valueChanged(PROPERTY_OUT_SENSORS);

			if (session->getBool(PROPERTY_DEBUG_ENABLED))
			{
				ostringstream ss;
				ss << *currentsensors << endl;
				session->setString(PROPERTY_OUT_ALLDATA,ss.str());
			
				ostringstream ss2;
				for (size_t k=0; k<JOINTS_VALUES_SIZE; k++) { 
					ss2 << currentsensors->sensors.jointsValues[k] << " "; 
				}
				session->setString(PROPERTY_OUT_POSE,ss2.str());

				RDK_TRACE_STREAM(currentsensors->getStringRepresentation());
				RDK_TRACE_STREAM(currentjoints->getStringRepresentation());
			}
#endif
			shmsensor->signal(NAOQI);

		SESSION_END_CATCH_TERMINATE(session);
	}
}

void NaoQiSensorsModule::exitRequested()
{
	if (shmsensor == 0)
		return;
	shmsensor->signal(NAOQI);
}

//void NaoQiSensorsModule::cleanup() { }

MODULE_FACTORY(NaoQiSensorsModule);

}} // namespace
