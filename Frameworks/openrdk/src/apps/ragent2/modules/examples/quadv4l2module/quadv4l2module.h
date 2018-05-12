/**
 * @file
 * @brief This file contains the declaration of QuadV4L2Module
 */

#ifndef RDK_MODULE_TEMPLATEMODULE
#define RDK_MODULE_TEMPLATEMODULE

#include <rdkcore/modules/module.h>

#include "V4L2CameraDevice.h"

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of QuadV4L2Module here.
 *
 * Please write a description of QuadV4L2Module here.
 *
 * @ingroup RAgentModules
 */

class QuadV4L2Module : public Module {
private:
	//Image rgb; 
	V4L2CameraDevice *camera;
	bool noimage;

public:
	QuadV4L2Module() { }
	virtual ~QuadV4L2Module() { }

	bool initConfigurationProperties();
	// bool initInterfaceProperties();
	bool init();
	void exec();
	// void exitRequested();
	void cleanup();
	// void asyncAgentCmd(cstr cmd);
};

}} // namespace

#endif
