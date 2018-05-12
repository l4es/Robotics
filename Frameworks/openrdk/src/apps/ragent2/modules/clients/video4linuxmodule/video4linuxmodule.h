/**
 * @file
 * @brief This file contains the declaration of Video4LinuxModule
 */

#ifndef RDK_MODULE_VIDEO4LINUXMODULE
#define RDK_MODULE_VIDEO4LINUXMODULE

#include <rdkcore/modules/module.h>

#include "V4L2CameraDevice.h"

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of Video4LinuxModule here.
 *
 * Please write a description of Video4LinuxModule here.
 *
 * @ingroup RAgentModules
 */

class Video4LinuxModule : public Module {
private:
	V4L2CameraDevice *camera;
	int fd;
	bool noimage;

public:
	Video4LinuxModule() : camera(0), fd(-1), noimage(true) { }
	virtual ~Video4LinuxModule() { }

	bool initConfigurationProperties();
	bool init();
	void exec();
	void cleanup();
};

}} // namespace

#endif
