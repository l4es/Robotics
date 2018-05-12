/**
 * @file
 * @brief This file contains the declaration of V4LModule
 */

#ifndef RDK_MODULE_V4LMODULE
#define RDK_MODULE_V4LMODULE

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of V4LModule here.
 *
 * Please write a description of V4LModule here.
 *
 * @ingroup RAgentModules
 */

class V4LModule : public Module {
private:
	int fd;
public:
	V4LModule() : fd(-1) { }
	virtual ~V4LModule() { }

	bool initConfigurationProperties();
	bool init();
	void exec();
	void cleanup();
};

}} // namespace

#endif
