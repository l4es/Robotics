/**
 * @file
 * @brief This file contains the declaration of StringWriterModule
 */

#ifndef RDK2_MODULE_STRINGWRITERMODULE
#define RDK2_MODULE_STRINGWRITERMODULE

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief This is an example module, it will print an output message.
 *
 * StringWriterModule is a module that prints an output message on the console. The message string is taken from the configuration.
 *
 * @ingroup RAgentModules
 */

class StringWriterModule : public Module {
public:
	StringWriterModule() { }
	virtual ~StringWriterModule() { }

	bool initConfigurationProperties();
	bool init();
	void exec();
};

}} // namespace

#endif
