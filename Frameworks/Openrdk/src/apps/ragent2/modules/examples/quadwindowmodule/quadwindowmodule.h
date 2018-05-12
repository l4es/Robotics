/*
 *  quadwindowmodule.h
 *  
 *
 *  Created by:
 *  Gennari Cristiano
 *  Paparo Valentina
 *  Sbandi Angelo
 *
 *  Copyright 2009 RoCoCo LAB. All rights reserved.
 *
 */

#ifndef RDK_MODULE_HOOVERINGMODULE
#define RDK_MODULE_HOOVERINGMODULE

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of QuadWindowModule here.
 *
 * Please write a description of QuadWindowModule here.
 *
 * @ingroup RAgentModules
 */

class QuadWindowModule : public Module {
public:
	QuadWindowModule() { }
	virtual ~QuadWindowModule() { }

	bool initConfigurationProperties();
	// bool initInterfaceProperties();
	bool init();
	void exec();
	// void exitRequested();
	// void cleanup();
	// void asyncAgentCmd(cstr cmd);
};

}} // namespace

#endif
