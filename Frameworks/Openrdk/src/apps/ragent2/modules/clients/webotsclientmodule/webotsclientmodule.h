#ifndef RDK2_MODULE_WEBOTSCLIENTMODULE
#define RDK2_MODULE_WEBOTSCLIENTMODULE

#include <rdkcore/modules/module.h>

#include "webotsconnection.h"

namespace RDK2 { namespace RAgent {

class WebotsClientModule : public RDK2::RAgent::Module {
public:
	WebotsClientModule() { }
	virtual ~WebotsClientModule() { }

	bool initConfigurationProperties();
	bool init();
	void exec();
	void exitRequested();

private:
	int playerNumber;

	void doExec();

	WebotsData wdata;
	WebotsConnection wc;
};

}} // namespace

#endif
