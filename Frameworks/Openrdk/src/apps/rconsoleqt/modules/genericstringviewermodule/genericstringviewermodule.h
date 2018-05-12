#ifndef RDK2_RQM_GENERICSTRINGVIEWERMODULE
#define RDK2_RQM_GENERICSTRINGVIEWERMODULE

#include "../../rqcommon/simplepropertyviewermodule.h"

#include <qlineedit.h>
#include <qlabel.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/rprimitive/rstring.h>

#define LOGGING_MODULE "GenericStringViewerModule"

namespace RDK2 { namespace RConsoleQt {
		
class GenericStringViewerModule : public SimplePropertyViewerModule {
public: 
	GenericStringViewerModule() : prototype(NULL) {}
	virtual ~GenericStringViewerModule() {
		delete prototype;
	}

protected:

	RDK2::Object* prototype;

	virtual QWidget* createWidget();
	virtual RDK2::Object* buildRObject();
	virtual void refreshWidgetFields();	
	QLineEdit* lineEdit;
};

}} // ns

#endif
