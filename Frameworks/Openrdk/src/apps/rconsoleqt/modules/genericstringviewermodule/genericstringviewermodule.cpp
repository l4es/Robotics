#include "genericstringviewermodule.h"

namespace RDK2 { namespace RConsoleQt {

	QWidget* GenericStringViewerModule::createWidget() {
		string myUrl = "<error>";
		SESSION_TRY_START(session)
			myUrl = session->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
		SESSION_END_CATCH_TERMINATE(session)
		
		QWidget* widget = new QWidget();
		
		QVBoxLayout* mainLayout = new QVBoxLayout(widget);
		QLabel* label = new QLabel(SMALL_URL_CAPTION(myUrl.c_str()), widget);
		QWidget* editPanel = new QWidget(widget);
		editPanel->setPaletteForegroundColor(Qt::red);
		mainLayout->addWidget(label);
		mainLayout->addWidget(editPanel);
		
		QHBoxLayout* editLayout = new QHBoxLayout(mainLayout);
		editLayout->addWidget(lineEdit = new QLineEdit(widget));
		
		editWidgets.push_back(lineEdit);
		
		return widget;
	}

	RDK2::Object* GenericStringViewerModule::buildRObject() {
		RDK2::Object* obj = prototype->clone();
		if (obj->loadFromStringRepresentation(lineEdit->text().latin1())) {
			return obj;
		}
		else {
			delete obj;
			return 0;
		}
	}

	void GenericStringViewerModule::refreshWidgetFields() {
		QT_THREAD_GUARD()
		Session* guiSession = RqCommon::getGuiSession(getModuleName());
		string value;
		SESSION_TRY_START(guiSession)
			string url = guiSession->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
			guiSession->lock(url, HERE);
			RDK2::Object* objVal = guiSession->getObjectL(url);
			if (prototype == NULL)
				prototype = objVal->clone();
			value = (objVal ? objVal->getStringRepresentation() : "<unset>");
			if (objVal) received = true;
			else received = false;
			guiSession->unlock(url);
		SESSION_END_CATCH_TERMINATE(guiSession)
		
		lineEdit->setText(QString(value));
	}

	MODULE_FACTORY(GenericStringViewerModule);
}}
