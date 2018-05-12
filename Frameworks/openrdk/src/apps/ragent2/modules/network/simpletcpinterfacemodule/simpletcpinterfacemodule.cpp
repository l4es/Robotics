#define MODULE_NAME "SimpleTcpInterfaceModule"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/serialization_binary/binarywriter.h>

#include <rdkcore/rmaps/ritemonmapvector.h>
#include <rdkcore/rmaps/rpoint2donmap.h>
#include <rdkcore/rmaps/rellipseonmap.h>
#include <rdkcore/rmaps/rline2donmap.h>
#include <rdkcore/rgraphics/color.h>

#define LOGGING_MODULE MODULE_NAME

#ifdef MagickPP_FOUND
#include <Magick++.h>
#endif

#include "simpletcpinterfacemodule.h"

#define PROPERTY_PORT "params/listeningPort"

namespace RDK2 { namespace RAgent {

	using namespace RDK2::Serialization::Binary;

	bool SimpleTcpInterfaceModule::initConfigurationProperties()
	{
		SESSION_TRY_START(session)
			// put false in the second parameter if you want the module disabled for default
			// add a third parameter valued false if you don't want the state property
			Common::createDefaultProperties(session, true);
		session->createInt(PROPERTY_PORT, "TCP listening port", 9876);
		session->setNotEditable(PROPERTY_PORT);
		SESSION_END(session)
			return true;
		SESSION_CATCH_TERMINATE(session)
			return false;
	}

	bool SimpleTcpInterfaceModule::init()
	{
		SESSION_TRY_START(session)
			// here you can declare the events you are waiting in the main exec() loop,
			// for example:
			//session->listenToTimer(500.);
			tcpServerSocket = new TCPServerSocket(session->getInt(PROPERTY_PORT));
		RDK_INFO_PRINTF("SimpleTcpInterface bound to port %d", session->getInt(PROPERTY_PORT));
		SESSION_END(session)
			return true;
		SESSION_CATCH_TERMINATE(session)
			return false;
	}

	void SimpleTcpInterfaceModule::exec()
	{
		while (session->dontWait(), !exiting) {
			SESSION_TRY_START(session)
				if (tcpSocket == 0) {
					RDK_INFO_PRINTF("Waiting for incoming connections on port %d", session->getInt(PROPERTY_PORT));
					try { tcpSocket = tcpServerSocket->accept(); }
					catch (const NetException& e) { if (!exiting) {RDK_ERROR_PRINTF(e.fullwhat()); } }
					if (!exiting && tcpSocket) {
						InetAddress peerAddress = tcpSocket->getPeerAddress();
						RDK_INFO_PRINTF("Accepted a connection from %s:%d", peerAddress.getIPAddress().c_str(), peerAddress.getPort());
						processCmd("help", tcpSocket);
					}
				}

			if (!exiting) {
				string cmd;
				getline(*tcpSocket, cmd);
				if (!exiting) {
					if (cmd != "") {
						while (cmd.size() > 0 && (cmd[cmd.size()-1] == 10 || cmd[cmd.size()-1] == 13)) {
							cmd = cmd.substr(0, cmd.size()-1);
						}
						//RDK_DEBUG_PRINTF("Received command: %s", cmd.c_str());
						processCmd(cmd, tcpSocket);
					}
					else {
						delete tcpSocket;
						tcpSocket = 0;
					}
				}
			}
			SESSION_END_CATCH_TERMINATE(session)
		}
	}

	void SimpleTcpInterfaceModule::processCmd(cstr cmd, TCPSocket* tcpSocket)
	{
		if (cmd == "") return;
		vector<string> cmdline = tokenize(cmd, " ");
		try {
			if (cmdline[0] == "help") {
				if (cmdline.size() != 1) {
					*tcpSocket << "ERROR: wrong number of parameters, usage: help" << endl;
					return;
				}
				*tcpSocket << "LINES: 8" << endl;
				*tcpSocket << "getValue <propertyUrl>          - gets the value of the property" << endl;
				*tcpSocket << "setValue <propertyUrl> <value>  - sets the value of the property" << endl;
				*tcpSocket << "propertyList [<prefix>]         - retrieve all property names [starting with <prefix>]" << endl;
				*tcpSocket << "ls [<prefix>]                   - shows the properties in the 'current directory/<prefix>'" << endl;
				*tcpSocket << "cd [<prefix>]                   - change the 'current directory' to <prefix>" << endl;
				*tcpSocket << "getImage <propertyUrl> <format> - retrieve an image (<format> can be 'jpeg'/'jpg', 'rdk')" << endl;
				*tcpSocket << "asyncCmd <command> [<params>]   - sends an async command" << endl;
				*tcpSocket << "help                            - show this help" << endl;
			}
			else if (cmdline[0] == "getValue") {
				if (cmdline.size() != 2) {
					*tcpSocket << "ERROR: wrong number of parameters, usage: getValue <propertyUrl>" << endl;
					return;
				}
			}
			else if (cmdline[0] == "setValue") {
				if (cmdline.size() < 3) {
					*tcpSocket << "ERROR: wrong number of parameters, usage: setValue <propertyUrl> <value>" << endl;
					return;
				}
				// builds the <value> part putting together the cmdline[2+] strings
				string pvalue;
				for (size_t i = 2; i < cmdline.size(); i++) pvalue += cmdline[i] + " ";
				pvalue = pvalue.substr(0, pvalue.size()-1);
				session->getRepository()->setPropertyValueFromTextConfig(cmdline[1], pvalue, "/", session);
				session->valueChanged(cmdline[1]);
			}
			else if (cmdline[0] == "propertyList") {
				string prefix = "";
				if (cmdline.size() > 1) prefix = cmdline[1];
				vector<Url> pnames = session->getPropertyNamesStartingWith(prefix);
				*tcpSocket << "LINES: " << (pnames.size() + 1) << endl;
				if (prefix == "") *tcpSocket << "All properties:" << endl;
				else *tcpSocket << "Properties with prefix '" << prefix << "': " << endl;
				for (size_t i = 0; i < pnames.size(); i++) {
					*tcpSocket << pnames[i] << endl;
				}
			}
			else if (cmdline[0] == "getImage" || cmdline[0] == "getMap") {
				if (cmdline.size() != 3) {
					*tcpSocket << "ERROR: wrong number of parameters, usage: getImage <propertyName> <format, i.e. jpeg, jpg, rdk>"
						<< endl;
					return;
				}
				if (cmdline[2] == "rdk") {
					BinaryWriter bw(true);
					session->lock(cmdline[1], HERE);
					RImage* img = 0;
					if (cmdline[0] == "getImage") img = session->getObjectAsL<RImage>(cmdline[1]);
					else img = session->getObjectAsL<RMapImage>(cmdline[1])->image;
					string s = bw.serialize(true, img);
					session->unlock(cmdline[1]);
					// FIXME anche map
					*tcpSocket << "IMAGE-VALUE-OF: " << cmdline[1] << " " << cmdline[2] << " " << s.size() << endl;
					*tcpSocket << s; tcpSocket->flush();
				}
				else if (cmdline[2] == "jpeg" || cmdline[2] == "jpg") {
#ifdef MagickPP_FOUND
					session->lock(cmdline[1], HERE);
					RMapImage* mapimage = 0;
					RImage* img = 0;
					if (cmdline[0] == "getImage") img = session->getObjectAsL<RImage>(cmdline[1]);
					else {
						mapimage = session->getObjectAsL<RMapImage>(cmdline[1]);
						img = mapimage->image;
					}
					RImage* img32 = img->convertTo(RImage::RGB32);
					string formt = (cmdline[0] == "getImage" ? "RGBA" : "BGRA");
					Magick::Image image(img->getWidth(), img->getHeight(), formt, Magick::CharPixel, img32->getBuffer());
					session->unlock(cmdline[1]);
					Magick::Blob blob;
					image.write(&blob, "jpeg");
					if (cmdline[0] == "getImage") {
						*tcpSocket << "IMAGE-VALUE-OF: " << cmdline[1] << " " << cmdline[2] << " " << blob.length() << endl;
					}
					else {
						*tcpSocket << "MAP-VALUE-OF: " << cmdline[1] << " " << cmdline[2] << " "
							<< mapimage->x << " " << mapimage->y << " " << (img->getWidth() / mapimage->getRealWidth()) << " "
							<< blob.length() << endl;
					}
					tcpSocket->write((const char*) blob.data(), blob.length());
					tcpSocket->flush();
					//RDK_DEBUG_PRINTF("Flushed %d bytes", blob.length());
					/*ofstream ofs("temp.jpg");
						ofs.write((const char*) blob.data(), blob.length());
						ofs.close();*/
#else
					RDK_ERROR_PRINTF("You need ImageMagick support to send jpeg compressed images, "
							"please check your build configuration (and src/manual.cmake)");
					*tcpSocket << "ERROR: no ImageMagick support in the server, cannot compress images" << endl;
#endif
				}
			}
			else if (cmdline[0] == "asyncCmd") {
				if (cmdline.size() < 2) {
					*tcpSocket << "ERROR: wrong number of parameters, usage: asyncCmd <command> [<parameters>]" << endl;
					return;
				}
				else {
					string s;
					for (size_t i = 1; i < cmdline.size(); i++) s += cmdline[i] + " ";
					s = s.substr(0, s.size()-1);
					*tcpSocket << "ERROR: async command not yet implemented" << endl;
				}
			}

			//Added by Mirko Malacario
			else if (cmdline[0] == "getVectorRItemOnMap")
			{
				if (cmdline.size() != 2) 
				{
					*tcpSocket << "ERROR: wrong number of parameters, usage: getVectorRItemOnMap <propertyName>"<< endl;
					return;
				}

				BinaryWriter bw(true);
				session->lock(cmdline[1], HERE);

				RDK2::RMaps::RItemOnMapVector* vector_item = 0; 
				vector_item = session->getObjectAsL<RDK2::RMaps::RItemOnMapVector>(cmdline[1]);

				//string s = "";//bw.serialize(true, vector_item);
				bool error = false;
				ostringstream os;

				RDK2::RMaps::RItemOnMapVector::const_iterator it;
				for (it=vector_item->begin(); it!=vector_item->end(); ++it)
				{
					if (vector_item->hasStringRepresentation())
					{
						os << vector_item->getClassName()            << "$ "
						   << vector_item->getStringRepresentation() << " ; ";
						continue;
					}
					error = true;
				}
				session->unlock(cmdline[1]);

				if (error)
				{
					*tcpSocket << "ERROR: some object in the vector is not valid" << endl;
				}
				else *tcpSocket << os.str() << endl;

				tcpSocket->flush();
			}
			else {
				*tcpSocket << "ERROR: unknown command '" << cmdline[0] << "'" << endl;
			}

			if (cmdline[0] == "getValue" || cmdline[0] == "setValue") {
				RDK2::Object* obj = session->getObjectClone(cmdline[1]);
				if (obj) *tcpSocket << "VALUE-OF: " << cmdline[1] << " " << obj->getStringForVisualization() << endl;
				else *tcpSocket << "ERROR: cannot retrieve value of " << cmdline[1] << endl;
				delete obj;
			}
		}
		catch (const SessionException& e) {
			*tcpSocket << "ERROR: " << e.what() << endl;
		}
	}

	void SimpleTcpInterfaceModule::exitRequested()
	{
		if (tcpServerSocket) tcpServerSocket->unbind();
		if (tcpSocket) tcpSocket->disconnect();
	}

	void SimpleTcpInterfaceModule::cleanup()
	{
		if (tcpSocket) 
			delete tcpSocket;
		tcpSocket = 0;
		if (tcpServerSocket)
			delete tcpServerSocket;
		tcpServerSocket = 0;
	}

	MODULE_FACTORY(SimpleTcpInterfaceModule);

}} // namespace
