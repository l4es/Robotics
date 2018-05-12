#define MODULE_NAME "TcpInterfaceModule"

#include "tcpinterfacemodule.h"
#include "tcpinterfacemodule_names.h"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/ns.h>
#include <rdkcore/simplethreads/simplethreads.h>

#include <rdkcore/serialization_binary/binarywriter.h>

#include <rdkcore/rmaps/ritemonmapvector.h>
#include <rdkcore/rmaps/rpoint2donmap.h>
#include <rdkcore/rmaps/rellipseonmap.h>
#include <rdkcore/rmaps/rline2donmap.h>
#include <rdkcore/rgraphics/color.h>
#include <rdkcore/rsensordata/rpercept.h>
#include <errno.h>

#define LOGGING_MODULE MODULE_NAME

#ifdef MagickPP_FOUND
#include <Magick++.h>
#endif

#include <cstdio>
#include <rdkcore/rmaps/rarrowonmap.h>
#include <string.h>
// #include <../../OpenRDK-SPQR/src/ra-modules/rescue/maploc/distancemapmodule/astar.h>
// #include <../../OpenRDK-SPQR/src/ra-modules/rescue/maploc/elevationmappermodule/colorconverision.h>

namespace RDK2 { namespace RAgent { 
  using namespace std;
  using RDK2::RSensorData::RPercept;

bool TcpInterfaceModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		// put false in the second parameter if you want the module disabled for default
		// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, true);
	
		session->createBool(PROPERTY_CUSTOM, "bool", false);
		session->createInt(PROPERTY_PORT, "TCP listening port", 9876);

	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}


bool TcpInterfaceModule::init()
{
	// in the init() function you should initialize everything your module needs (drivers initialization,
	// sockets, and so on); all modules wait for all init() before they start
	SESSION_TRY_START(session)
		// here you can declare the events you are waiting in the main exec() loop,
		// for example:
		tcpServerSocket = new TCPServerSocket(session->getInt(PROPERTY_PORT));
		
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void TcpInterfaceModule::exec()
{
	while (session->dontWait(), !exiting) {
		SESSION_TRY_START(session)
				
				
				RDK_INFO_PRINTF("Waiting for incoming connections on port %d",session->getInt(PROPERTY_PORT));
				try { tcpSocket = tcpServerSocket->accept();}
				catch (const NetException& e) { if (!exiting) {RDK_ERROR_PRINTF(e.fullwhat()); } }
				
				  
				if (!exiting && tcpSocket) {
					
					InetAddress peerAddress = tcpSocket->getPeerAddress();
					RDK_INFO_PRINTF("Accepted a connection from %s:%d", peerAddress.getIPAddress().c_str(), peerAddress.getPort());
					Session* dhsthread =
					    session->getRepository()->createSession("TCP manager", "TCP manager", session->getUrlContext());
				        DhsThread* peerManager = new DhsThread(tcpSocket, dhsthread);
					peerManagers.push_back(peerManager);
					
					peerManager->start();
					
					for (list<DhsThread*>::iterator it = peerManagers.begin(); it != peerManagers.end(); ) {
					  if (!(*it)->connected) {
						(*it)->stop();
						delete *it;
						it = peerManagers.erase(it);
					}
					
					else ++it;
					}
			
				}
			
			
		SESSION_END_CATCH_TERMINATE(session)
	}
}
	
void TcpInterfaceModule::exitRequested()
{	
	if (tcpServerSocket) tcpServerSocket->unbind();
}


void TcpInterfaceModule::cleanup()
{

	for (list<DhsThread*>::iterator it = peerManagers.begin(); it != peerManagers.end(); ++it) {
	
	  if (!(*it)->connected)
	  {	(*it)->stop();
		delete *it;
		it = peerManagers.erase(it);
	  }
		
		
	}
	
	if (tcpServerSocket)
		delete tcpServerSocket;
	tcpServerSocket = 0;
}

DhsThread::DhsThread(Network::TCPSocket* tcpSocket, Session* session) : connected(true), tcpSocket(tcpSocket), session(session){
	//processCmd("help");
}

  void DhsThread::exec()
{
  
	connected=true;
	while (!exiting) {
		
 			string cmd;
 			getline(*tcpSocket, cmd);
			
			if (!exiting)
			{
				if (cmd != "") {
					while (cmd.size() > 0 && (cmd[cmd.size()-1] == 10 || cmd[cmd.size()-1] == 13)) {
						cmd = cmd.substr(0, cmd.size()-1);
					}
					//DEBUG PURPOSES
					RDK_INFO_PRINTF("Received remote command: %s", cmd.c_str());
					processCmd(cmd);
					
				}
				else
				{  
				   if (tcpSocket)
					   tcpSocket = 0;
					
				}
					
				if (!tcpSocket)
				{
				    RDK_INFO_PRINTF("Connection closed");
				    connected = false;
				    delete tcpSocket;
				  return;
				}
			}
				
		}
}


void DhsThread::processCmd(cstr cmd)
{
  SESSION_TRY_START(session)
  
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
			RDK_INFO_PRINTF("Processing the setValue command: %s", cmd.c_str());
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
		// Added by Mirko Malacario.
		else if (cmdline[0] == "getVectorRItemOnMap")
		{
		  if (cmdline.size() != 3) 
		  {
			*tcpSocket << "ERROR: wrong number of parameters, usage: getVector <propertyName> <format, i.e. vector, url, spqr>"<< endl;
				return;
		  }
		  if (cmdline[2] == "rdk") 
		  {
			*tcpSocket << "Should be implemented?"<<endl;
		  }
		  
		   else if (cmdline[2] == "spqr") 
		   {
			BinaryWriter bw(true);
			session->lock(cmdline[1], HERE);
			
			RDK2::RMaps::RItemOnMapVector* vector_item = 0; 
			vector_item = session->getClonedObjectAs<RDK2::RMaps::RItemOnMapVector>(cmdline[1]);
			
			string s = bw.serialize(true, vector_item);
			bool error = false;
			ostringstream os;
			
			session->unlock(cmdline[1]);
			
			RDK2::RMaps::RItemOnMapVector::const_iterator it;
			for (it=vector_item->begin(); it!=vector_item->end(); ++it)
			{
			  RDK2::RMaps::RItemOnMap* r = *it;
			  RDK2::RMaps::RPoint2dOnMap* point2d = dynamic_cast < RDK2::RMaps::RPoint2dOnMap*>(r);
			  RDK2::RMaps::REllipseOnMap* ellipse = dynamic_cast < RDK2::RMaps::REllipseOnMap*>(r);
			  RDK2::RMaps::RLine2dOnMap* line2d = dynamic_cast < RDK2::RMaps::RLine2dOnMap*>(r);
			  RDK2::RMaps::RArrowOnMap* arrow = dynamic_cast < RDK2::RMaps::RArrowOnMap*>(r);
			  
			  if(point2d)
			  {
			    os << s << "RPoint2dOnMap$" << point2d->x << " " << point2d->y << " " << RDK2::RGraphics::getColorName(point2d->color)
			       << " " << point2d->drawWidth << ";";
			    s = os.str();
			    
			  }
			  else if(ellipse)
			  {
			    os << s <<"REllipseOnMap$" << ellipse->x << " " << ellipse->y << " " << ellipse->theta << " " << ellipse->a << " " << ellipse->b 
			       << " " << ellipse->filled << " " << RDK2::RGraphics::getColorName(ellipse->color) << " " << ellipse->drawWidth << ";";
			       s = os.str();
			       
			  }
			  else if(line2d)
			  {
			    os << s <<"RLine2dOnMap$" << line2d->p0.x << " " << line2d->p0.y << " " << line2d->p1.x << " " << line2d->p1.y << " " 
			       << RDK2::RGraphics::getColorName(line2d->color) << " " << line2d->drawWidth << ";";
			       s = os.str();
			       
			  }
			  else if(arrow)
			  {
			    os << "RArrowOnMap$" << arrow->x << " " << arrow->y << " " << arrow->theta << " " << arrow->filled << " "
				   << RDK2::RGraphics::getColorName(arrow->color) << " " << arrow->drawWidth << ";";
			    
				s = os.str();
			  }
			  else
			  {
			    error = true;
			  }
			  
			}
			
			ostringstream temp;
			
			temp << s << endl;
			
			s = temp.str();
			
			if (error)
			{
			  *tcpSocket << "ERROR: some object in the vector is not valid" << endl;
			}
			else *tcpSocket << s;
			
			tcpSocket->flush();
		  }
		}
		// Added by Fabio Previtali.
		else if (cmdline[0] == "getRPercept")
		{
			if (cmdline.size() != 2)
			{
				*tcpSocket << "ERROR: wrong number of parameters, usage: getRPercept <propertyName>"<< endl;
				
				return;
			}
			
			session->lock(cmdline[1], HERE);
			
			std::vector<const RDK2::RSensorData::RPercept*> vectorRPercept = session->queueFreezeAs<const RDK2::RSensorData::RPercept>(cmdline[1]);
			
			ostringstream os;
			
			for (std::vector<const RDK2::RSensorData::RPercept*>::iterator it = vectorRPercept.begin(); it != vectorRPercept.end(); it++)
			{
				os << "RPercept$" << " " << (*it)->rho << " " << (*it)->theta << " " << (*it)->logtag << ";";
			}
			
			*tcpSocket << os.str() << endl;
			
			session->unlock(cmdline[1]);
		}
		// Added by Martina Deturres.
		else if(cmdline[0] == "setQueue")
		{
			if(cmdline.size()!=3) 
				*tcpSocket << "ERROR: wrong number of parameters, usage: setQueue <propertyName> <string>"<< endl;
			
			session->lock(cmdline[1], HERE);
			
			vector<string> vectorReadings = tokenize(cmdline[2], ";"); //vector of items like "RPercept$rho,theta,logtag"
			
			for (vector<string>::const_iterator it = vectorReadings.begin(); it != vectorReadings.end(); it++)
			{
				vector<string> reading = tokenize(*it, "$"); //i.e "RPercept" and "rho,theta,logtag"
				
				if (reading.at(0) == "RPercept")
				{
					vector<string> valueReading = tokenize(reading.at(1), ",");
					
					RPercept* simulatedReading = new RPercept();
					
					simulatedReading->rho = atof(valueReading.at(0).c_str());
					simulatedReading->theta = atof(valueReading.at(1).c_str());
					simulatedReading->logtag = valueReading.at(2).c_str();
					
					session->queuePush(cmdline[1],simulatedReading);
				}
				else RDK_WARNING_STREAM("Warning: Reading type not recognized!");
				
			}
			session->unlock(cmdline[1]);
			
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
 SESSION_END_CATCH_TERMINATE(session)
}	


void DhsThread::exitRequested()
{
  if (tcpSocket)
  {tcpSocket->disconnect();}

}

DhsThread::~DhsThread()
{
    delete tcpSocket;
}

MODULE_FACTORY(TcpInterfaceModule);

}} // namespace
