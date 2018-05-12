/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2014 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_RobotImporterFactory_h_
#define _VirtualRobot_RobotImporterFactory_h_

#include "../VirtualRobot.h"
#include "../VirtualRobotImportExport.h"
#include "../AbstractFactoryMethod.h"
#include "../Robot.h"
#include "../XML/RobotIO.h"

#include <Eigen/Core>


namespace VirtualRobot
{

    /*!
        A RobotImporter can be used ti import robots from various data formats.
         By default the Simox XML-based RobotImporter can be retrieved as follows:
         \code
         RobotImporterPtr imp = RobotImporterFactory::fromName("SimoxXML", NULL);
         RobotPtr robot = imp->loadFromFile(filename);
         \endcode

         Other importers (e.g. collada) have to be selected during project setup.
    */
    class RobotImporterFactory;
    typedef boost::shared_ptr<RobotImporterFactory> RobotImporterFactoryPtr;
    class VIRTUAL_ROBOT_IMPORT_EXPORT RobotImporterFactory  : public AbstractFactoryMethod<RobotImporterFactory, void*>
    {
    public:
        RobotImporterFactory()
        {
            ;
        }
        virtual ~RobotImporterFactory()
        {
            ;
        }

        virtual RobotPtr loadFromFile(const std::string& filename, RobotIO::RobotDescription loadMode = RobotIO::eFull) = 0;

        virtual std::string getFileExtension() = 0;
        virtual std::string getFileFilter() = 0;

        static std::string getAllFileFilters();
        static std::string getAllExtensions();
        static RobotImporterFactoryPtr fromFileExtension(std::string type, void* params);
    };


} // namespace VirtualRobot

#endif // _VirtualRobot_RobotImporterFactory_h_
