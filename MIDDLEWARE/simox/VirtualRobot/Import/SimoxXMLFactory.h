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
#ifndef _VirtualRobot_SimoxXMLFactory_h_
#define _VirtualRobot_SimoxXMLFactory_h_

#include "../VirtualRobotImportExport.h"
#include "../Robot.h"
#include "RobotImporterFactory.h"




namespace VirtualRobot
{
    class Robot;

    class VIRTUAL_ROBOT_IMPORT_EXPORT SimoxXMLFactory  : public RobotImporterFactory
    {
    public:
        SimoxXMLFactory();
        virtual ~SimoxXMLFactory();

        virtual RobotPtr loadFromFile(const std::string& filename, RobotIO::RobotDescription loadMode = RobotIO::eFull);

        // AbstractFactoryMethod
    public:
        static std::string getName();
        static boost::shared_ptr<RobotImporterFactory> createInstance(void*);
    private:
        static SubClassRegistry registry;


        // RobotImporterFactory interface
    public:
        virtual std::string getFileFilter();
        virtual std::string getFileExtension();
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_SimoxXMLFactory_h_
