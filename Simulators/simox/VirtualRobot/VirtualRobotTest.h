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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010,2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_Test_h_
#define _VirtualRobot_Test_h_

#ifndef BOOST_TEST_DYN_LINK
#define BOOST_TEST_DYN_LINK
#endif

#include <boost/test/unit_test.hpp>

#include "VirtualRobotImportExport.h"
#include <string>
#include <fstream>

struct OutputConfiguration
{
    OutputConfiguration()
    {
        std::string logFileName;
#ifdef Simox_TEST_DIR
        logFileName = std::string(Simox_TEST_DIR);
#endif
        logFileName.append(boost::unit_test::framework::master_test_suite().p_name);
        logFileName.append(".xml");
        logFile.open(logFileName.c_str());
        boost::unit_test::unit_test_log.set_stream(logFile);
    }
    ~OutputConfiguration()
    {
        logFile << "</TestLog>" << std::flush;
        logFile.close();
        boost::unit_test::unit_test_log.set_stream(std::cout);
    }
    std::ofstream logFile;
};

BOOST_GLOBAL_FIXTURE(OutputConfiguration);

#endif /* _VirtualRobot_Test_h_ */
