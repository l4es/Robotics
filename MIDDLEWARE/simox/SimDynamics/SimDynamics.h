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
* @package    SimDynamics
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _SimDynamics_h_
#define _SimDynamics_h_


/*! \defgroup SimDynamics The Dynamic Simulation Library
This library can be used to simulate physical interactions and dynamic systems.

*/

#ifdef WIN32

// needed to have M_PI etc defined
#if !defined(_USE_MATH_DEFINES)
#define _USE_MATH_DEFINES
#endif

// eigen wants this on windows
#if !defined(NOMINMAX)
#define NOMINMAX
#endif

#endif

#include "VirtualRobot/VirtualRobot.h"
#include "VirtualRobot/VirtualRobotException.h"


#include <iostream>
#include <sstream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef WIN32
#  include <winsock2.h>
#  include <windows.h>
#  pragma warning ( disable : 4251 )
#  if defined(SimDynamics_EXPORTS)
#    define SIMDYNAMICS_IMPORT_EXPORT __declspec(dllexport)
#  else
#    define SIMDYNAMICS_IMPORT_EXPORT __declspec(dllimport)
#  endif
#else
#  define SIMDYNAMICS_IMPORT_EXPORT
#endif


namespace SimDynamics
{
    // only valid within the SimDynamics namespace
    using std::cout;
    using std::endl;

    class DynamicsWorld;

    typedef boost::shared_ptr<DynamicsWorld> DynamicsWorldPtr;


#define SIMDYNAMICS_INFO VR_INFO
#define SIMDYNAMICS_WARNING VR_WARNING
#define SIMDYNAMICS_ERROR VR_ERROR

#define THROW_SIMDYNAMICS_EXCEPTION(a) THROW_VR_EXCEPTION(a)

#ifdef _DEBUG
#define SIMDYNAMICS_ASSERT(a) if (!(a)) {std::cout << "ASSERT failed (" << #a <<")"<<std::endl; THROW_SIMDYNAMICS_EXCEPTION( "ASSERT failed (" << #a << ")" )};
#define SIMDYNAMICS_ASSERT_MESSAGE(a,b) if (!(a)) {std::cout << "ASSERT failed (" << #a <<"): "<<b<<std::endl; THROW_SIMDYNAMICS_EXCEPTION( "ASSERT failed (" << #a << "): " << b )};

#else
#define SIMDYNAMICS_ASSERT(a)
#define SIMDYNAMICS_ASSERT_MESSAGE(a,b)
#endif

}

#endif // _SimDynamics_h_
