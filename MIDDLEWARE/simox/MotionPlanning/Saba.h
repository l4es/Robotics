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
* @package    Saba
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _Saba_h_
#define _Saba_h_


/*! \defgroup Saba The Sampling-Based Motion Planning Library
The Sampling-based Motion Planning Library Saba offers state-of-the-art algorithms for planning collision-free motions
in high-dimensional configuration spaces. Standard approaches, as the Rapidly-exploring Random Trees (RRT), are efficiently implemented.
Furthermore motion planning algorithms for grasping and for bimanual tasks are provided, which can be used to solve plannign problems for
mobile manipulators or service and humanoid robots.
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
#  if defined(Saba_EXPORTS)
#    define SABA_IMPORT_EXPORT __declspec(dllexport)
#  else
#    define SABA_IMPORT_EXPORT __declspec(dllimport)
#  endif
#else
#  define SABA_IMPORT_EXPORT
#endif


namespace Saba
{
    // only valid within the Saba namespace
    using std::cout;
    using std::endl;

    class CSpace;
    class CSpaceSampled;
    class CSpacePath;
    class CSpaceTree;
    class CSpaceNode;
    class Sampler;
    class ConfigurationConstraint;
    class Rrt;
    class MotionPlanner;
    class BiRrt;
    class GraspIkRrt;
    class GraspRrt;
    class PathProcessor;
    class ShortcutProcessor;
    class ApproachDiscretization;
    class PlanningThread;
    class PathProcessingThread;

    typedef boost::shared_ptr<CSpace> CSpacePtr;
    typedef boost::shared_ptr<CSpaceSampled> CSpaceSampledPtr;
    typedef boost::shared_ptr<CSpacePath> CSpacePathPtr;
    typedef boost::shared_ptr<Sampler> SamplerPtr;
    typedef boost::shared_ptr<CSpaceTree> CSpaceTreePtr;
    typedef boost::shared_ptr<CSpaceNode> CSpaceNodePtr;
    typedef boost::shared_ptr<MotionPlanner> MotionPlannerPtr;
    typedef boost::shared_ptr<Rrt> RrtPtr;
    typedef boost::shared_ptr<BiRrt> BiRrtPtr;
    typedef boost::shared_ptr<GraspIkRrt> GraspIkRrtPtr;
    typedef boost::shared_ptr<GraspRrt> GraspRrtPtr;
    typedef boost::shared_ptr<PathProcessor> PathProcessorPtr;
    typedef boost::shared_ptr<ShortcutProcessor> ShortcutProcessorPtr;
    typedef boost::shared_ptr<ConfigurationConstraint> ConfigurationConstraintPtr;
    typedef boost::shared_ptr<ApproachDiscretization> ApproachDiscretizationPtr;
    typedef boost::shared_ptr<PlanningThread> PlanningThreadPtr;
    typedef boost::shared_ptr<PathProcessingThread> PathProcessingThreadPtr;

#define SABA_INFO VR_INFO
#define SABA_WARNING VR_WARNING
#define SABA_ERROR VR_ERROR

#define THROW_SABA_EXCEPTION(a) THROW_VR_EXCEPTION(a)

#ifdef _DEBUG
#define SABA_ASSERT(a) if (!(a)) {cout << "ASSERT failed (" << #a <<")"<<endl; THROW_SABA_EXCEPTION( "ASSERT failed (" << #a << ")" )};
#define SABA_ASSERT_MESSAGE(a,b) if (!(a)) {cout << "ASSERT failed (" << #a <<"): "<<b<<endl; THROW_SABA_EXCEPTION( "ASSERT failed (" << #a << "): " << b )};

#else
#define SABA_ASSERT(a)
#define SABA_ASSERT_MESSAGE(a,b)
#endif

}

#endif // _Saba_h_
