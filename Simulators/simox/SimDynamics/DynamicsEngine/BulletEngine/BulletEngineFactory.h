/*!
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
#ifndef _SimDynamics_BulletEngineFactory_h_
#define _SimDynamics_BulletEngineFactory_h_

#include "../DynamicsEngineFactory.h"
#include "BulletEngine.h"

namespace SimDynamics
{

    /*!
        A bullet based implementation.
        This factory creates a BulletEngine.
        No need to use this class directly, use DynamicsWorld::Init() to automatically create the correct factory and engine.
    */
    class SIMDYNAMICS_IMPORT_EXPORT BulletEngineFactory : public DynamicsEngineFactory
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        BulletEngineFactory();
        virtual ~BulletEngineFactory();

        virtual DynamicsEnginePtr createEngine(DynamicsEngineConfigPtr config = DynamicsEngineConfigPtr());

        virtual DynamicsObjectPtr createObject(VirtualRobot::SceneObjectPtr o);
        virtual DynamicsRobotPtr createRobot(VirtualRobot::RobotPtr robot);

        // AbstractFactoryMethod
    public:
        static std::string getName();
        static boost::shared_ptr<DynamicsEngineFactory> createInstance(void*);
    private:
        static SubClassRegistry registry;
    };

} // namespace VirtualRobot

#endif // _SimDynamics_BulletEngineFactory_h_
