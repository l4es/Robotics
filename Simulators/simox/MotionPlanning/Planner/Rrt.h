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
#ifndef _Saba_Rrt_h
#define _Saba_Rrt_h

#include "../Saba.h"
#include "../CSpace/CSpaceSampled.h"
#include "../CSpace/CSpacePath.h"
#include "MotionPlanner.h"

namespace Saba
{
    /*!
     *
     * \brief A simple sampling based, single rrt planner using the extend or connect method.
     *
     * Rrt-related algorithms are known to allow efficient motion planning in high dimensional
     * configuration spaces. Complex setups can be specified via the used c-space classes.
     * The number of degrees of freedom (DoF/dimension) is defined via the c-space. Also
     * the collision detection is handled by the c-space classes.
     *
     * @see CSpaceSampled
     * @see CSpacePath
     * @see CSpaceTree
     *
     */
    class SABA_IMPORT_EXPORT Rrt : public MotionPlanner
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        enum RrtMethod
        {
            eExtend,                            // extend
            eConnect,                           // extend path until collision is determined
            eConnectCompletePath                // all or nothing: connect only when complete path is valid
        };

        enum ExtensionResult
        {
            eError,         // an error occurred
            eFailed,        // the extend failed (e.g. a collision was detected)
            eSuccess,       // the extension succeeded completely
            ePartial,       // the extension was performed only partially
        };

        /*!
            Constructor
            \param cspace A cspace, defining the used joints (the dimension/DoF) and the collision detection setup.
            \param mode Specify the RRT method that should be used
            \param probabilityExtendToGoal Specify how often the goal node should be used instead of a random config (value must be between 0 and 1)
        */
        Rrt(CSpaceSampledPtr cspace, RrtMethod mode = eConnect, float probabilityExtendToGoal = 0.1f);
        virtual ~Rrt();

        /*!
            Do the planning (blocking method). On success the Rrt can be accessed with the getTree() method and the
            found solution with getSolution().
            \param bQuiet Print some info or not.
            \return true if solution was found, otherwise false
        */
        virtual bool plan(bool bQuiet = false);

        virtual void printConfig(bool printOnlyParams = false);

        virtual void reset();

        //! set start configuration
        virtual bool setStart(const Eigen::VectorXf& c);

        //! set goal configuration
        virtual bool setGoal(const Eigen::VectorXf& c);

        void setProbabilityExtendToGoal(float p);

        CSpaceTreePtr getTree();

    protected:

        virtual bool createSolution(bool bQuiet = false);

        CSpaceTreePtr tree;                 //!< the rrt on which are operating


        virtual ExtensionResult extend(Eigen::VectorXf& c, CSpaceTreePtr tree, int& storeLastAddedID);
        virtual ExtensionResult connectComplete(Eigen::VectorXf& c, CSpaceTreePtr tree, int& storeLastAddedID);
        virtual ExtensionResult connectUntilCollision(Eigen::VectorXf& c, CSpaceTreePtr tree, int& storeLastAddedID);

        CSpaceNodePtr startNode;        //!< start node (root of RRT)
        CSpaceNodePtr goalNode;         //!< goal node (set when RRT weas successfully connected to goalConfig)

        Eigen::VectorXf tmpConfig;      //!< tmp config

        float extendGoToGoal;           //!< the probability that the goal config is used instead of a randomly created configuration

        float extendStepSize;           //!< step size for one rrt extend (copied from cspace)
        int lastAddedID;                //!< ID of last added node

        RrtMethod rrtMode;
    };

} // namespace

#endif // _Saba_RRT_h

