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
#ifndef _saba_cspace_h_
#define _saba_cspace_h_

#include "../Saba.h"
#include "VirtualRobot/Robot.h"
#include "VirtualRobot/VirtualRobot.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include <iostream>
#include <vector>

namespace Saba
{


    /*!

     \brief A c-space interface for motion planning.

     A CSpace is defined by a set of robot nodes and a collision manager.
     The RobotNodeSet specifies the dimension and the borders of the CSpace.
     The collision manager is used to check configurations for collisions. Here,
     multiple sets of objects can be defined which are internally mutually checked for
     collisions. This allows to specify complex collision queries.
     When performing sampling-based motion planning, usually paths in c-space have to be verified
     if constraints are violated or collisions occur. Therefore this class provides an interface,
     in order to combine various planning approaches with different collision detection methods

     Please note:
     This implementation is not able to determine the collision status of path segments.
     Use CSpaceSampled for discrete collision detection.

     @see VirtualRobot::CDManager
     @see MotionPlanner, Rrt, BiRrt
     @see CSpaceSampled

     */
    class SABA_IMPORT_EXPORT CSpace : public boost::enable_shared_from_this<CSpace>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        /*!
            Construct a c-space that represents the given set of joints.
            The dimensionality of this c-space is set by the number of nodes in robotNodes.
            The boundaries of this c-space are set according to definitions in robotNodes.
            On startup, memory is allocate din order to allow fats processing on runtime.
        */
        CSpace(VirtualRobot::RobotPtr robot, VirtualRobot::CDManagerPtr collisionManager, VirtualRobot::RobotNodeSetPtr robotNodes, unsigned int maxConfigs = 50000, unsigned int randomSeed = 0);

        //! destructor
        virtual ~CSpace();

        /*!
            Create a path from start to goal without any checks.
            Intermediate configurations are added according to the current implementation of the cspace.
            In this implementation only the start and goal config are added to the path.
        */
        virtual CSpacePathPtr createPath(const Eigen::VectorXf& start, const Eigen::VectorXf& goal);

        /*!
            Create a path from start to the goal configuration until an invalid (collision,constraints) configuration is found.
            Intermediate configurations are added according to the current implementation of the cspace
            In this implementation only the start and goal config are added to the path.
            \param start The start configuration.
            \param goal The goal configuration.
            \param storeAddedLength The length of the collision free path is stored here (1.0 means the complete path from start to goal was valid)
        */
        virtual CSpacePathPtr createPathUntilInvalid(const Eigen::VectorXf& start, const Eigen::VectorXf& goal, float& storeAddedLength);


        /*!
            Set boundary values for each dimension of the cspace.
            Only needed when other values needed than those defined in the corresponding robotNodes.
          \param min minimum values (array has to have same dimension as the cspace)
          \param max maximum values (array has to have same dimension as the cspace)
        */
        void setBoundaries(const Eigen::VectorXf& min, const Eigen::VectorXf& max);
        void setBoundary(unsigned int dim, float min, float max);

        /*!
            Set weights to uniformly handle differing dimensions in c-space.
            Setting the weights with this methods enables weighting of distance calculations in c-space.
            This affects path-creation and distance calculations.
        */
        void setMetricWeights(const Eigen::VectorXf& weights);


        //! get minimum boundary of an index
        float getBoundaryMin(unsigned int d);
        //! get maximum boundary of an index
        float getBoundaryMax(unsigned int d);
        //! get boundary distance of an index
        float getBoundaryDist(unsigned int d);

        bool isBorderlessDimension(unsigned int dim) const;

        //! get cspace dimension
        unsigned int getDimension() const;


        //! get the robot of cspace
        VirtualRobot::RobotPtr getRobot() const;

        //! get the sets of robotnodes representing of cspace
        VirtualRobot::RobotNodeSetPtr getRobotNodeSet() const;

        VirtualRobot::CDManagerPtr getCDManager() const;

        /*!
            Returns a uniformly sampled random value for dimension dim.
        */
        virtual float getRandomConfig_UniformSampling(unsigned int dim);

        /*!
            This method returns a random configuration. The value is generated uniformly (standard)
            or, in case a sampler is defined a custom sampling strategy can be used.
            \param storeValues Store the values here.
            \param checkValid When set to true, it is guaranteed that the sample not in collision and all constraints are considered
        */
        void getRandomConfig(Eigen::VectorXf& storeValues, bool checkValid = false);

        //! set values of configuration considering boundaries
        virtual void respectBoundaries(Eigen::VectorXf& config);

        /*!
            Check path between two configurations.
            This cspace implementation just checks q2!
            \param q1 first configuration (from)
            \param q2 second configuration (to)
            \return true if path is valid
        */
        virtual bool isPathValid(const Eigen::VectorXf& q1, const Eigen::VectorXf& q2);

        /*!
            Method for externally stopping the path checking routines.
            Only useful in multi-threaded planners.
        */
        void requestStop();

        //! get random seed

        unsigned int getRandomSeed() const
        {
            return randomSeed;
        }

        void setRandomSeed(unsigned int random_seed)
        {
            randomSeed = random_seed;
        }

        //! checks complete solution path (with the pathCheck methods provided by the CSpace implementation)
        virtual bool checkSolution(CSpacePathPtr path, bool verbose = false);

        //! checks complete tree (with the pathCheck methods provided by the CSpace implementation)
        virtual bool checkTree(CSpaceTreePtr tree);

        void resetPerformanceVars()
        {
            performaceVars_collisionCheck = 0;
            performaceVars_distanceCheck = 0;
        }

        virtual void reset();

        /*!
            Returns array of size dimension.
        */
        Eigen::VectorXf getMetricWeights()
        {
            return metricWeights;
        }


        /*!
            Enable/Disable weighting of c-space dimensions. (standard: disabled)
            By setting the weights with setMetricWeights() the weighting is enabled.
            \param enable When set, the metric weights of this c-space are used to compute distances (when no metric weights have been specified for this c-space the option has no effect).
        */
        void enableWeights(bool enable);


        /*!
            Method to allow concurrent/parallel access on robot.
         Enabling the multiThreading support is only needed when sharing robot or environment models
         between multiple CSpaces and the collision detection is done in parallel.
         If each CSpace operates on it's own instances of robot and environment, the multithreading support must
         not be enabled. Also rrtBiPlanners do not need to enable these option.
         If enabled, the robot is locked for collision checking, so that only one instance operates on the model.
         Planners operate much faster when doing "real" parallel collision checking, meaning that multiple instances
         of the models are created and these instances have their own collision checker.
         \param bGranted When set to true, no mutex protection is used to access the robot and to perform the collision detection. (standard)
                                         If set to false, the Robot and CD calls are protected by a mutex.
         */
        void exclusiveRobotAccess(bool bGranted = true);
        bool hasExclusiveRobotAccess();


        //! if multithreading is enabled, the colChecking mutex can be locked/unlocked externally
        static void lock();
        //! if multithreading is enabled, the colChecking mutex can be locked/unlocked externally
        static void unlock();

        /*!
        Clone this CSpace structure
        The new Robot and the new CCM are used, the robot and the ccm have to be linked to the new ColChecker!
        */
        virtual CSpacePtr clone(VirtualRobot::CollisionCheckerPtr newCollisionChecker, VirtualRobot::RobotPtr newRobot, VirtualRobot::CDManagerPtr newCDM, unsigned int nNewRandomSeed = 0) = 0;


        /*!
            In the standard setup no sampler is used and the method getRandomConfig() performs a uniformly sampling of random configurations.
            Since most planners use the getRandomConfig() method for generating their random configurations (@see Rrt),
            this method offers the possibility to exchange the sampling routine by a custom sample algorithm (e.g. a brideover-sampler or a
            Gaussian sampler can be realized).
            In case NULL is passed, the sampling routine is set back to the standard uniform sample algorithm.
        */
        void setRandomSampler(SamplerPtr sampler);

        int performaceVars_collisionCheck;
        int performaceVars_distanceCheck;

        /*!
            Compute the distance in c-space.
            The modes useMetricWeights (standard:disabled) and checkForBorderlessDims (stanbdard:enabled) may affect the results.
            \see setMetricWeights
            \see checkForBorderlessDimensions

        */
        float calcDist(const Eigen::VectorXf& c1, const Eigen::VectorXf& c2, bool forceDisablingMetricWeights = false);
        float calcDist2(const Eigen::VectorXf& c1, const Eigen::VectorXf& c2, bool forceDisablingMetricWeights = false);


        //! calculate distance to obstacles
        /*!
          \param config The config for which the obstacle distance should be calculated (according to the cdm of this cspace)
        */
        virtual float calculateObstacleDistance(const Eigen::VectorXf& config);

        CSpaceNodePtr getNode(unsigned int id);

        virtual CSpaceNodePtr createNewNode();
        virtual void removeNode(CSpaceNodePtr node);

        //! returns the collision status (true for a valid config)
        virtual bool isCollisionFree(const Eigen::VectorXf& config);

        //! returns the boundary violation status (true for a valid config)
        virtual bool isInBoundary(const Eigen::VectorXf& config);

        //! returns the constraint violation status (true for a valid config)
        virtual bool isSatisfyingConstraints(const Eigen::VectorXf& config);

        void printConfig(const Eigen::VectorXf& c) const;


        /*!
            When the size of a c-space dimension is > 2PI and the corresponding joint is rotational, it is assumed that there are no borders
            \param enable When set, for each c-space dimension the (rotational) corresponding joints are checked:
                Borderless dimension, if the distance between Lo and Hi limit is >2PI
                This is useful e.g. for free flying or holonomic moving robots.
                Translational joints are not considered.
        */
        void checkForBorderlessDimensions(bool enable);

        /*!
            Interpolates between two values in dimension dim.
            Checks weather the corresponding joint moves translational or a rotational and performs the interpolation accordingly.
            When joint boundaries of a rotational joint are >= 2PI (and checkForBorderlessDimensions was not disabled), the correct direction for interpolating is determined automatically.
        */
        float interpolate(const Eigen::VectorXf& q1, const Eigen::VectorXf& q2, int dim, float step);
        Eigen::VectorXf interpolate(const Eigen::VectorXf& q1, const Eigen::VectorXf& q2, float step);


        //! check whether a configuration is valid (collision, boundary, and constraints check)
        virtual bool isConfigValid(const Eigen::VectorXf& pConfig, bool checkBorders = true, bool checkCollisions = true, bool checkConstraints = true);

        /*!
            Add a configuration constraint to be checked within this cspace.
            Standard: No constraints, meaning that a check for constraints will report a valid status
        */
        virtual void addConstraintCheck(Saba::ConfigurationConstraintPtr constraint);

    protected:



        // gets direction vector from c1 to c2, with (weighted) length
        virtual void getDirectionVector(const Eigen::VectorXf& c1, const Eigen::VectorXf& c2, Eigen::VectorXf& storeDir, float length);

        virtual void generateNewConfig(const Eigen::VectorXf& randomConfig, const Eigen::VectorXf& nearestConfig, Eigen::VectorXf& storeNewConfig, float stepSize, float preCalculatedDist = -1.0);



        //! return upper limit for movement of any point on joints if moving from config to nextConfig
        virtual float getDirectedMaxMovement(const Eigen::VectorXf& config, const Eigen::VectorXf& nextConfig);

        static int cloneCounter;

        //! interpolates linear between a and b using step as step size
        float interpolateLinear(float a, float b, float step);

        //! interpolates rotational between a and b using step as step size
        float interpolateRotational(float a, float b, float step);

        unsigned int dimension;                                     //!< dimension of this c-space

        Eigen::VectorXf boundaryMax, boundaryMin, boundaryDist;     //!< boundaries of this c-space

        Eigen::VectorXf metricWeights;                              //!< weights for distance computation

        bool stopPathCheck;

        VirtualRobot::RobotPtr robo;                                //!< the robot for collision checking
        VirtualRobot::RobotNodeSetPtr robotNodes;                   //!< the robot nodes defining the c-space

        VirtualRobot::CDManagerPtr cdm;                             //!< handling of collision detections


        int maxNodes;
        std::vector< CSpaceNodePtr > nodes;                         //! vector with pointers to really used nodes
        std::vector< CSpaceNodePtr > freeNodes;                     //! vector with pointers to free (not used) nodes

        std::vector<VirtualRobot::RobotNodePtr> robotJoints;        //!< joints of the robot that we are manipulating

        unsigned int randomSeed;

        float randMult;
        bool useMetricWeights;
        bool checkForBorderlessDims;
        std::vector< bool > borderLessDimension;         // store borderless state

        bool multiThreaded;                             // indicates that more than one CSpace is used by some threads
        static boost::mutex colCheckMutex;              // only needed when multithreading support is enabled
        //  -> setting the configurations and checking against collisions is protected by this mutex
        std::vector<ConfigurationConstraintPtr>  constraints;

        SamplerPtr sampleAlgorithm; // standard is NULL (uniformly sampling), is used in getRandomConfig()
    };

} // namespace

#endif // _saba_cspace_h
