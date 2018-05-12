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
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_Trajectory_h_
#define _VirtualRobot_Trajectory_h_

#include "VirtualRobotImportExport.h"
#include "MathTools.h"
#include "RobotNodeSet.h"

#include <Eigen/Core>
#include <vector>

namespace VirtualRobot
{

    /*!
        A representation of a trajectory in joint space, associated with a RobotNodeSet.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT Trajectory : public boost::enable_shared_from_this<Trajectory>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Construct a trajectory for the given set of RobotNodes
        */
        Trajectory(RobotNodeSetPtr rns, const std::string& name = "");

        virtual ~Trajectory();



        std::vector <Eigen::VectorXf> getPoints();


        /*!
          Insert a configuration as one point to the trajectory.
          \param c configuration vector
        */
        void addPoint(const Eigen::VectorXf& c);

        /*!
          Get a point in trajectory with a index.
          \param nr index to point in trajectory
        */
        Eigen::VectorXf getPoint(unsigned int nr) const;

        //! return total number of trajectory points
        unsigned int getNrOfPoints() const;

        //! to retrieve entries of trajectory
        bool getPoints(unsigned int start, unsigned int end , std::vector< Eigen::VectorXf >& storePosList) const;


        /*!
          Creates a copy of the instance.
          \return pointer to new instance (copy)
        */
        TrajectoryPtr clone() const;

        /*!
            Create a new trajectory from startIndex to endIndex.
        */
        TrajectoryPtr createSubPath(unsigned int startIndex, unsigned int endIndex) const;

        //! reset all data
        virtual void reset();

        //! reverse position order: end becomes start
        virtual void reverse();


        /*!
          Erase a point in trajectory.
          \param pos position of point in trajectory array
        */
        virtual void erasePosition(unsigned int pos);


        /*!
           Erases all points from start to end in trajectory (including start&end).
          \param startPos start position of point in trajectory array
          \param endPos end position of point in trajectory array
        */
        virtual unsigned int removePositions(unsigned int startPos, unsigned int endPos);

        /*!
          Insert a point into trajectory.
          \param pos position of point being inserted
          \param c configuration / valid joint values to insert as a point in trajectory
        */
        virtual void insertPosition(unsigned int pos, const Eigen::VectorXf& c);
        virtual void insertPosition(unsigned int pos, std::vector<Eigen::VectorXf >& newConfigurations);
        virtual void insertTrajectory(unsigned int pos, TrajectoryPtr trajectoryToInsert);

        /*!
            Returns the euclidean length of the complete trajectory.
        */
        virtual float getLength() const;

        /*!
         return position on trajectory for time t (0<=t<=1)
         If storeIndex!=NULL the index of the last trajectory point is stored
         */
        virtual void interpolate(float t, Eigen::VectorXf& storePos, int* storeIndex = NULL) const;


        /*!
        */
        unsigned int getDimension() const;


        //! prints trajectory contents to console
        virtual void print() const;

        //! For quick access to data.
        const std::vector <Eigen::VectorXf >& getData() const;

        VirtualRobot::RobotNodeSetPtr getRobotNodeSet();

        /*!
            Creates the corresponding trajectory in workspace.
            \param r The RobotNode that should be considered (if not set, the TCP of the RobotNodeSet is used)
            \return For each point of this joint space trajectory, the pose of r in workspace is computed and added to the resulting vector.
        */
        std::vector< Eigen::Matrix4f > createWorkspaceTrajectory(VirtualRobot::RobotNodePtr r = VirtualRobot::RobotNodePtr());


        /*!
            Create an XML string. All lines are indented with tabs tab stops.
        */
        virtual std::string toXML(int tabs = 0) const;

        std::string getName() const;

        std::string getRobotName() const;

        /*!
            Get a visualization for this trajectory.
            \param visualizationFactoryName The string that identifies the factory. If not given, the first registered factory (which is usually the only one) is used.
        */
        VisualizationNodePtr getVisualization(std::string visualizationFactoryName = "");

    protected:
        std::vector < Eigen::VectorXf > path; //!< vector with configurations which represent the path
        RobotNodeSetPtr rns;
        std::string name;
        unsigned int dimension;     //!< dimension of rns
    };


} // namespace VirtualRobot

#endif /* _VirtualRobot_Trajectory_h_ */
