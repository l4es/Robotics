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
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Stefan Ulbrich
* @copyright  2011 Stefan Ulbrich
* license     http://www.gnu.org/licenses/gpl.txt
*             GNU General Public License
*/

#ifndef _LinkedCoordinate_h_
#define _LinkedCoordinate_h_

#include "VirtualRobotImportExport.h"
#include "Nodes/RobotNode.h"
#include <string>
#include <vector>
#include "VirtualRobotException.h"


namespace VirtualRobot
{


    /** @brief This class provides intelligent coordinates for the virtual robot.
     *
     * @details The intelligent coordinates (and poses!)  are connected to the virtual robot
     * and are aware of the reference frame they have been defined in. In addition,
     * they allow coordinate transformations using the current state of the robot.
     *
     * @todo Let instances conveniently be created by RobotNode and Robot instances.
     * @todo Look for convenient converter functions. For instance, QuaternionToMatrix4f.
     * @todo Compile and design test cases. For instance, the transformation between to
     * successing joints should be easy to confirm if there is a getTransformFromParent().
     * @todo Check if the order of const and throw is okay.
     */
    class VIRTUAL_ROBOT_IMPORT_EXPORT LinkedCoordinate //: public boost::enable_shared_from_this<LinkedCoordinate>
    {
    public:


        /** Creates a new intelligent coordinate.
         * @param virtualRobot Pointer to the virtual robot the coordinate is connected to.
         */
        LinkedCoordinate(const RobotPtr& virtualRobot) : robot(virtualRobot), pose(Eigen::Matrix4f::Identity()) {}
        LinkedCoordinate(const LinkedCoordinate& other);    // copy constructor
        LinkedCoordinate& operator=(const LinkedCoordinate& other);

        virtual ~LinkedCoordinate();

        /** Sets the frame of reference and the pose relative to it.
         * @param frame The name of the robot node that defines the reference frame of the coordinate.
         * @param pose A homogeneous matrix that defines the pose relative to the frame of reference.
         *      If omitted, it defaults to the identity matrix.
         * @throw VirtualRobotException An exception is thrown if frame does not belong to the robot
         *      or the reference is empty.
         * @details If you want to use a different format for the pose you can use these
         * helper functions @sa EulerToMatrix4x4 and @sa QuaternionsToMatrix4x4
         */
        void set(const RobotNodePtr& frame, const Eigen::Matrix4f& pose = Eigen::Matrix4f::Identity());

        /** Sets the frame of reference and the coordinate relative to it.
         * @param frame The name of the robot node that defines the reference frame of the coordinate.
         * @param position A translational vector defines the position relative to the frame of reference.
         * @throw VirtualRobotException An exception is thrown if frame does not belong to the robot.
         * @details A homogeneous vector can be converted using Eigen::Vector4f::head<3>().
         */

        void set(const RobotNodePtr& frame, const Eigen::Vector3f& position);

        /** Sets the frame of reference and the pose relative to it.
         * @param frame The name of the robot node that defines the reference frame of the coordinate.
         * @param pose A homogeneous matrix that defines the pose relative to the frame of reference.
         *      If omitted, it defaults to the identity matrix.
         * @throw VirtualRobotException An exception is thrown if frame does not belong to the robot
         *      or the reference is empty.
         * @sa set(Eigen::Matrix4x4,const RobotNodePtr &)
         */

        void set(const std::string& frame, const Eigen::Matrix4f& pose = Eigen::Matrix4f::Identity());

        /** Sets the frame of reference and the coordinate relative to it.
         * @param frame The name of the robot node that defines the reference frame of the coordinate.
         * @param position The pose relative to the frame of reference.
         * @throw VirtualRobotException An exception is thrown if frame does not belong to the robot.
         * @details A homogeneous vector can be previously converted using Eigen::Vector4f::head<3>().
         */

        void set(const std::string& frame, const Eigen::Vector3f& position);

        /** Returns a RobotNodePtr that contains the frame of reference the coordinate is defined in.
         * @returns The reference or an empty RobotNodePtr is none has been defined.
         * @details The actual matrix then can be obtained by calling RobotNode::getGlobalPose().
         */
        inline RobotNodePtr getFrame() const
        {
            return this->frame;
        };

        /** Sets a new reference frame and performs a coordinate transformation into this new frame.
         * @param frame The name of the robot node that defines the new reference.
         * @details If iCoord::set has not been called before, the pose is set to the
         * unit marix.
         */
        void changeFrame(const std::string& frame);

        /** Sets a new reference frame and performs a coordinate transformation into this new frame.
         * @param frame A reference to the robot node that defines the new reference.
         * @details If iCoord::set has not been called before, the pose is set to the
         * unit marix.
         * @throw VirtualRobotException An exception is thrown if frame is NULL.
         */
        void changeFrame(const RobotNodePtr& frame);

        /** Performs \a only a coordinate transformation into a new frame of reference.
         * @param frame The name of the robot node that defines the new reference.
         * @details If iCoord::set has not been called before, the pose is set to the
         * unit marix.
         * @returns A homogeneous matrix of the pose
         * @sa getPose()
         */
        Eigen::Matrix4f getInFrame(const std::string& frame) const;

        /** Performs \a only a coordinate transformation into a new frame of reference.
         * @param frame A reference to the robot node that defines the new reference.
         * @details If iCoord::set has not been called before, the pose is set to the
         * unit marix.
         * @returns A homogeneous matrix of the pose
         * @sa getPose()
         * @details If you are only interested in the translational part use iCoord::getPosition() or Matrix4f::block<3,1>(0,3).
         * @throw VirtualRobotException An exception is thrown if frame is NULL.
         */
        Eigen::Matrix4f getInFrame(const RobotNodePtr& frame) const;

        /** Returns the actual pose stored in this object.
         * @details If you are only interested in the translational part use iCoord::getPosition() or Matrix4f::block<3,1>(0,3).
         */
        inline Eigen::Matrix4f getPose() const
        {
            return this->pose;
        };

        /** Conveniently returns the translational part of the actual pose stored in this object.
         */
        inline Eigen::Vector3f getPosition() const
        {
            return this->getPose().block<3, 1>(0, 3);
        }

        /// Computes the transformation matrix that represents a coordinate transformation.
        static Eigen::Matrix4f getCoordinateTransformation(const RobotNodePtr& origin,
                const RobotNodePtr& destination, const RobotPtr& robot);

    protected:

        RobotPtr robot;
        Eigen::Matrix4f pose;
        RobotNodePtr frame;
    };

} // namespace VirtualRobot

#endif // _LinkedCoordinate_h_
