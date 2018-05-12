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
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_SceneObject_h_
#define _VirtualRobot_SceneObject_h_

#include "VirtualRobot.h"
#include "VirtualRobotException.h"
#include "VirtualRobotImportExport.h"
#include "Visualization/VisualizationNode.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <iomanip>
#include <vector>




namespace VirtualRobot
{
    class Robot;

    class VIRTUAL_ROBOT_IMPORT_EXPORT SceneObject : public boost::enable_shared_from_this<SceneObject>
    {
        friend class RobotFactory;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        enum VisualizationType
        {
            Full,           //!< the full model
            Collision,      //!< the collision model
            CollisionData   //!< a visualization of the collision model data that is internally used (this mode is only for debug purposes, the model is static, i.e. updates/movements/rotations are not visualized!)
        };

        struct Physics
        {
            enum CoMLocation
            {
                eCustom,            //!< Not related to 3d model, the position is set by hand
                eVisuBBoxCenter     //!< The CoM position is automatically computed from the bounding box of the collision model
            };
            enum SimulationType
            {
                eStatic,        // cannot move, but collide
                eKinematic,     // can be moved, but no dynamics
                eDynamic,       // full dynamic simulation
                eUnknown        // not specified
            };
            Physics()
            {

                localCoM.setZero();
                intertiaMatrix.setIdentity();
                massKg = 0.0f;
                comLocation = eVisuBBoxCenter;
                simType = eUnknown;
            }
            std::string getString(SimulationType s) const
            {
                std::string r;

                switch (s)
                {
                    case eStatic:
                        r = "Static";
                        break;

                    case eKinematic:
                        r = "Kinematic";
                        break;

                    case eDynamic:
                        r = "Dynamic";
                        break;

                    default:
                        r = "Unknown";
                }

                return r;
            }

            void print() const
            {
                std::cout << " ** Simulation Type: " << getString(simType) << endl;
                std::cout << " ** Mass: ";

                if (massKg <= 0)
                {
                    std::cout << "<not set>" << std::endl;
                }
                else
                {
                    std::cout << massKg << " [kg]" << std::endl;
                }

                cout << " ** local CoM [mm] ";

                if (comLocation == SceneObject::Physics::eVisuBBoxCenter)
                {
                    std::cout << "(center of visualization's bounding box):";
                }
                else
                {
                    std::cout << ":";
                }

                std::cout << localCoM(0) << ", " << localCoM(1) << ", " << localCoM(2) << std::endl;
                {
                    // scope
                    std::ostringstream sos;
                    sos << std::setiosflags(std::ios::fixed);
                    sos << " ** inertial matrix [kg*m^2]:" << std::endl << intertiaMatrix << std::endl;
                    std::cout << sos.str();
                } // scope

                if (ignoreCollisions.size() > 0)
                {
                    std::cout << " ** Ignore Collisions with:" << std::endl;

                    for (size_t i = 0; i < ignoreCollisions.size(); i++)
                    {
                        std::cout << " **** " << ignoreCollisions[i] << std::endl;
                    }
                }
            }

            bool isSet()
            {
                return (massKg != 0.0f || comLocation != eVisuBBoxCenter || !intertiaMatrix.isIdentity() || ignoreCollisions.size() > 0);
            }

            virtual std::string toXML(int tabs)
            {
                std::string ta;
                std::stringstream ss;

                for (int i = 0; i < tabs; i++)
                {
                    ta += "\t";
                }

                ss << ta << "<Physics>\n";

                if (simType != eUnknown)
                {
                    ss << ta << "\t<SimulationType value='" << getString(simType) << "'/>\n";
                }

                ss << ta << "\t<Mass unit='kg' value='" << massKg << "'/>\n";
                ss << ta << "\t<CoM location=";

                if (comLocation == eVisuBBoxCenter)
                {
                    ss << "'VisualizationBBoxCenter'/>\n";
                }
                else
                {
                    ss << "'Custom' x='" << localCoM(0) << "' y='" << localCoM(1) << "' z='" << localCoM(2) << "'/>\n";
                }

                ss << ta << "\t<InertiaMatrix>\n";
                ss << MathTools::getTransformXMLString(intertiaMatrix, tabs + 2, true);
                ss << ta << "\t</InertiaMatrix>\n";

                for (size_t i = 0; i < ignoreCollisions.size(); i++)
                {
                    ss << ta << "\t<IgnoreCollisions name='" << ignoreCollisions[i] << "'/>\n";
                }

                ss << ta << "</Physics>\n";
                return ss.str();
            }

            Physics scale(float scaling) const
            {
                THROW_VR_EXCEPTION_IF(scaling <= 0, "Scaling must be > 0");
                Physics res = *this;
                res.localCoM *= scaling;
                return res;
            }

            Eigen::Vector3f localCoM;   //!< Defined in the local coordinate system of this object [mm]
            float massKg;               //!< The mass of this object
            CoMLocation comLocation;    //!< Where is the CoM located
            Eigen::Matrix3f intertiaMatrix; //! in kg*m^2
            SimulationType simType;
            std::vector< std::string > ignoreCollisions; // ignore collisions with other objects (only used within collision engines)
        };

        /*!
        */
        SceneObject(const std::string& name, VisualizationNodePtr visualization = VisualizationNodePtr(), CollisionModelPtr collisionModel = CollisionModelPtr(), const Physics& p = Physics(), CollisionCheckerPtr colChecker = CollisionCheckerPtr());

        /*!
        */
        virtual ~SceneObject();


        std::string getName() const;

        /*!
            Rename this object
        */
        void setName(const std::string& name);

        /*!
            The global pose defines the position of the object in the world. For non-joint objects it is identical with the visualization frame.
        */
        virtual Eigen::Matrix4f getGlobalPose() const;
        
        /*!
            Usually it is checked weather the object is linked to a parent. This check can be omitted (be careful, just do this if you know the effects)
            \param pose The new pose of this object
        */
        virtual void setGlobalPoseNoChecks(const Eigen::Matrix4f& pose);
        /*!
            Update the pose of this object. The visualization and collision models are updated accordingly.
            \param pose The new pose of this object
        */
        virtual void setGlobalPose(const Eigen::Matrix4f& pose);

        virtual CollisionModelPtr getCollisionModel();
        virtual CollisionCheckerPtr getCollisionChecker();

        /*!
            Sets the main visualization of this object.
        */
        void setVisualization(VisualizationNodePtr visualization);
        void setCollisionModel(CollisionModelPtr colModel);

        /*!
            Return visualization object.
            \param visuType Set the type of visualization.
        */
        virtual VisualizationNodePtr getVisualization(SceneObject::VisualizationType visuType = SceneObject::Full);

        /*!
            Initialize this object. Optionally the parents and children can be specified.
        */
        virtual bool initialize(SceneObjectPtr parent = SceneObjectPtr(), const std::vector<SceneObjectPtr>& children = std::vector<SceneObjectPtr>());

        /*!
            Enables/Disables the visualization updates of collision model and visualization model.
        */
        void setUpdateVisualization(bool enable);
        bool getUpdateVisualizationStatus();

        /*!
            Setup the visualization of this object.
            \param showVisualization If false, the visualization is disabled.
            \param showAttachedVisualizations If false, the visualization of any attached optional visualizations is disabled.
        */
        virtual void setupVisualization(bool showVisualization, bool showAttachedVisualizations);


        /*!
            Display the coordinate system of this object.
            If the object does not own a visualization yet, the VisualizationFactory is queried to get the first registered
            VisualizationType in order to build a valid visualization.
            \p enable Show or hide coordinate system
            \p scaling Size of coordinate system
            \p text Text to display at coordinate system. If not given, the name of this robot node will be displayed.
        */
        virtual void showCoordinateSystem(bool enable, float scaling = 1.0f, std::string* text = NULL);

        /*!
            Display some physics debugging information.
            \p enableCoM If true, the center of mass is shown (if given). If a comModel is given it is used for visualization, otherwise a standrad marker is shown.
            \p enableInertial If true, a visualization of the inertial matrix is shown (if given).
            \p comModel If set, this visualization is used to display the CoM location. If not set, a standard marker is used.
        */
        virtual void showPhysicsInformation(bool enableCoM, bool enableInertial, VisualizationNodePtr comModel = VisualizationNodePtr());

        /*!
            Returns true when the coordinate system is currently shown.
        */
        virtual bool showCoordinateSystemState();


        /*!
            Display the bounding box of this object's collisionModel.
            The bbox is not updated when you move the object, so you have to call this method again after touching the scene in order to ensure a correct visualization.
            If the object does not own a visualization yet, the VisualizationFactory is queried to get the first registered
            VisualizationType in order to build a valid visualization.
            \p enable Show or hide bounding box
            \p wireframe Wireframe or solid visualization.
        */
        virtual void showBoundingBox(bool enable, bool wireframe = false);



        /*!
            Builds a dummy visualization if necessary.
            \param visualizationType    If given the visualization is forced to be this type.
                                        If not set, the first registered visualization from VisualizationFactory is used.
        */
        virtual bool ensureVisualization(const std::string& visualizationType = "");

        /*!
            Transforms the pose, given in global coordinate system, to the local coordinate system of this object.
            @param poseGlobal The pose, given in global coordinate system, that should be transformed to the local coordinate system of this joint.
            @return The transformed pose.
        */
        Eigen::Matrix4f toLocalCoordinateSystem(const Eigen::Matrix4f& poseGlobal) const;
        Eigen::Vector3f toLocalCoordinateSystemVec(const Eigen::Vector3f& positionGlobal) const;
        /*!
            Transforms the pose, given in local coordinate system, to the global coordinate system.
            @param poseLocal The pose, given in local coordinate system of this joint, that should be transformed to the global coordinate system.
            @return The transformed pose.
        */
        Eigen::Matrix4f toGlobalCoordinateSystem(const Eigen::Matrix4f& poseLocal) const;
        Eigen::Vector3f toGlobalCoordinateSystemVec(const Eigen::Vector3f& positionLocal) const;

        /*!
            Returns the transformation matrix from this object to otherObject
        */
        Eigen::Matrix4f getTransformationTo(const SceneObjectPtr otherObject);


        /*!
            Returns the transformation matrix from otherObject to this object
        */
        Eigen::Matrix4f getTransformationFrom(const SceneObjectPtr otherObject);

        /*!
            Transform pose to local coordinate system of this object
        */
        Eigen::Matrix4f transformTo(const SceneObjectPtr otherObject, const Eigen::Matrix4f& poseInOtherCoordSystem);

        /*!
            Transform position to local coordinate system of this object
        */
        Eigen::Vector3f transformTo(const SceneObjectPtr otherObject, const Eigen::Vector3f& positionInOtherCoordSystem);

        /*!
            get number of faces (i.e. triangles) of this object
            \p collisionModel Indicates weather the faces of the collision model or the full model should be returned.
        */
        virtual int getNumFaces(bool collisionModel = false);

        /*!
            Return Center of Mass in local coordinate frame. This method does not consider children.
        */
        virtual Eigen::Vector3f getCoMLocal();
        /*!
            Return Center of Mass in global coordinates. This method does not consider children.
        */
        virtual Eigen::Vector3f getCoMGlobal();

        /*!
            Mass in Kg
        */
        float getMass() const;

        void setMass(float m);

        /*!
            The simulation type is of interest in SimDynamics.
        */
        Physics::SimulationType getSimulationType() const;
        void setSimulationType(Physics::SimulationType s);

        /*
            Inertia matrix in kg*m^2.
        */
        Eigen::Matrix3f getInertiaMatrix();

        void setInertiaMatrix(const Eigen::Matrix3f& im);

        /*!
            Collisions with these models are ignored by physics engine (only considered within the SimDynamics package!).
        */
        std::vector<std::string> getIgnoredCollisionModels();


        virtual void print(bool printChildren = false, bool printDecoration = true) const;


        /*!
            Retrieve a visualization in the given format.
            Example usage:
             boost::shared_ptr<VirtualRobot::CoinVisualization> visualization = robot->getVisualization<CoinVisualization>();
             SoNode* visualisationNode = NULL;
             if (visualization)
                 visualisationNode = visualization->getCoinVisualization();

            @see CoinVisualizationFactory::getCoinVisualization() for convenient access!
        */
        template <typename T> boost::shared_ptr<T> getVisualization(SceneObject::VisualizationType visuType = SceneObject::Full);

        /*!
          Convenient method for highlighting the visualization of this object.
          It is automatically checked whether the collision model or the full model is part of the visualization.

          @see \fn VisualizationNode->highlight()

          \param visualization The visualization for which the highlighting should be performed.
          \param enable Set or unset highlighting.
        */
        void highlight(VisualizationPtr visualization, bool enable);

        /*!
            Clones this object. If no col checker is given, the one of the original object is used.
        */
        SceneObjectPtr clone(const std::string& name, CollisionCheckerPtr colChecker = CollisionCheckerPtr(), float scaling = 1.0f) const
        {
            return SceneObjectPtr(_clone(name, colChecker, scaling));
        }

        /*!
            Attach a connected object. The connected object is linked to this SceneObject and moves accordingly.
            Any call to the child's setGlobalPose is forbidden and will fail.
            \param child The child to attach
        */
        virtual bool attachChild(SceneObjectPtr child);

        /*!
            Removes the child from children list.
        */
        virtual void detachChild(SceneObjectPtr child);


        /*!
            \return true, if child is attached
        */
        virtual bool hasChild(SceneObjectPtr child, bool recursive = false) const;

        /*!
            \return true, if child is attached
        */
        virtual bool hasChild(const std::string& childName, bool recursive = false) const;

        /*!
            \return true, if this object is attached to another object.
        */
        virtual bool hasParent();

        /*!
            \return If this object is attached, the parent is returned. Otherwise an empty object is returned.
        */
        virtual SceneObjectPtr getParent() const;

        virtual std::vector<SceneObjectPtr> getChildren() const
        {
            return children;
        }

        //! Compute the global pose of this object
        virtual void updatePose(bool updateChildren = true);

        /*!
            Saves model files (visu and col model, if present) to model path.
            \param modelPath The path where the model files should be stored.
            \param replsceFilenames If set, the filenames will be replaced with 'name_visu' and 'name_col'. Otherwise the original filename is used without any preceding paths.
        */
        virtual bool saveModelFiles(const std::string& modelPath, bool replaceFilenames);

    protected:
        virtual SceneObject* _clone(const std::string& name, CollisionCheckerPtr colChecker = CollisionCheckerPtr(), float scaling = 1.0f) const;

        //! Parent detached this object
        virtual void detachedFromParent();
        //! Parent attached this object
        virtual void attached(SceneObjectPtr parent);

        virtual void updatePose(const Eigen::Matrix4f& parentPose, bool updateChildren = true);



        std::string getFilenameReplacementVisuModel(const std::string standardExtension = ".wrl");
        std::string getFilenameReplacementColModel(const std::string standardExtension = ".wrl");

        SceneObject() {}

        //! basic data, used by Obstacle and ManipulationObject
        std::string getSceneObjectXMLString(const std::string& basePath, int tabs);

        ///////////////////////// SETUP ////////////////////////////////////
        std::string name;
        bool initialized;                                                       //< Invalid object when false
        ///////////////////////// SETUP ////////////////////////////////////

        Eigen::Matrix4f globalPose;                                             //< The transformation that is used for visualization

        std::vector<SceneObjectPtr> children;
        SceneObjectWeakPtr parent;

        CollisionModelPtr collisionModel;
        VisualizationNodePtr visualizationModel;                                //< This is the main visualization

        bool updateVisualization;

        virtual bool initializePhysics();
        Physics physics;

        CollisionCheckerPtr collisionChecker;
    };


    /**
     * This method creates a new Visualization
     * subclass which is given by the template parameter T.
     * T must be a subclass of VirtualRobot::Visualization.
     * A compile time error is thrown if a different class type is used as template argument.
     */
    template <typename T>
    boost::shared_ptr<T> SceneObject::getVisualization(SceneObject::VisualizationType visuType)
    {
        const bool IS_SUBCLASS_OF_VISUALIZATION = ::boost::is_base_of<Visualization, T>::value;
        BOOST_MPL_ASSERT_MSG(IS_SUBCLASS_OF_VISUALIZATION, TEMPLATE_PARAMETER_FOR_VirtualRobot_getVisualization_MUST_BT_A_SUBCLASS_OF_VirtualRobot__Visualization, (T));
        boost::shared_ptr<T> visualization(new T(getVisualization(visuType)));
        return visualization;
    }

} // namespace VirtualRobot

#endif // _VirtualRobot_SceneObject_h_
