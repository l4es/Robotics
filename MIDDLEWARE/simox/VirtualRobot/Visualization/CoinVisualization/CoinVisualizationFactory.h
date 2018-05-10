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
* @package    VirtualRobot
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010, 2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_CoinVisualizationFactory_h_
#define _VirtualRobot_CoinVisualizationFactory_h_


#include "../../VirtualRobotImportExport.h"
#include "../VisualizationFactory.h"
#include "../../BoundingBox.h"
#include "../../SceneObject.h"
#include "../../EndEffector/EndEffector.h"
#include "../ColorMap.h"
#include "../../Workspace/WorkspaceRepresentation.h"

#include <Inventor/SoInput.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/SoOffscreenRenderer.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoCamera.h>



#include <string>
#include <fstream>

namespace VirtualRobot
{
    class VisualizationNode;

    /*!
        A Coin3D based implementation of a VisualizationFactory.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT CoinVisualizationFactory  : public VisualizationFactory
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CoinVisualizationFactory();
        virtual ~CoinVisualizationFactory();

        virtual VisualizationNodePtr getVisualizationFromPrimitives(const std::vector<Primitive::PrimitivePtr>& primitives, bool boundingBox = false, Color color = Color::Gray());
        virtual VisualizationNodePtr getVisualizationFromFile(const std::string& filename, bool boundingBox = false);
        virtual VisualizationNodePtr getVisualizationFromSTLFile(const std::string& filename, bool boundingBox = false);
        virtual VisualizationNodePtr getVisualizationFromCoin3DFile(const std::string& filename, bool boundingBox = false);
        virtual VisualizationNodePtr getVisualizationFromFile(const std::ifstream& ifs, bool boundingBox = false);
        virtual VisualizationNodePtr getVisualizationFromString(const std::string& modelString, bool boundingBox = false);
        virtual VisualizationNodePtr createBox(float width, float height, float depth, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        virtual VisualizationNodePtr createLine(const Eigen::Vector3f& from, const Eigen::Vector3f& to, float width = 1.0f, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        virtual VisualizationNodePtr createLine(const Eigen::Matrix4f& from, const Eigen::Matrix4f& to, float width = 1.0f, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        virtual VisualizationNodePtr createSphere(float radius, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        virtual VisualizationNodePtr createCylinder(float radius, float height, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        virtual VisualizationNodePtr createCoordSystem(float scaling = 1.0f, std::string* text = NULL, float axisLength = 100.0f, float axisSize = 3.0f, int nrOfBlocks = 10);
        virtual VisualizationNodePtr createBoundingBox(const BoundingBox& bbox, bool wireFrame = false);
        virtual VisualizationNodePtr createVertexVisualization(const Eigen::Vector3f& position, float radius, float transparency,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        virtual VisualizationNodePtr createTriMeshModelVisualization(TriMeshModelPtr model, Eigen::Matrix4f& pose);
        virtual VisualizationNodePtr createTriMeshModelVisualization(TriMeshModelPtr model, bool showNormals, Eigen::Matrix4f& pose, bool showLines = true);
        virtual VisualizationNodePtr createPlane(const Eigen::Vector3f& position, const Eigen::Vector3f& normal, float extend, float transparency, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        virtual VisualizationNodePtr createArrow(const Eigen::Vector3f& n, float length = 50.0f, float width = 2.0f, const Color& color = Color::Gray());
        virtual VisualizationNodePtr createTrajectory(TrajectoryPtr t, Color colorNode = Color::Blue(), Color colorLine = Color::Gray(), float nodeSize = 15.0f, float lineSize = 4.0f);
        virtual VisualizationNodePtr createText(const std::string& text, bool billboard = false, float scaling = 1.0f, Color c = Color::Black(), float offsetX = 20.0f, float offsetY = 20.0f, float offsetZ = 0.0f);
        /*!
            Creates an coordinate axis aligned ellipse
            \param x The extend in x direction must be >= 1e-6
            \param y The extend in y direction must be >= 1e-6
            \param z The extend in z direction must be >= 1e-6
            \param showAxes If true, the axes are visualized
            \param axesHeight The height of the axes (measured from the body surface)
            \param axesWidth The width of the axes.
            \return A VisualizationNode containing the visualization.
        */
        virtual VisualizationNodePtr createEllipse(float x, float y, float z, bool showAxes = true, float axesHeight = 4.0f, float axesWidth = 8.0f);
        /*!
            Move local visualization by homogeneous matrix m.
        */
        virtual void applyDisplacement(VisualizationNodePtr o, Eigen::Matrix4f& m);

        /*!
            Create an empty VisualizationNode.
        */
        virtual VisualizationNodePtr createVisualization();

        /*!
            Create a united visualization. Internally all visualizations are copied and added to one SoSeparator.
            All visualizations have to be of type CoinVisualizationNode.
        */
        virtual VisualizationNodePtr createUnitedVisualization(const std::vector<VisualizationNodePtr>& visualizations) const;


        static SoSeparator* CreateConvexHull2DVisualization(const MathTools::ConvexHull2DPtr ch, MathTools::Plane& p, VisualizationFactory::Color colorInner = VisualizationFactory::Color::Blue(), VisualizationFactory::Color colorLine = VisualizationFactory::Color::Black(), float lineSize = 5.0f, const Eigen::Vector3f& offset = Eigen::Vector3f::Zero());
        static SoSeparator* CreatePolygonVisualization(const std::vector<Eigen::Vector3f>& points, VisualizationFactory::Color colorInner = VisualizationFactory::Color::Blue(), VisualizationFactory::Color colorLine = VisualizationFactory::Color::Black(), float lineSize = 4.0f);
        static SoSeparator* CreatePolygonVisualization(const std::vector<Eigen::Vector3f>& points, VisualizationFactory::PhongMaterial mat, VisualizationFactory::Color colorLine = VisualizationFactory::Color::Black(), float lineSize = 4.0f);
        static SoSeparator* CreatePlaneVisualization(const Eigen::Vector3f& position, const Eigen::Vector3f& normal, float extend, float transparency, bool grid = true,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f, std::string textureFile = std::string());
        static SoSeparator* CreateCoordSystemVisualization(float scaling = 1.0f, std::string* text = NULL, float axisLength = 100.0f, float axisSize = 3.0f, int nrOfBlocks = 10);
        static SoSeparator* CreateBoundingBox(SoNode* ivModel, bool wireFrame = false);
        static SoSeparator* CreateGrid(float width, float depth, float widthMosaic, float depthMosaic, bool InvertNormal, const char* pFileName, float Transparency);
        static SoSeparator* CreateBBoxVisualization(const BoundingBox& bbox, bool wireFrame = false);
        static SoSeparator* CreatePointVisualization(const MathTools::ContactPoint& point, bool showNormals = false);
        static SoSeparator* CreatePointsVisualization(const std::vector<MathTools::ContactPoint>& points, bool showNormals = false);
        static SoSeparator* CreateArrow(const Eigen::Vector3f& n, float length = 50.0f, float width = 2.0f, const Color& color = Color::Gray());
        static SoSeparator* CreateVertexVisualization(const Eigen::Vector3f& position, float radius, float transparency, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);

        /*!
            Creates an coordinate axis aligned ellipse
            \param x The extend in x direction must be >= 1e-6
            \param y The extend in y direction must be >= 1e-6
            \param z The extend in z direction must be >= 1e-6
            \param matBody If not given a standard material is used for the ellipse body
            \param showAxes If true, the axes are visualized
            \param axesHeight The height of the axes (measured from the body surface)
            \param axesWidth The width of the axes.
            \param matAxisX If not given a standard material is used for axis X
            \param matAxisY If not given a standard material is used for axis Y
            \param matAxisZ If not given a standard material is used for axis Z
            \return A separator containing the visualization.
        */
        static SoSeparator* CreateEllipse(float x, float y, float z,
                                          SoMaterial* matBody = NULL,
                                          bool showAxes = true,
                                          float axesHeight = 4.0f,
                                          float axesWidth = 8.0f,
                                          SoMaterial* matAxisX = NULL,
                                          SoMaterial* matAxisY = NULL,
                                          SoMaterial* matAxisZ = NULL
                                         );

        static SoSeparator* Create2DMap(const Eigen::MatrixXf& d, float extendCellX, float extendCellY, const VirtualRobot::ColorMap cm = VirtualRobot::ColorMap(VirtualRobot::ColorMap::eHot), bool drawZeroCells = false, bool drawLines = true);
        static SoSeparator* Create2DHeightMap(const Eigen::MatrixXf& d, float extendCellX, float extendCellY, float heightZ, const VirtualRobot::ColorMap cm = VirtualRobot::ColorMap(VirtualRobot::ColorMap::eHot), bool drawZeroCells = false, bool drawLines = true);

        static SoSeparator* CreateOOBBVisualization(const MathTools::OOBB& oobb, Color colorLine = Color::Gray(), float lineSize = 4.0f);
        static SoSeparator* CreateSegmentVisualization(const MathTools::Segment& s, Color colorLine = Color::Gray(), float lineSize = 4.0f);

        /*!
            Creates a colored model, by creating a new SoSeparator and adding a basecolor with overide flags followed by the model.
        */
        static SoSeparator* Colorize(SoNode* model, VisualizationFactory::Color c);

        static SbMatrix getSbMatrix(const Eigen::Matrix4f& m);
        static SbMatrix getSbMatrixVec(const Eigen::Vector3f& m);

        /*!
            Create a visualization of a set of grasps.
            \param graspSet The grasps to visualize
            \param eef The visualization of this eef is used to visualize the grasps
            \param pose The grasp set is visualized relatively to this pose (e.g. use the object position here)
            \param visu The visualization type of the EEFs.
        */
        static SoSeparator* CreateGraspSetVisualization(GraspSetPtr graspSet, EndEffectorPtr eef, const Eigen::Matrix4f& pose = Eigen::Matrix4f::Identity(), SceneObject::VisualizationType visu = SceneObject::Full);
        static SoSeparator* CreateGraspVisualization(GraspPtr grasp, SoSeparator* eefVisu, const Eigen::Matrix4f& pose = Eigen::Matrix4f::Identity());
        static SoSeparator* CreateGraspVisualization(GraspPtr grasp, EndEffectorPtr eef, const Eigen::Matrix4f& pose = Eigen::Matrix4f::Identity(), SceneObject::VisualizationType visu = SceneObject::Full);

        /*!
            Create a visualization of the end effector.
            The visualization is moved, so that the origin is identical with the coordinate system of the TCP.
        */
        static SoSeparator* CreateEndEffectorVisualization(EndEffectorPtr eef, SceneObject::VisualizationType = SceneObject::Full);

        /*!
            Creates a material node.
        */
        static SoNode* getColorNode(Color color);
        /*!
            Text visu. Offsets in mm.
        */
        static SoSeparator* CreateText(const std::string& s, float offsetX = 20.0f, float offsetY = 20.0f, float offsetZ = 0.0f);
        /*!
            Billboard Text visu. Offsets in mm.
        */
        static SoSeparator* CreateBillboardText(const std::string& s, float offsetX = 20.0f, float offsetY = 20.0f, float offsetZ = 0.0f);

        /*!
            Convenient method to retrieve a coin visualization for a robot
        */
        static SoNode* getCoinVisualization(RobotPtr robot, SceneObject::VisualizationType visuType);
        /*!
            Convenient method to retrieve a coin visualization for a SceneObject/Obstacle/ManipulationObject
        */
        static SoNode* getCoinVisualization(SceneObjectPtr object, SceneObject::VisualizationType visuType);

        /*!
            Convenient method to retrieve a coin visualization for a set of contacts.
            \param contacts The contacts to be visualized
            \param frictionConeHeight The height of the friction cone [mm].
            \param frictionConeRadius The radius of the cone [mm].
            \param scaleAccordingToApproachDir If true, the parallel component of the contact normal is computed according to the approahcDirection of the contact.
        */
        static SoNode* getCoinVisualization(VirtualRobot::EndEffector::ContactInfoVector& contacts, float frictionConeHeight = 30.0f,  float frictionConeRadius = 15.0f, bool scaleAccordingToApproachDir = true);
        /*!
            Convenient method to retrieve a coin visualization for a contact.
            \param contact The contact to be visualized
            \param frictionConeHeight The height of the friction cone [mm].
            \param frictionConeRadius The radius of the cone [mm].
            \param scaleAccordingToApproachDir If true, the parallel component of the contact normal is computed according to the approahcDirection of the contact.
        */
        static SoNode* getCoinVisualization(EndEffector::ContactInfo& contact, float frictionConeHeight = 30.0f,  float frictionConeRadius = 15.0f, bool scaleAccordingToApproachDir = true);

        static SoNode* getCoinVisualization(VisualizationNodePtr visu);

        static SoNode* getCoinVisualization(TriMeshModelPtr model);
        static SoNode* getCoinVisualization(TriMeshModelPtr model, bool shownormals, VisualizationFactory::Color color = VisualizationFactory::Color::Gray(), bool showLines = true);


        static SoNode* getCoinVisualization(TrajectoryPtr t, Color colorNode = Color::Blue(), Color colorLine = Color::Gray(), float nodeSize = 15.0f, float lineSize = 4.0f);
        /*!
            Create a visualization of the reachability data.
        */
        static SoNode* getCoinVisualization(WorkspaceRepresentationPtr reachSpace, const VirtualRobot::ColorMap cm, bool transformToGlobalPose = true);
        /*!
            Creates a visualization of the reachability data. For each 3D point, the orientation with maximum entry is determined and visualized as an arrow. The direction of this arrow is aligned to the param axis.
        */
        static SoNode* getCoinVisualization(WorkspaceRepresentationPtr reachSpace, VirtualRobot::ColorMap cm, const Eigen::Vector3f& axis, bool transformToGlobalPose = true, unsigned char minValue = 0, float arrowSize = 0, int nrRotations = 1);
        //! Helper method: Create reach space visualization of a fixed orientation
        static SoNode* getCoinVisualization(WorkspaceRepresentationPtr reachSpace, const Eigen::Vector3f& fixedEEFOrientationGlobalRPY, VirtualRobot::ColorMap cm = VirtualRobot::ColorMap(VirtualRobot::ColorMap::eHot), bool transformToGlobalPose = true, const Eigen::Vector3f& axis = Eigen::Vector3f(0, 0, 1.0f), unsigned char minValue = 0, float arrowSize = 0);
        //! helper method: create nrBestEntries arrows in direction of maximum orientation for voxelPosition (a,b,c)
        static SoNode* getCoinVisualization(WorkspaceRepresentationPtr reachSpace, int a, int b, int c, int nrBestEntries, SoSeparator* arrow, const VirtualRobot::ColorMap& cm, bool transformToGlobalPose, unsigned char minValue);

        static SoNode* getCoinVisualization(TSRConstraintPtr constraint, const Color& color);
        static SoNode* getCoinVisualization(BalanceConstraintPtr constraint, const Color& color);
        static SoNode* getCoinVisualization(PoseConstraintPtr constraint, const Color& color);

        static SoNode* getCoinVisualization(SupportPolygonPtr supportPolygon, const Color& color);

        /*!
            Create a SoMatrixTransform from the given pose
            \param m The pose with translation given in meter.
        */
        static SoMatrixTransform* getMatrixTransform(Eigen::Matrix4f& m);

        /*!
            Create a SoMatrixTransform from the given pose
            Converts the pose from MM to M (scales by 0.001)
            \param m The pose with translation given in millimeter.
        */
        static SoMatrixTransform* getMatrixTransformScaleMM2M(Eigen::Matrix4f& m);
        static SoNode* createCoinLine(const Eigen::Matrix4f& from, const Eigen::Matrix4f& to, float width, float colorR, float colorG, float colorB);

        /*!
            Creates a visualization of the reachability grid.
        */
        static SoNode* getCoinVisualization(WorkspaceGridPtr reachGrid, VirtualRobot::ColorMap cm, bool transformToGlobalPose = true);

        /*!
            Create quads according to cutXY plane. Normal can be UnitX, UnitY or UnitZ.
        */
        static SoNode* getCoinVisualization(VirtualRobot::WorkspaceRepresentation::WorkspaceCut2DPtr cutXY, VirtualRobot::ColorMap cm, const Eigen::Vector3f& normal = Eigen::Vector3f::UnitY());

        /*!
            Create an offscreen renderer object with the given width and height.

            \see renderOffscreen
        */
        static SoOffscreenRenderer* createOffscreenRenderer(int width, int height);

        /*!
            The cam node has to be oriented as follows:
            The camera is pointing along the positive z axis and the positive x axis is pointing upwards.

            \param renderer The renderer should have been created with the createOffscreenRenderer method
            \param camNode The node of the robot that defines the position of the camera. Any node of the robot can host a camera.
            \param scene The scene that should be rendered.
            \param buffer The result is stored here. The origin of the 2D image is at the left bottom!
            The resulting buffer has the size width*height*3, with the extends as defined in the createOffscreenRenderer method.

            \return true on success

            \see createOffscreenRenderer
        */
        static bool renderOffscreen(SoOffscreenRenderer* renderer, RobotNodePtr camNode, SoNode* scene, unsigned char** buffer);

        /*!
        Use a custom camera for rendering
        \param renderer The renderer should have been created with the createOffscreenRenderer method
        \param cam The camera.
        \param scene The scene that should be rendered.
        \param buffer The result is stored here. The origin of the 2D image is at the left bottom!
        The resulting buffer has the size width*height*3, with the extends as defined in the createOffscreenRenderer method.
        \return true on success

        \see createOffscreenRenderer
        */
        static bool renderOffscreen(SoOffscreenRenderer* renderer, SoCamera* cam, SoNode* scene, unsigned char** buffer);

        /*!
            When SoFiles are used, Coin3D just stores references to files instead of the real contents.
            This may cause problems when saving an inventor file. With this method, a group node can be converted in
            order to ensure that all potential files are loaded and added to the group.
        */
        static SoGroup* convertSoFileChildren(SoGroup* orig);

        static SoNode* copyNode(SoNode* n);

        /*!
            Here, a manual cleanup can be called, no Coin3D access possible after this.
            Usually no need to call cleanup explicitly, since cleanup is performed automatically at application exit.
        */
        virtual void cleanup();
    protected:
        static SoNode* GetNodeFromPrimitive(Primitive::PrimitivePtr primitive, bool boundingBox, Color color);
        static void GetVisualizationFromSoInput(SoInput& soInput, VisualizationNodePtr& visualizationNode, bool bbox = false);

        static inline char IVToolsHelper_ReplaceSpaceWithUnderscore(char input)
        {
            if (' ' == input)
            {
                return '_';
            }
            else
            {
                return input;
            }
        }

        // AbstractFactoryMethod
    public:
        static std::string getName();
        static boost::shared_ptr<VisualizationFactory> createInstance(void*);
    private:
        static SubClassRegistry registry;
    };

    typedef boost::shared_ptr<CoinVisualizationFactory> CoinVisualizationFactoryPtr;


} // namespace VirtualRobot

#endif // _VirtualRobot_CoinVisualizationFactory_h_
