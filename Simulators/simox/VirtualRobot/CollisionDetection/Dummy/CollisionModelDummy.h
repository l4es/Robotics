
#ifndef _CollisionModelDummy_h_
#define _CollisionModelDummy_h_

#include "../../VirtualRobotImportExport.h"
#include "../CollisionModelImplementation.h"
#include <string>
#include <vector>
#include <map>
#include <set>

namespace VirtualRobot
{

    class CollisionChecker;
    class CollisionCheckerDummy;

    class VIRTUAL_ROBOT_IMPORT_EXPORT CollisionModelDummy : public CollisionModelImplementation
    {
    public:
        friend class CollisionModel;

        /*!Standard Constructor
        * If collision checks should be done in parallel, different CollisionCheckers can be specified.
        */
        CollisionModelDummy(CollisionCheckerPtr pColChecker);

        /*!Standard Destructor
        */
        virtual ~CollisionModelDummy();



        /*! Stores ivModel in this collision model (and creates internal representation)
         use this method to create a collision model from an inventor node
         the inventor object is not moved around with the SetGlobalPose method!
         (use CManipulationObject for more convenient positioning of IV and collision models)
         Returns number of triangles used for building the collision model
         */
        //virtual int SetIVModel (SoNode *pModel, int id = 666);


        /*!
        Sets the position of the internal colModel data structure.
        No update of the pose of the IV model!
        */
        void setGlobalPose(const Eigen::Matrix4f& m) {};//avoiding unneccessary copies

        /*! Builds a (single) collision model, but assigns triangles of IVModels with the given IDs
         This can be useful to link a collision with a part of a scene
         Returns number of triangles used for building the collision model
         */
        //virtual int BuildColModel(std::map<SoNode*,int> &mIVIDMapping, std::vector<int> & vStoreIDs, SoSeparator *pAddIVModel);

    protected:

        //! delete all data
        virtual void destroyData();
    };

}

#endif
