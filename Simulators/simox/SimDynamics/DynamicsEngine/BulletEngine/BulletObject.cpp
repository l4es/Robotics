#include "BulletObject.h"
#include "BulletEngine.h"
#include "../../DynamicsWorld.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/SceneObject.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/Primitive.h>


#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>

//#define DEBUG_FIXED_OBJECTS
//#define USE_BULLET_GENERIC_6DOF_CONSTRAINT

#include <typeinfo>

using namespace VirtualRobot;

namespace SimDynamics
{

    BulletObject::BulletObject(VirtualRobot::SceneObjectPtr o)
        : DynamicsObject(o)
    {
        btScalar interatiaFactor = btScalar(1.0);
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
        interatiaFactor = 5.0f;
#endif

        btMargin = (btScalar)(0.000001);
        com.setZero();
        THROW_VR_EXCEPTION_IF(!o, "NULL object");
        CollisionModelPtr colModel = o->getCollisionModel();

        if (!colModel)
        {
            VR_WARNING << "Building empty collision shape for object " << o->getName() << endl;
            collisionShape.reset(new btEmptyShape());
        }
        else
        {
            THROW_VR_EXCEPTION_IF(!colModel, "No CollisionModel, could not create dynamics model...");

            if (o->getName() != "Floor")
            {
                std::vector<Primitive::PrimitivePtr> primitives = colModel->getVisualization()->primitives;

                /*if (primitives.size() == 1 && primitives[0]->transform.isIdentity())
                {
                    collisionShape.reset(getShapeFromPrimitive(primitives[0]));
                } else
                */
                if (primitives.size() > 0)
                {
                    btCompoundShape* compoundShape = new btCompoundShape(true);
                    std::vector<Primitive::PrimitivePtr>::const_iterator it;
                    Eigen::Matrix4f currentTransform = Eigen::Matrix4f::Identity();

                    Eigen::Matrix4f localComTransform;
                    localComTransform.setIdentity();
                    localComTransform.block(0,3,3,1) = -o->getCoMLocal();
                    //cout << "localComTransform:\n" << localComTransform;


                    for (it = primitives.begin(); it != primitives.end(); it++)
                    {
                        //currentTransform *= (*it)->transform;
                        currentTransform = localComTransform*(*it)->transform;
                        compoundShape->addChildShape(BulletEngine::getPoseBullet(currentTransform), getShapeFromPrimitive(*it));
                    }

                    collisionShape.reset(compoundShape);
                }
                else
                {
                    TriMeshModelPtr trimesh;
                    trimesh = colModel->getTriMeshModel();
                    THROW_VR_EXCEPTION_IF((!trimesh || trimesh->faces.size() == 0) , "No TriMeshModel, could not create dynamics model...");
                    collisionShape.reset(createConvexHullShape(trimesh));
                }
            }
            else
            {
                // the floor needs a primitive shape, works better with collision handling
                VirtualRobot::BoundingBox bb = colModel->getBoundingBox();
                Eigen::Vector3f half_size = (bb.getMax() - bb.getMin()) / 1000.0 / 2;
                btBoxShape* box = new btBoxShape(btVector3(half_size.x(), half_size.y(), half_size.z()));
                collisionShape.reset(box);
            }
        }

        //collisionShape->setMargin(btMargin);

        btScalar mass = o->getMass();
        btVector3 localInertia;

        if (mass <= 0 && (o->getSimulationType() == VirtualRobot::SceneObject::Physics::eDynamic || o->getSimulationType() == VirtualRobot::SceneObject::Physics::eUnknown))
        {
            //THROW_VR_EXCEPTION ("mass == 0 -> SimulationType must not be eDynamic! ");
            mass = btScalar(1.0f); // give object a dummy mass
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
            mass = btScalar(1.0f);
#endif

            //type = eKinematic;
            if (colModel)
            {
                VR_WARNING << "Object:" << o->getName() << ": mass == 0 -> SimulationType must not be eDynamic! Setting mass to 1" << endl;
            }
        }

#ifdef DEBUG_FIXED_OBJECTS
        cout << "TEST" << endl;
        mass = 0;
        localInertia.setValue(0.0f, 0.0f, 0.0f);
#else

        if (o->getSimulationType() != VirtualRobot::SceneObject::Physics::eDynamic && o->getSimulationType() != VirtualRobot::SceneObject::Physics::eUnknown)
        {
            mass = 0;
            localInertia.setValue(0.0f, 0.0f, 0.0f);
        }
        else
        {
            if (colModel)
            {
                collisionShape->calculateLocalInertia(mass, localInertia);
            }
            else
#ifndef USE_BULLET_GENERIC_6DOF_CONSTRAINT
                localInertia.setValue(btScalar(1), btScalar(1), btScalar(1)); // give Object a dummy inertia matrix

#else
                localInertia.setValue(btScalar(1), btScalar(1), btScalar(1)); // give Object a dummy inertia matrix
#endif
        }

#endif
        localInertia *= interatiaFactor;
        motionState = new SimoxMotionState(o);
        btRigidBody::btRigidBodyConstructionInfo btRBInfo(mass, motionState, collisionShape.get(), localInertia);
        //btRBInfo.m_additionalDamping = true;

        rigidBody.reset(new btRigidBody(btRBInfo));
        rigidBody->setUserPointer((void*)(this));
#if 0
        rigidBody->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
        cout << "TEST3" << endl;
#endif

        setPoseIntern(o->getGlobalPose());
    }

    BulletObject::~BulletObject()
    {
        rigidBody.reset();
        delete motionState;
    }


    btCollisionShape* BulletObject::getShapeFromPrimitive(VirtualRobot::Primitive::PrimitivePtr primitive)
    {
        btCollisionShape* result;

        if (primitive->type == Primitive::Box::TYPE)
        {
            Primitive::Box* box = boost::dynamic_pointer_cast<Primitive::Box>(primitive).get();
            // w/h/d have to be halved
            btBoxShape* boxShape = new btBoxShape(btVector3(box->width / 2000.f, box->height / 2000.f, box->depth / 2000.f));
            result = boxShape;
        }
        else if (primitive->type == Primitive::Sphere::TYPE)
        {
            Primitive::Sphere* sphere = boost::dynamic_pointer_cast<Primitive::Sphere>(primitive).get();
            btSphereShape* sphereShape = new btSphereShape(btScalar(sphere->radius / 1000.f));
            result = sphereShape;
        }
        else if (primitive->type == Primitive::Cylinder::TYPE)
        {
            Primitive::Cylinder* cyl = boost::dynamic_pointer_cast<Primitive::Cylinder>(primitive).get();
            btCylinderShape* cylShape = new btCylinderShape(btVector3(cyl->radius / 1000.f, cyl->height / 1000.f, cyl->radius / 1000.f));
            result = cylShape;
        }
        else
        {
            VR_ERROR << "Unsupported shape type " << primitive->type << std::endl;
            result = new btEmptyShape();
        }

        return result;
    }

    btConvexHullShape* BulletObject::createConvexHullShape(VirtualRobot::TriMeshModelPtr trimesh)
    {
        VR_ASSERT(trimesh);
        // create triangle shape
        btTriangleMesh* btTrimesh = new btTriangleMesh();

        //com = trimesh->getCOM();

        Eigen::Matrix4f comLoc;
        comLoc.setIdentity();
        comLoc.block(0, 3, 3, 1) = sceneObject->getCoMGlobal();
        comLoc = (sceneObject->getGlobalPose().inverse() * comLoc);
        com = comLoc.block(0, 3, 3, 1);

        double sc = 1.0f;

        if (DynamicsWorld::convertMM2M)
        {
            sc = 0.001f;
        }

        for (size_t i = 0; i < trimesh->faces.size(); i++)
        {
            btVector3 v1(btScalar((trimesh->vertices[trimesh->faces[i].id1][0] - com[0])*sc), btScalar((trimesh->vertices[trimesh->faces[i].id1][1] - com[1])*sc), btScalar((trimesh->vertices[trimesh->faces[i].id1][2] - com[2])*sc));
            btVector3 v2(btScalar((trimesh->vertices[trimesh->faces[i].id2][0] - com[0])*sc), btScalar((trimesh->vertices[trimesh->faces[i].id2][1] - com[1])*sc), btScalar((trimesh->vertices[trimesh->faces[i].id2][2] - com[2])*sc));
            btVector3 v3(btScalar((trimesh->vertices[trimesh->faces[i].id3][0] - com[0])*sc), btScalar((trimesh->vertices[trimesh->faces[i].id3][1] - com[1])*sc), btScalar((trimesh->vertices[trimesh->faces[i].id3][2] - com[2])*sc));
            btTrimesh->addTriangle(v1, v2, v3);
        }

        // convert COM to visualization frame (->no, the trimesh points are given in local visu frame!)
        /*Eigen::Matrix4f comLoc;
        comLoc.setIdentity();
        comLoc.block(0,3,3,1) = com;
        Eigen::Matrix4f comConv = sceneObject->getGlobalPoseVisualization() * comLoc;
        com = comConv.block(0,3,3,1);*/

        // build convex hull
        boost::shared_ptr<btConvexShape> btConvexShape(new btConvexTriangleMeshShape(btTrimesh));
        btConvexShape->setMargin(btMargin);

        boost::shared_ptr<btShapeHull> btHull(new btShapeHull(btConvexShape.get()));
        btHull->buildHull(btMargin);
        btConvexHullShape* btConvex = new btConvexHullShape();
        btConvex->setLocalScaling(btVector3(1, 1, 1));

        for (int i = 0; i < btHull->numVertices(); i++)
        {
            btConvex->addPoint(btHull->getVertexPointer()[i]);
        }

        btConvex->setMargin(btMargin);

        // trimesh not needed any more
        delete btTrimesh;
        return btConvex;
    }

    boost::shared_ptr<btRigidBody> BulletObject::getRigidBody()
    {
        return rigidBody;
    }

    void BulletObject::setPosition(const Eigen::Vector3f& posMM)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block(0, 3, 3, 1) = posMM;
        setPose(pose);
    }

    void BulletObject::setPoseIntern(const Eigen::Matrix4f& pose)
    {
        MutexLockPtr lock = getScopedLock();
        /* convert to local coord system, apply comoffset and convert back*/
        Eigen::Matrix4f poseLocal = sceneObject->getGlobalPose().inverse() * pose;
        poseLocal.block(0, 3, 3, 1) += com;
        Eigen::Matrix4f poseGlobal = sceneObject->getGlobalPose() * poseLocal;
        this->rigidBody->setWorldTransform(BulletEngine::getPoseBullet(poseGlobal));

        // notify motionState -> not needed, automatically done
        //motionState->setGlobalPose(pose);
    }

    void BulletObject::setPose(const Eigen::Matrix4f& pose)
    {
        MutexLockPtr lock = getScopedLock();
        DynamicsObject::setPose(pose);
        setPoseIntern(pose);
    }

    Eigen::Vector3f BulletObject::getLinearVelocity()
    {
        MutexLockPtr lock = getScopedLock();

        if (!rigidBody)
        {
            return Eigen::Vector3f::Zero();
        }

        return (BulletEngine::getVecEigen(rigidBody->getLinearVelocity()));
    }

    Eigen::Vector3f BulletObject::getAngularVelocity()
    {
        MutexLockPtr lock = getScopedLock();

        if (!rigidBody)
        {
            return Eigen::Vector3f::Zero();
        }

        return (BulletEngine::getVecEigen(rigidBody->getAngularVelocity(), false));
    }

    void BulletObject::setLinearVelocity(const Eigen::Vector3f& vel)
    {
        MutexLockPtr lock = getScopedLock();

        if (!rigidBody)
        {
            return;
        }

        btVector3 btVel = BulletEngine::getVecBullet(vel, false);
        rigidBody->setLinearVelocity(btVel);
    }

    void BulletObject::setAngularVelocity(const Eigen::Vector3f& vel)
    {
        MutexLockPtr lock = getScopedLock();

        if (!rigidBody)
        {
            return;
        }

        btVector3 btVel = BulletEngine::getVecBullet(vel, false);
        rigidBody->setAngularVelocity(btVel);
    }

    Eigen::Matrix4f BulletObject::getComGlobal()
    {
        MutexLockPtr lock = getScopedLock();
        btTransform tr;
        motionState->getWorldTransform(tr);
        return BulletEngine::getPoseEigen(tr);
    }

    void BulletObject::applyForce(const Eigen::Vector3f& force)
    {
        MutexLockPtr lock = getScopedLock();

        if (!rigidBody)
        {
            return;
        }

        btVector3 btVel = BulletEngine::getVecBullet(force, false);
        rigidBody->applyCentralForce(btVel);
    }

    void BulletObject::applyTorque(const Eigen::Vector3f& torque)
    {
        MutexLockPtr lock = getScopedLock();

        if (!rigidBody)
        {
            return;
        }

        btVector3 btVel = BulletEngine::getVecBullet(torque, false);
        rigidBody->applyTorque(btVel);
    }

    void BulletObject::setSimType(VirtualRobot::SceneObject::Physics::SimulationType s)
    {
        btVector3 localInertia;
        localInertia.setZero();
        CollisionModelPtr colModel = sceneObject->getCollisionModel();

        switch (s)
        {
            case VirtualRobot::SceneObject::Physics::eDynamic:
            case VirtualRobot::SceneObject::Physics::eUnknown:
                if (colModel)
                {
                    collisionShape->calculateLocalInertia(sceneObject->getMass(), localInertia);
                }
                else
                {
                    localInertia.setValue(btScalar(1), btScalar(1), btScalar(1));    // give Object a dummy inertia matrix
                }

                rigidBody->setMassProps(sceneObject->getMass(), localInertia);
                break;

            default:
                // static
                rigidBody->setMassProps(0, localInertia);
        }

        DynamicsObject::setSimType(s);
    }


} // namespace VirtualRobot
