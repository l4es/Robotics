#ifndef __BULLET_LOGGER_H__
#define __BULLET_LOGGER_H__

#include <Eigen/Dense>
#include <VirtualRobot/RobotNodeSet.h>

#include "BulletEngine.h"
#include "BulletRobot.h"

/*
 * Logger for a BulletRobot that logs target/actual position/velocity to a file.
 */
namespace SimDynamics
{

    class SIMDYNAMICS_IMPORT_EXPORT BulletRobotLogger
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        BulletRobotLogger(BulletEnginePtr engine,
                          const BulletRobotPtr robot,
                          const VirtualRobot::RobotNodeSetPtr& jointNodes,
                          const VirtualRobot::RobotNodeSetPtr& bodyNodes)
            : robot(robot)
            , running(false)
            , jointNodes(jointNodes)
            , bodyNodes(bodyNodes)
            , max_samples(1024 * 1024)
            , timestamp(0.0f)
            , logPath("")
        {
            int dof = jointNodes->getSize();
            actualAngle.resize(dof);
            targetAngle.resize(dof);
            actualVelocity.resize(dof);
            targetVelocity.resize(dof);
            actualTorque.resize(dof);
            actualForces.resize(3, dof);
            engine->addExternalCallback(logCB, (void*) this);
        }

        ~BulletRobotLogger()
        {
            if (logPath.size() > 0)
            {
                writeToFile(logPath);
            }
        }

        void setLogPath(const std::string& path)
        {
            logPath = path;
        }

        void writeToFile(const std::string& path);
        void startLogging();
        void stopLogging();

    private:
        const BulletRobotPtr robot;
        bool running;
        VirtualRobot::RobotNodeSetPtr jointNodes;
        VirtualRobot::RobotNodeSetPtr bodyNodes;
        std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf> > targetAngleLog;
        std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf> > targetVelocityLog;
        std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf> > actualAngleLog;
        std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf> > actualVelocityLog;
        std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf> > actualJointTorquesLog;
        std::vector<Eigen::Matrix3Xf, Eigen::aligned_allocator<Eigen::Matrix3Xf> > actualJointForcesLog;
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > actualCoMLog;
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > actualCoMVelocityLog;
        std::vector<double> timestamps;
        int max_samples;
        double timestamp;
        std::string logPath;

        Eigen::VectorXf actualAngle;
        Eigen::VectorXf targetAngle;
        Eigen::VectorXf actualVelocity;
        Eigen::VectorXf targetVelocity;
        Eigen::VectorXf actualTorque;
        Eigen::Matrix3Xf actualForces;

        static void logCB(void* data, btScalar dt);
        void log(btScalar dt);
    };

    typedef boost::shared_ptr<BulletRobotLogger> BulletRobotLoggerPtr;

}

#endif
