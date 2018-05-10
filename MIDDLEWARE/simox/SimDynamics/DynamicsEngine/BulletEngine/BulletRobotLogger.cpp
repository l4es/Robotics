
#include <Eigen/Dense>
#include <fstream>
#include <VirtualRobot/RobotNodeSet.h>

#include "BulletRobotLogger.h"

using namespace SimDynamics;

void BulletRobotLogger::startLogging()
{
    running = true;
}

void BulletRobotLogger::stopLogging()
{
    running = false;
}

void BulletRobotLogger::writeToFile(const std::string& path)
{
    // Nothing to log
    if (targetAngleLog.size() == 0)
    {
        return;
    }

    std::ofstream output(path.c_str());

    int num_frames = timestamps.size();
    int num_dof = targetAngleLog[0].rows();

    output << "Timestamp" << ",";

    for (int dof = 0; dof < num_dof; dof++)
    {
        std::string name = (*jointNodes)[dof]->getName();
        output << "TargetAngle" << name << ",";
        output << "ActualAngle" << name << ",";
        output << "TargetVelocity" << name << ",";
        output << "ActualVelocity" << name << ",";
        output << "ActualTorque" << name << ",";
        output << "ActualForceX" << name << ",";
        output << "ActualForceY" << name << ",";
        output << "ActualForceZ" << name << ",";
    }

    output << "CoM X" << ",";
    output << "CoM Y" << ",";
    output << "CoM Z" << ",";
    output << "CoMVelocity X" << ",";
    output << "CoMVelocity Y" << ",";
    output << "CoMVelocity Z" << ",";
    output << std::endl;

    for (int frame = 0; frame < num_frames; frame++)
    {
        output << timestamps[frame] << ",";

        for (int dof = 0; dof < num_dof; dof++)
        {
            output << targetAngleLog[frame](dof) << ",";
            output << actualAngleLog[frame](dof) << ",";
            output << targetVelocityLog[frame](dof) << ",";
            output << actualVelocityLog[frame](dof) << ",";
            output << actualJointTorquesLog[frame](dof) << ",";
            output << actualJointForcesLog[frame](0, dof) << ",";
            output << actualJointForcesLog[frame](1, dof) << ",";
            output << actualJointForcesLog[frame](2, dof) << ",";
        }

        output << actualCoMLog[frame].x() << ",";
        output << actualCoMLog[frame].y() << ",";
        output << actualCoMLog[frame].z() << ",";
        output << actualCoMVelocityLog[frame].x() << ",";
        output << actualCoMVelocityLog[frame].y() << ",";
        output << actualCoMVelocityLog[frame].z() << ",";
        output << std::endl;
    }
}

void BulletRobotLogger::logCB(void* data, btScalar dt)
{
    BulletRobotLogger* logger = static_cast<BulletRobotLogger*>(data);
    logger->log(dt);
}

void BulletRobotLogger::log(btScalar dt)
{
    if (!running)
    {
        return;
    }

    if (int(actualAngleLog.size()) > max_samples)
    {
        std::cout << "Warning: Exceeded max_samples! Stopping logging." << std::endl;
        running = false;
        return;
    }

    int dof = jointNodes->getSize();
    Eigen::VectorXf actualAngle(dof);
    Eigen::VectorXf targetAngle(dof);
    Eigen::VectorXf actualVelocity(dof);
    Eigen::VectorXf targetVelocity(dof);
    Eigen::VectorXf actualTorque(dof);
    Eigen::Matrix3Xf actualForces(3, dof);

    for (unsigned int i = 0; i < jointNodes->getSize(); i++)
    {
        const VirtualRobot::RobotNodePtr& node = (*jointNodes)[i];
        actualAngle(i)    = float(robot->getJointAngle(node));
        // bullet changes the sign???
        actualVelocity(i) = float(-robot->getJointSpeed(node));
        targetAngle(i) = float(robot->getNodeTarget(node));
        targetVelocity(i) = float(robot->getJointTargetSpeed(node));
        actualTorque(i) = float(robot->getJointTorque(node));
        actualForces.col(i) = robot->getJointForces(node);
    }

    actualJointTorquesLog.push_back(actualTorque);
    actualJointForcesLog.push_back(actualForces);
    actualCoMLog.push_back(bodyNodes->getCoM());
    actualCoMVelocityLog.push_back(robot->getComVelocityGlobal(bodyNodes));
    actualVelocityLog.push_back(actualVelocity);
    actualAngleLog.push_back(actualAngle);
    targetAngleLog.push_back(targetAngle);
    targetVelocityLog.push_back(targetVelocity);
    timestamp += dt;
    timestamps.push_back(timestamp);
}

