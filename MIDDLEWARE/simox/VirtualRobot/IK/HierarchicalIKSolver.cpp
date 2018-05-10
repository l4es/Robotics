#include "HierarchicalIKSolver.h"
//#define DEBUG

HierarchicalIKSolver::HierarchicalIKSolver(RobotNodeSetPtr allRobotNodes) : HierarchicalIK(allRobotNodes)
{
    //rns = allRobotNodes;
}

bool HierarchicalIKSolver::solveIK( float stepSize, float minChange, int maxSteps)
{
    return computeSteps(stepSize,minChange,maxSteps);
}

bool HierarchicalIKSolver::computeSteps(float stepSize, float minChange, int maxSteps) {
    //std::vector<RobotNodePtr> rn = rns->getAllRobotNodes();
    //RobotPtr robot = rns->getRobot();
    //std::vector<float> jv(rns->getSize(),0.0f);
    int step = 0;
    checkTolerances();
    //float lastDist = FLT_MAX;

    while (step < maxSteps) {
        Eigen::VectorXf delta = computeStep(jacobies, stepSize);

        if(!MathTools::isValid(delta)) {
#ifdef DEBUG
            VR_INFO << "Singular Jacobian" << endl;
#endif
            return false;
        }

        Eigen::VectorXf jv(delta.rows());
        rns->getJointValues(jv);
        jv += delta;
        rns->setJointValues(jv);

        if(checkTolerances()) {
#ifdef DEBUG
            VR_INFO << "Tolerances ok, loop:" << step << endl;
#endif
            return true;
        }

        if (delta.norm()<minChange)
        {
#ifdef DEBUG
            VR_INFO << "Could not improve result any more (dTheta.norm()=" << delta.norm() << "), loop:" << step << endl;
#endif
            return false;
        }

        step++;
    }
    return false;
}

bool HierarchicalIKSolver::checkTolerances() {
    for(int i = 0; i < jacobies.size(); i++) {
        if (!jacobies[i]->checkTolerances())
            return false;
    }
    return true;
}

void HierarchicalIKSolver::addIK(JacobiProviderPtr jacProvider)
{
    jacobies.push_back(jacProvider);
}

void HierarchicalIKSolver::clearIKs() {
    jacobies.clear();
}
