
#include "PoseQualityExtendedManipulability.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <float.h>



using namespace std;

namespace VirtualRobot
{


    PoseQualityExtendedManipulability::PoseQualityExtendedManipulability(VirtualRobot::RobotNodeSetPtr rns, PoseQualityManipulability::ManipulabilityIndexType i)
        : PoseQualityManipulability(rns, i)
    {
        name = getTypeName();
        createCartDimPermutations(cartDimPermutations);
        considerObstacle = false;
        obstacleDir.setZero();
        obstacle_alpha = 1.0f;
        obstacle_beta = 1.0f;

        joints = rns->getAllRobotNodes();
        tmpJac.resize(6, joints.size());
    }

    PoseQualityExtendedManipulability::~PoseQualityExtendedManipulability()
    {
    }

    float PoseQualityExtendedManipulability::getPoseQuality()
    {
        return getPoseQuality(manipulabilityType, -1);
    }

    float PoseQualityExtendedManipulability::getPoseQuality(PoseQualityManipulability::ManipulabilityIndexType i, int considerFirstSV)
    {
        return getPoseQuality(jacobian, rns, i, considerFirstSV);
    }

    float PoseQualityExtendedManipulability::getPoseQuality(DifferentialIKPtr jac, RobotNodeSetPtr rns, PoseQualityManipulability::ManipulabilityIndexType i, int considerFirstSV)
    {
        //extManipData d;
        currentManipData.reset();
        if (!getDetailedAnalysis(jac, rns, currentManipData, considerFirstSV))
        {
            VR_ERROR << "ERROR" << endl;
            return 0;
        }

        switch (i)
        {
            case eMultiplySV:
            {
                return currentManipData.extManip_Volume;
            }
            break;

            case eMinMaxRatio:
            {
                return currentManipData.extManip_InvCondNumber;
            }
            break;

            default:
                VR_ERROR << "NYI..." << endl;
        }

        return 0;
    }

    float PoseQualityExtendedManipulability::getPoseQuality(const Eigen::VectorXf& direction)
    {
        return getManipulability(direction);
    }


    void PoseQualityExtendedManipulability::getQualityWeighting(float jv, float limitMin, float limitMax, float& storeWeightMin, float& storeWeightMax)
    {
        float w;

        if (fabs(limitMax - jv) > 1e-10 && fabs(jv - limitMin) > 1e-10)
        {
            w = ((limitMax - limitMin) * (limitMax - limitMin) * (2.0f * jv - limitMax - limitMin)) / (4.0f * (limitMax - jv) * (limitMax - jv) * (jv - limitMin) * (jv - limitMin));
        }
        else
        {
            w = 1e10;
        }

        // w is \delta H / \delta q, hence we are only interested in the direction -> fabs, a neg value wouldn't make sense in further processing
        w = fabs(w);

        if (boost::math::isinf(w) || boost::math::isinf(-w) || boost::math::isnan(w))
        {
            cout << "nan" << endl;
        }


        if (fabs(jv - limitMin) > fabs(jv - limitMax))
        {
            // next to upper limit
            storeWeightMin = 1.0f;
            storeWeightMax = 1.0f + w;
        }
        else
        {
            // next to lower limit
            storeWeightMin = 1.0f + w;
            storeWeightMax = 1.0f;
        }
    }

    void PoseQualityExtendedManipulability::getPenalizations(const std::vector<RobotNodePtr>& joints, Eigen::VectorXf& penLo, Eigen::VectorXf& penHi)
    {
        penLo.resize(joints.size());
        penHi.resize(joints.size());

        for (size_t i = 0; i < joints.size(); i++)
        {
            float l, h;
            getQualityWeighting(joints[i]->getJointValue(), joints[i]->getJointLimitLo(), joints[i]->getJointLimitHi(), l, h);
            penLo(i) = 1.0f / sqrtf(l);
            penHi(i) = 1.0f / sqrtf(h);
        }

        if (verbose)
        {
            cout << "PEN LO:";
            VirtualRobot::MathTools::print(penLo);
            cout << "PEN HI:";
            VirtualRobot::MathTools::print(penHi);
        }
    }

    void PoseQualityExtendedManipulability::getObstaclePenalizations(RobotNodeSetPtr rns, const Eigen::Vector3f& obstVect, const Eigen::MatrixXf& jac, Eigen::MatrixXf& penObstLo, Eigen::MatrixXf& penObstHi)
    {
        std::vector<RobotNodePtr> joints = rns->getAllRobotNodes();
        return getObstaclePenalizations(joints, obstVect, jac, penObstLo, penObstHi);
    }

    void PoseQualityExtendedManipulability::getObstaclePenalizations(const std::vector<RobotNodePtr>& joints, const Eigen::Vector3f& obstVect, const Eigen::MatrixXf& jac, Eigen::MatrixXf& penObstLo, Eigen::MatrixXf& penObstHi)
    {
        float scaleFactor = 0.001f;

        penObstLo.resize(jac.rows(), jac.cols());
        penObstHi.resize(jac.rows(), jac.cols());
        penObstLo.setZero();
        penObstHi.setZero();

        VR_ASSERT(jac.rows() == 6);

        float d = obstVect.norm();

        if (d < 1e-10)
        {
            return;
        }

        d *= scaleFactor; // m

        // compute \delta P / \delta d
        float p1 = (float) - (exp((double)(-obstacle_alpha * d)) * pow((double)d, (double)(-obstacle_beta)) * (double)(obstacle_beta * 1.0f / d + obstacle_alpha));

        if (verbose)
        {
            cout << "++++++++++++++++++++++++++++++" << endl;
            cout << "+++++++++++++++++++ d:" << d << endl;
            cout << "+++++++++++++++++++ p1:" << p1 << endl;
            cout << "++++++++++++++++++++++++++++++" << endl;
        }

        Eigen::VectorXf v2(6);
        v2.setZero();
        v2.block(0, 0, 3, 1) = obstVect;
        v2 *= scaleFactor;

        Eigen::Vector3f obstDirNorm = obstVect;
        obstDirNorm.normalize();

        // compute gradient
        Eigen::VectorXf penObst = (jac.transpose() * v2).transpose();

        if (verbose)
        {
            cout << "PEN OBST (gradient)1:";
            VirtualRobot::MathTools::print(penObst);
        }

        penObst = penObst / d;

        if (verbose)
        {

            cout << "PEN OBST (gradient)2:";
            VirtualRobot::MathTools::print(penObst);
        }

        penObst *= p1;

        if (verbose)
        {
            cout << "PEN OBST (gradient):";
            VirtualRobot::MathTools::print(penObst);
        }

        // compute pen factor from gradient
        for (size_t j = 0; j < joints.size(); j++)
        {
            penObst(j) = 1.0f / sqrtf(1.0f + fabs(penObst(j)));
        }

        if (verbose)
        {
            cout << "PEN OBST (pen factor):";
            VirtualRobot::MathTools::print(penObst);
        }

        penObstLo.setConstant(1.0f);
        penObstHi.setConstant(1.0f);

        for (int i = 0; i < 3; i++)
        {
            //fabs(obstDirNorm(i))*distQual + (1.0f - fabs(obstDirNorm(i)))*1.0f
            Eigen::VectorXf scPen = penObst;

            //for (int j=0;j<penObst.rows();j++)
            //  scPen(j) = fabs(obstDirNorm(i))*scPen(j) + (1.0f - fabs(obstDirNorm(i)))*1.0f;
            if (obstVect(i) > 0)
            {
                penObstHi.block(i, 0, 1, penObstHi.cols()) = scPen.transpose();
            }
            else
            {
                penObstLo.block(i, 0, 1, penObstLo.cols()) = scPen.transpose();
            }
        }

        if (verbose)
        {

            cout << "PEN OBST (penObstLo):\n" << penObstLo << endl;
            cout << "PEN OBST (penObstHi):\n" << penObstHi << endl;
        }

        if (verbose)
        {

            cout << "ObstVect: d=" << obstVect.norm() << ", v=";
            MathTools::print(obstVect);
        }
    }

    Eigen::MatrixXf PoseQualityExtendedManipulability::getJacobianWeightedObstacles(const Eigen::MatrixXf& jac, const std::vector<float>& directionVect, const Eigen::VectorXf& penLo, const Eigen::VectorXf& penHi, const Eigen::MatrixXf& penObstLo, const Eigen::MatrixXf& penObstHi)
    {
        Eigen::MatrixXf res = jac;

        VR_ASSERT(penHi.rows() == penLo.rows());
        VR_ASSERT(directionVect.size() == 6);

        for (int i = 0; i < jac.rows(); i++) // cart
        {
            for (int j = 0; j < jac.cols(); j++) // joints
            {
                bool posPen = true; // jl

                // check for neg quadrant
                if (directionVect[i] < 0)
                {
                    posPen = !posPen;
                    res(i, j) *= penObstLo(i, j);
                }
                else
                {
                    res(i, j) *= penObstHi(i, j);
                }

                // check for inverted movement of joint
                if (jac(i, j) < 0)
                {
                    posPen = !posPen;
                }

                if (posPen)
                {
                    res(i, j) *= penHi(j);
                }
                else
                {
                    res(i, j) *= penLo(j);
                }
            }
        }

        return res;
    }


    Eigen::MatrixXf PoseQualityExtendedManipulability::getJacobianWeighted(const Eigen::MatrixXf& jac, const std::vector<float>& directionVect, const Eigen::VectorXf& penLo, const Eigen::VectorXf& penHi)
    {
        Eigen::MatrixXf res = jac;

        VR_ASSERT(penHi.rows() == penLo.rows());
        VR_ASSERT(directionVect.size() == 6);

        for (int i = 0; i < jac.rows(); i++) // cart
        {
            for (int j = 0; j < jac.cols(); j++) // joints
            {
                bool posPen = true;

                // check for neg quadrant
                if (directionVect[i % 6] < 0)
                {
                    posPen = !posPen;
                }

                // check for inverted movement of joint
                if (jac(i, j) < 0)
                {
                    posPen = !posPen;
                }

                if (posPen)
                {
                    res(i, j) *= penHi(j);
                }
                else
                {
                    res(i, j) *= penLo(j);
                }
            }
        }

        return res;
    }

    bool PoseQualityExtendedManipulability::analyzeJacobian(const Eigen::MatrixXf& jac, Eigen::VectorXf& sv, Eigen::MatrixXf& singVectors, Eigen::MatrixXf& U, Eigen::MatrixXf& V, bool printInfo)
    {
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
        U = svd.matrixU();
        V = svd.matrixV();
        sv = svd.singularValues();

        float maxEV = sv(0);

        // scale Cartesian SingularValues to scaled influence
        singVectors = U;

        for (int i = 0; i < sv.rows(); i++)
        {
            /*Eigen::MatrixXf Bl = U.block(0, i, U.rows(), 1); // cart sing vectors are normed (norm == 1)
            Bl *= sv(i);// / maxEV;
            singVectors.block(0, i, U.rows(), 1) = Bl;*/

            singVectors.block(0, i, U.rows(), 1) = U.block(0, i, U.rows(), 1) * sv(i);
        }

        if (printInfo)
        {
            cout << "Sing Values:\n" << sv << endl;
            cout << "U:\n" << U << endl;
            cout << "Sing Vectors (scaled according to SingValues:\n" << singVectors << endl;
        }

        return true;
    }

    bool PoseQualityExtendedManipulability::getDetailedAnalysis(extManipData& storeData, bool (&dims)[6], int considerFirstSV)
    {
        return getDetailedAnalysis(jacobian, rns, storeData, dims, considerFirstSV);
    }

    bool PoseQualityExtendedManipulability::getDetailedAnalysis(DifferentialIKPtr jac, RobotNodeSetPtr rns, extManipData& storeData, bool (&dims)[6], int considerFirstSV)
    {
        jac->updateJacobianMatrix(tmpJac, rns->getTCP(), IKSolver::All);

        //Eigen::MatrixXf j = jac->getJacobianMatrix(rns->getTCP());
        return getDetailedAnalysis(tmpJac, joints, storeData, dims, considerFirstSV);
    }

    bool PoseQualityExtendedManipulability::getDetailedAnalysis(const Eigen::MatrixXf& jac, const std::vector<RobotNodePtr>& joints, extManipData& storeData, bool (&dims)[6], int considerFirstSV)
    {
        storeData.jac = jac;
        storeData.nrJoints = storeData.jac.cols();

        // penalize rotation
        storeData.jac.block(3, 0, 3, storeData.jac.cols()) *= penalizeRotationFactor;

        getPenalizations(joints, storeData.penLo, storeData.penHi);
        storeData.obstacles = considerObstacle;

        if (considerObstacle)
        {
            getObstaclePenalizations(joints, obstacleDir, storeData.jac, storeData.penObstLo, storeData.penObstHi);
        }

        //bool verbose = false;
        if (verbose)
        {
            cout << "considerFirstSV=" << considerFirstSV << endl;
            cout << "JAC:\n" << endl;
            MathTools::printMat(storeData.jac);
        }

        for (int i = 0; i < 64; i++)
        {
            if (considerObstacle)
            {
                storeData.jacPen[i] = getJacobianWeightedObstacles(storeData.jac, cartDimPermutations[i], storeData.penLo, storeData.penHi, storeData.penObstLo, storeData.penObstHi);
            }
            else
            {
                storeData.jacPen[i] = getJacobianWeighted(storeData.jac, cartDimPermutations[i], storeData.penLo, storeData.penHi);
            }

            for (int j = 0; j < 6; j++)
            {
                if (!dims[j])
                {
                    storeData.jacPen[i].block(j, 0, 1, storeData.jacPen[i].cols()).setConstant(0);
                }
            }

            if (verbose && i < 4)
            {
                cout << "JAC PEN:" << i << endl;
                MathTools::printMat(storeData.jacPen[i]);
            }

            analyzeJacobian(storeData.jacPen[i], storeData.sv[i], storeData.singVectors[i], storeData.U[i], storeData.V[i], (verbose && i < 4));
        }

        if (considerFirstSV <= 0 || considerFirstSV > storeData.sv[0].rows())
        {
            considerFirstSV = storeData.sv[0].rows();
        }

        storeData.consideFirstSV = considerFirstSV;
        float result_v = FLT_MAX;
        float result_c = FLT_MAX;
        float minSV = FLT_MAX;
        float maxSV = 0.0f;

        for (int k = 0; k < 64; k++)
        {
            Eigen::VectorXf sv = storeData.sv[k];//.block(0,k,svAll.rows(),1);
            //cout << "k: sv:";
            //VirtualRobot::MathTools::print(sv);
            float tmpRes = 0;

            // volume
            if (sv.rows() >= 1)
            {
                tmpRes = sv(0) * sv(0);

                for (int j = 1; j < considerFirstSV; j++)
                {
                    tmpRes *= sv(j) * sv(j);
                }

                tmpRes = sqrtf(tmpRes);
            }

            if (tmpRes < result_v)
            {
                result_v = tmpRes;
            }

            //cout << "## k:" << k << " -> tmpRes: " << tmpRes << ", result:" << result << endl;

            // cond numb
            if (sv.rows() >= 2)
            {
                float minSV_loc = FLT_MAX;
                float maxSV_loc = 0.0f;

                for (int j = 0; j < considerFirstSV; j++)
                {
                    if (sv(j) < minSV)
                    {
                        minSV = sv(j);
                    }

                    if (sv(j) > maxSV)
                    {
                        maxSV = sv(j);
                    }

                    if (sv(j) < minSV_loc)
                    {
                        minSV_loc = sv(j);
                    }

                    if (sv(j) > maxSV_loc)
                    {
                        maxSV_loc = sv(j);
                    }
                }

                if (maxSV_loc != 0)
                {
                    tmpRes = minSV_loc / maxSV_loc;
                }
                else
                {
                    tmpRes = 0;
                }

                if (tmpRes < result_c)
                {
                    result_c = tmpRes;

                    if (verbose)
                    {
                        cout << "## k:" << k << " -> minSV: " << minSV_loc << ", maxSV:" << maxSV_loc << endl;
                        cout << "## pen jac:\n" << endl;
                        MathTools::printMat(storeData.jacPen[k]);
                    }
                }
            }

        }

        storeData.extManip_InvCondNumber = result_c;
        storeData.extManip_Volume = result_v;

        storeData.minSV = minSV;
        storeData.maxSV = maxSV;

        return true;
    }

    bool PoseQualityExtendedManipulability::getDetailedAnalysis(extManipData& storeData, int considerFirstSV)
    {
        if (considerFirstSV <= 0 || considerFirstSV > 6)
        {
            considerFirstSV = 6;
        }

        bool dims[6];
        int i = 0;

        for (i = 0; i < 6; i++)
        {
            dims[i] = true;
        }

        return getDetailedAnalysis(jacobian, rns, storeData, dims, considerFirstSV);
    }

    bool PoseQualityExtendedManipulability::getDetailedAnalysis(DifferentialIKPtr jac, RobotNodeSetPtr rns, extManipData& storeData, int considerFirstSV)
    {
        if (considerFirstSV <= 0 || considerFirstSV > 6)
        {
            considerFirstSV = 6;
        }

        bool dims[6];
        int i = 0;

        for (i = 0; i < 6; i++)
        {
            dims[i] = true;
        }

        return getDetailedAnalysis(jac, rns, storeData, dims, considerFirstSV);
    }

    bool PoseQualityExtendedManipulability::createCartDimPermutations(std::vector < std::vector<float> >& storePerm)
    {
        for (int i = 0; i < 64; i++)
        {
            std::vector<float> dirVect;

            if (i % 2 == 0)
            {
                dirVect.push_back(1);
            }
            else
            {
                dirVect.push_back(-1);
            }

            if ((i / 2) % 2 == 0)
            {
                dirVect.push_back(1);
            }
            else
            {
                dirVect.push_back(-1);
            }

            if ((i / 4) % 2 == 0)
            {
                dirVect.push_back(1);
            }
            else
            {
                dirVect.push_back(-1);
            }

            if ((i / 8) % 2 == 0)
            {
                dirVect.push_back(1);
            }
            else
            {
                dirVect.push_back(-1);
            }

            if ((i / 16) % 2 == 0)
            {
                dirVect.push_back(1);
            }
            else
            {
                dirVect.push_back(-1);
            }

            if ((i / 32) % 2 == 0)
            {
                dirVect.push_back(1);
            }
            else
            {
                dirVect.push_back(-1);
            }

            storePerm.push_back(dirVect);
        }

        return true;
    }

    std::vector < std::vector<float> > PoseQualityExtendedManipulability::getPermutationVector()
    {
        return cartDimPermutations;
    }

    void PoseQualityExtendedManipulability::considerObstacles(bool enable, float alpha /*= 1.0f*/, float beta /*= 1.0f*/)
    {
        PoseQualityManipulability::considerObstacle = enable;
        obstacle_alpha = alpha;
        obstacle_beta = beta;
    }

    std::string PoseQualityExtendedManipulability::getTypeName()
    {
        return std::string("PoseQualityExtendedManipulability_JLWeightsQuadrants");
    }

    bool PoseQualityExtendedManipulability::consideringJointLimits()
    {
        return true;
    }

    float PoseQualityExtendedManipulability::getManipulability(const Eigen::VectorXf& direction, int considerFirstSV)
    {
        VR_ASSERT(direction.rows() == 3 || direction.rows() == 6);
        Eigen::VectorXf d(6);

        if (direction.rows() == 6)
        {
            d = direction;
        }
        else
        {
            d.setZero();
            d.segment(0, 3) = direction;
        }

        float result = 0;

        // jacobian (in global coord system!)
        VirtualRobot::DifferentialIKPtr jacobianGlobal(new VirtualRobot::DifferentialIK(rns));
        jacobianGlobal->convertModelScalingtoM(convertMMtoM);
        Eigen::MatrixXf jacGlobal = jacobianGlobal->getJacobianMatrix(rns->getTCP());
        // penalize rotation
        jacGlobal.block(3, 0, 3, jacGlobal.cols()) *= penalizeRotationFactor;


        Eigen::MatrixXf jacPen;
        // the joint limit penalizations in neg direction (n values)
        Eigen::VectorXf penLo;
        // the joint limit penalizations in pos direction (n values)
        Eigen::VectorXf penHi;
        // the obstacle penalizations in neg direction (6xn values)
        Eigen::MatrixXf penObstLo;
        // the joint limit penalizations in pos direction (6xn values)
        Eigen::MatrixXf penObstHi;

        std::vector<float> quadrant;

        for (int i = 0; i < 6; i++)
            if (d(i) < 0)
            {
                quadrant.push_back(-1.0f);
            }
            else
            {
                quadrant.push_back(1.0f);
            }

        std::vector<RobotNodePtr> joints = rns->getAllRobotNodes();
        getPenalizations(joints, penLo, penHi);

        if (considerObstacle)
        {
            Eigen::Vector3f zero = Eigen::Vector3f::Zero();
            Eigen::Vector3f pos = obstacleDir;
            zero = rns->getTCP()->toGlobalCoordinateSystemVec(zero);
            pos = rns->getTCP()->toGlobalCoordinateSystemVec(pos);

            // the orientation is skipped! (we have a position vector)
            Eigen::Vector3f obstVecGlobal;
            obstVecGlobal.block(0, 0, 3, 1) = pos - zero;
            getObstaclePenalizations(rns, obstVecGlobal, jacGlobal, penObstLo, penObstHi);
            jacPen = getJacobianWeightedObstacles(jacGlobal, quadrant, penLo, penHi, penObstLo, penObstHi);
        }
        else
        {
            jacPen = getJacobianWeighted(jacGlobal, quadrant, penLo, penHi);
        }

        // compute gradient
        Eigen::VectorXf gradient = (jacPen.transpose() * d).transpose();

        // quality
        result = gradient.norm();


        // analyze corresponding hyperoctant
        Eigen::MatrixXf U;
        Eigen::MatrixXf V;
        Eigen::VectorXf sv;
        Eigen::MatrixXf singVectors;
        analyzeJacobian(jacPen, sv, singVectors, U, V, false);

        if (considerFirstSV <= 0 || considerFirstSV > sv.rows())
        {
            considerFirstSV = sv.rows();
        }

        float result_v = FLT_MAX;
        float result_c = FLT_MAX;
        float minSV = FLT_MAX;
        float maxSV = 0.0f;
        float tmpRes = 0;

        // volume
        if (sv.rows() >= 1)
        {
            tmpRes = sv(0) * sv(0);

            for (int j = 1; j < considerFirstSV; j++)
            {
                tmpRes *= sv(j) * sv(j);
            }

            tmpRes = sqrtf(tmpRes);
        }

        if (tmpRes < result_v)
        {
            result_v = tmpRes;
        }

        // cond numb
        if (sv.rows() >= 2)
        {
            float minSV_loc = FLT_MAX;
            float maxSV_loc = 0.0f;

            for (int j = 0; j < considerFirstSV; j++)
            {
                if (sv(j) < minSV)
                {
                    minSV = sv(j);
                }

                if (sv(j) > maxSV)
                {
                    maxSV = sv(j);
                }

                if (sv(j) < minSV_loc)
                {
                    minSV_loc = sv(j);
                }

                if (sv(j) > maxSV_loc)
                {
                    maxSV_loc = sv(j);
                }
            }

            if (maxSV_loc != 0)
            {
                tmpRes = minSV_loc / maxSV_loc;
            }
            else
            {
                tmpRes = 0;
            }

            if (tmpRes < result_c)
            {
                result_c = tmpRes;

                if (verbose)
                {
                    cout << "##  -> minSV: " << minSV_loc << ", maxSV:" << maxSV_loc << endl;
                    cout << "## pen jac:\n" << jacPen << endl;
                }
            }
        }

        return result * result_c;
    }

}
