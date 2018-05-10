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
* @author     Nikolaus Vahrenkamp <vahrenkamp at users dot sf dot net>
* @copyright  2012 Nikolaus Vahrenkamp
* @license    http://www.gnu.org/licenses/gpl.txt
*             GNU General Public License
*/

#ifndef __PoseQualityExtManipulability_H_
#define __PoseQualityExtManipulability_H_


#include "PoseQualityManipulability.h"

namespace VirtualRobot
{

    /*!
        This pose quality measure is an extended approach based on Yoshikawa's manipulability measure.
        It considers joint limits and hence a detailed analysis of the constrained manipulability of a
        kinematic chain can be performed. Additionally self-distance and distances to obstacles can be considered.
        For details see:
        @INPROCEEDINGS{Vahrenkamp2012,
        author = {Nikolaus Vahrenkamp and Tamim Asfour and Giorgio Metta and Giulio Sandini and R\"udiger Dillmann},
        title = {Manipulability Analysis},
        booktitle = {Proceedings of IEEE-RAS International Conference on Humanoid Robots (Humanoids)},
        year = {2012}
        }

        For performance reasons this class is not thread safe and cannot be used from different threads in parallel.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT PoseQualityExtendedManipulability :  public VirtualRobot::PoseQualityManipulability
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor
            \param rns The kinematic chain for which the quality should be computed.
            \param i The method that should be used to compute the quality.
        */
        PoseQualityExtendedManipulability(VirtualRobot::RobotNodeSetPtr rns, PoseQualityManipulability::ManipulabilityIndexType i = PoseQualityManipulability::eMinMaxRatio);
        ~PoseQualityExtendedManipulability();

        virtual float getPoseQuality();

        float getPoseQuality(PoseQualityManipulability::ManipulabilityIndexType i, int considerFirstSV);

        /*!
            The quality is determined for a given Cartesian direction.
            The current configuration of the corresponding RNS is analyzed and the quality is returned.
            \param direction A 3d or 6d vector with the Cartesian direction to investigate.
        */
        virtual float getPoseQuality(const Eigen::VectorXf& direction);
        virtual float getManipulability(const Eigen::VectorXf& direction, int considerFirstSV = -1);


        struct extManipData
        {
            extManipData()
            {
                reset();
            }
            void reset()
            {
                extManip_InvCondNumber = -1.0f;
                extManip_Volume = -1.0f;
                minSV = -1.0f;
                maxSV = -1.0f;
                cartDim = 6;
                nrJoints = -1;
                consideFirstSV = -1;
            }
            // 2^6 = 64
            // for each hyperoctant, the SVD result is stored
            Eigen::MatrixXf U[64];
            Eigen::MatrixXf V[64];
            Eigen::VectorXf sv[64];
            Eigen::MatrixXf singVectors[64]; // the scaled singular vectors (U matrix, columns scaled according to sv)
            Eigen::MatrixXf jacPen[64];

            // the joint limit penalizations in neg direction (n values)
            Eigen::VectorXf penLo;
            // the joint limit penalizations in pos direction (n values)
            Eigen::VectorXf penHi;

            // the jacobian
            Eigen::MatrixXf jac;

            int nrJoints;
            int cartDim;

            float minSV;
            float maxSV;
            float extManip_InvCondNumber;
            float extManip_Volume;

            // considered obstacles
            bool obstacles;
            Eigen::VectorXf obstVect;
            // the obstacle penalizations in neg direction (6xn values)
            Eigen::MatrixXf penObstLo;
            // the joint limit penalizations in pos direction (6xn values)
            Eigen::MatrixXf penObstHi;

            // limit the analysis to the first sv (<=0 for complete ananlysis)
            int consideFirstSV;
        };

        bool getDetailedAnalysis(extManipData& storeData, int considerFirstSV = 0);
        bool getDetailedAnalysis(extManipData& storeData, bool (&dims)[6], int considerFirstSV = 0);
        std::vector < std::vector<float> > getPermutationVector();

        /*!
            If enabled (standard = disabled), you must take care of setting the obstacle vector before computing the manipulability. (see PoseQualityMeasurement::setObstacleDistanceVector)
        */
        void considerObstacles(bool enable, float alpha  = 40.0f, float beta = 1.0f);

        static std::string getTypeName();

        //! Indicates if joint limits are considered.
        virtual bool consideringJointLimits();


    protected:

        bool getDetailedAnalysis(DifferentialIKPtr jacobian, RobotNodeSetPtr rns, extManipData& storeData, int considerFirstSV = 0);
        bool getDetailedAnalysis(DifferentialIKPtr jacobian, RobotNodeSetPtr rns, extManipData& storeData, bool (&dims)[6], int considerFirstSV = 0);
        bool getDetailedAnalysis(const Eigen::MatrixXf& jac, const std::vector<RobotNodePtr>& joints, extManipData& storeData, bool (&dims)[6], int considerFirstSV = 0);

        float getPoseQuality(DifferentialIKPtr jacobian,  RobotNodeSetPtr rns, PoseQualityManipulability::ManipulabilityIndexType i, int considerFirstSV);


        bool createCartDimPermutations(std::vector < std::vector<float> >& storePerm);

        /*!
            Get the weighting for jv with limits.
            The weighting is computed as follows:
            w = ( (limitMax - limitMin)^2 (2jv - limitMax - limitMin) ) / ( 4(limitMax - jv)^2 (jv - limitMin)^2 )
            In case jv is on the lower half of the joint range -> storeWeightMin = 1+w; storeWeightMax = 1
            In case jv is on the upper half of the joint range -> storeWeightMin = 1;   storeWeightMax = 1+w
        */
        void getQualityWeighting(float jv, float limitMin, float limitMax, float& storeWeightMin, float& storeWeightMax);

        //! Compute weightings w and store 1/sqrt(w)
        void getPenalizations(const std::vector<RobotNodePtr>& joints, Eigen::VectorXf& penLo, Eigen::VectorXf& penHi);


        Eigen::MatrixXf getJacobianWeighted(const Eigen::MatrixXf& jac, const std::vector<float>& directionVect, const Eigen::VectorXf& penLo, const Eigen::VectorXf& penHi);

        /*!
            Perform SVD on jac.
            \param sv Singular vectors are stored in sv
            \param singVectors The scaled sing vectors are stored here (first one: length 1, the following are scaled according to their corresponding sv ratio)
            \param U The U matrix is stored here
            \param V The V matrix is stored here
        */
        bool analyzeJacobian(const Eigen::MatrixXf& jac, Eigen::VectorXf& sv, Eigen::MatrixXf& singVectors, Eigen::MatrixXf& U, Eigen::MatrixXf& V, bool printInfo = false);
        void getObstaclePenalizations(RobotNodeSetPtr rns, const Eigen::Vector3f& obstVect, const Eigen::MatrixXf& jac, Eigen::MatrixXf& penObstLo, Eigen::MatrixXf& penObstHi);
        void getObstaclePenalizations(const std::vector<RobotNodePtr>& joints, const Eigen::Vector3f& obstVect, const Eigen::MatrixXf& jac, Eigen::MatrixXf& penObstLo, Eigen::MatrixXf& penObstHi);
        Eigen::MatrixXf getJacobianWeightedObstacles(const Eigen::MatrixXf& jac, const std::vector<float>& directionVect, const Eigen::VectorXf& penLo, const Eigen::VectorXf& penHi, const Eigen::MatrixXf& penObstLo, const Eigen::MatrixXf& penObstHi);

        std::vector< std::vector<float> > cartDimPermutations;

        float obstacle_alpha, obstacle_beta;

        extManipData currentManipData;
        Eigen::MatrixXf tmpJac;
        std::vector<RobotNodePtr> joints;
    };

    typedef boost::shared_ptr<PoseQualityExtendedManipulability> PoseQualityExtendedManipulabilityPtr;

}

#endif
