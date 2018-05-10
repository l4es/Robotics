#include "HierarchicalIK.h"
using namespace VirtualRobot;
using namespace std;


namespace VirtualRobot
{


    HierarchicalIK::HierarchicalIK(VirtualRobot::RobotNodeSetPtr rns)
        : rns(rns)
    {
        VR_ASSERT(this->rns);
        verbose = false;
    }

    HierarchicalIK::~HierarchicalIK()
    {

    }

    void HierarchicalIK::setVerbose(bool v)
    {
        verbose = v;
    }

    Eigen::VectorXf HierarchicalIK::computeStep(const std::vector<JacobiProviderPtr> &jacDefs, float stepSize)
    {
        VR_ASSERT(jacDefs.size() > 0 && jacDefs[0] && jacDefs[0]->getRobotNodeSet());

        if (verbose)
        {
            VR_INFO << "Compute Step" << endl;
        }

        int ndof = jacDefs[0]->getRobotNodeSet()->getSize();
        Eigen::VectorXf result(ndof);
        result.setZero();
        std::vector<Eigen::MatrixXf> jacobies;
        std::vector<Eigen::MatrixXf> invJacobies;
        std::vector<Eigen::VectorXf> errors;

        for (size_t i = 0; i < jacDefs.size(); i++)
        {
            THROW_VR_EXCEPTION_IF(!jacDefs[i]->isInitialized(), "JacobiProvider is not initialized...");
            Eigen::MatrixXf j = jacDefs[i]->getJacobianMatrix();// jacDefs[i].tcp);
            jacobies.push_back(j);

            if (verbose)
            {
                VR_INFO << "Jacoby " << i << ":\n" << j << endl;
            }

            j = jacDefs[i]->computePseudoInverseJacobianMatrix(j);// jacDefs[i].tcp);
            invJacobies.push_back(j);

            errors.push_back(jacDefs[i]->getError());

            if (verbose)
            {
                VR_INFO << "Inv Jacoby " << i << ":\n" << j << endl;
            }

            if (jacobies[i].cols() != ndof)
            {
                THROW_VR_EXCEPTION("Expecting " << ndof << " DOFs, but Jacobi " << i << " has " << jacobies[i].cols() << " columns ");
            }

            if (jacobies[i].rows() != errors[i].rows())
            {
                THROW_VR_EXCEPTION("Jacobi " << i << " has " << jacobies[i].rows() << " rows, but delta has " << jacDefs[i]->getError().rows() << " rows ");
            }
        }

        // generate hierarchical gradient descent

        // init with first jacobi
        Eigen::MatrixXf J_i = jacobies[0];
        Eigen::MatrixXf Jinv_i = invJacobies[0];


        Eigen::MatrixXf Jinv_i_min1;
        Eigen::VectorXf result_i = Jinv_i * errors[0] * stepSize;

        if (verbose)
        {
            VR_INFO << "result_i 0:\n" << result_i << endl;
        }

        Eigen::VectorXf result_i_min1;
        Eigen::MatrixXf PA_i_min1;
        Eigen::MatrixXf JA_i_min1;
        Eigen::MatrixXf JAinv_i_min1;
        Eigen::MatrixXf id_ndof(ndof, ndof);
        id_ndof.setIdentity();

        int accRowCount = J_i.rows();

        float pinvtoler = 0.00001f;

        for (size_t i = 1; i < jacobies.size(); i++)
        {
            result_i_min1 = result_i;
            Jinv_i_min1 = Jinv_i;
            Jinv_i = invJacobies[i];
            J_i = jacobies[i];
            JA_i_min1.resize(accRowCount, ndof);
            int rowPos = 0;

            for (size_t j = 0; j < i; j++)
            {
                //JA_i_min1.block(colPos,0,jacobies[j].cols(),jacobies[j].rows()) = jacobies[j];
                JA_i_min1.block(rowPos, 0, jacobies[j].rows(), jacobies[j].cols()) = jacobies[j];
                rowPos += jacobies[j].rows();
            }
            if (verbose)
            {
                VR_INFO << "JA_i_min1 " << i << ":\n" << endl << JA_i_min1 << endl;
            }

            JAinv_i_min1 = MathTools::getPseudoInverse(JA_i_min1, pinvtoler);
            if (verbose)
            {
                VR_INFO << "JAinv_i_min1 " << i << ":\n" << endl << JAinv_i_min1 << endl;
            }
            PA_i_min1 = id_ndof - JAinv_i_min1 * JA_i_min1;
            if (verbose)
            {
                VR_INFO << "PA_i_min1 " << i << ":\n" << endl << PA_i_min1 << endl;
            }
            Eigen::MatrixXf J_tilde_i = J_i * PA_i_min1;
            if (verbose)
            {
                VR_INFO << "J_tilde_i " << i << ":\n" << endl << J_tilde_i << endl;
            }
            Eigen::MatrixXf Jinv_tilde_i = MathTools::getPseudoInverse(J_tilde_i, pinvtoler);
            if (verbose)
            {
                VR_INFO << "Jinv_tilde_i " << i << ":\n" << endl << Jinv_tilde_i << endl;
            }
            if (verbose)
            {
                VR_INFO << "jacDefs[i]->getError() " << i << ":\n" << endl << errors[i].transpose() << endl;
            }

            result_i = result_i_min1 + Jinv_tilde_i * (errors[i] * stepSize - J_i * result_i_min1);

            if (verbose)
            {
                VR_INFO << "result_i " << i << ":\n" << result_i << endl;
            }

            accRowCount += J_i.rows();
        }

        return result_i;

    }

} // namespace VirtualRobot
