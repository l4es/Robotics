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
* @author     Peter Kaiser
* @copyright  2015 Peter Kaiser
*             GNU Lesser General Public License
*
*/

#include "StackedIK.h"

using namespace VirtualRobot;

namespace VirtualRobot
{
    StackedIK::StackedIK(VirtualRobot::RobotNodeSetPtr rns, JacobiProvider::InverseJacobiMethod method) :
        rns(rns),
        method(method)
    {
        VR_ASSERT(this->rns);
        verbose = false;
    }

    StackedIK::~StackedIK()
    {
    }

    void StackedIK::setVerbose(bool v)
    {
        verbose = v;
    }

    Eigen::VectorXf StackedIK::computeStep(const std::vector<JacobiProviderPtr> &jacDefs, float stepSize)
    {
        VR_ASSERT(jacDefs.size() > 0);

        // Get dimensions of stacked jacobian
        int total_rows = 0;
        int total_cols = jacDefs[0]->getJacobianMatrix().cols();
        for(size_t i = 0; i < jacDefs.size(); i++)
        {
            THROW_VR_EXCEPTION_IF(!jacDefs[0]->isInitialized(), "JacobiProvider is not initialized");

            Eigen::MatrixXf J = jacDefs[i]->getJacobianMatrix();
            Eigen::VectorXf e = jacDefs[i]->getError();

            THROW_VR_EXCEPTION_IF(J.cols() != total_cols, "Jacobian size mismatch");
            THROW_VR_EXCEPTION_IF(J.rows() != e.rows(), "Jacobian matrix and error vector don't match");

            total_rows += J.rows();
        }

        // Accumulate stacked jacobian
        Eigen::MatrixXf jacobian(total_rows, total_cols);
        Eigen::VectorXf error(total_rows);

        int current_row = 0;
        for (size_t i = 0; i < jacDefs.size(); i++)
        {
            Eigen::MatrixXf J = jacDefs[i]->getJacobianMatrix();
            jacobian.block(current_row, 0, J.rows(), J.cols()) = J;
            error.block(current_row, 0, J.rows(), 1) = jacDefs[i]->getError();

            current_row += J.rows();
        }

        // Invert stacked jacobian
        Eigen::MatrixXf J_inv;
        switch(method)
        {
            case JacobiProvider::eTranspose:
                J_inv = jacobian.transpose();
                break;

            case JacobiProvider::eSVD:
                J_inv = MathTools::getPseudoInverse(jacobian, 0.00001f);
                break;

            case JacobiProvider::eSVDDamped:
                J_inv = MathTools::getPseudoInverseDamped(jacobian, 1.0);
                break;
        }

        // Compute IK step
        return J_inv * error * stepSize;
    }

} // namespace VirtualRobot

