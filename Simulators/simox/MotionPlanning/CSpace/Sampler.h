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
* @package    Saba
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _Saba_sampler_h
#define _Saba_sampler_h

#include "../Saba.h"
#include "../CSpace/CSpace.h"
#include <vector>

namespace Saba
{

    /*!
    *
    * \brief An interface class for custom sample algorithms
    *
    */
    class SABA_IMPORT_EXPORT Sampler
    {
    public:
        Sampler(unsigned int dimension);
        virtual ~Sampler();

        virtual void sample(Eigen::VectorXf& stroreConfig, CSpacePtr space) = 0;
        //virtual void sample(std::vector<float> &storeConfig, CSpacePtr cspace) = 0;

        /*!
            Enable metric weighting. This can be useful for different variance in each dimension.
            Standard: disabled
        */
        virtual void enableMetricWeights(const Eigen::VectorXf& weights);
        virtual void disableMetricWeights();

    protected:
        //! Returns a uniformly generated random configuration with nDimension components
        void getUniformlyRandomConfig(Eigen::VectorXf& stroreConfig, CSpacePtr space);

        /*! Returns a normal distributed random configuration with nDimension components.
            Note that we assume the covariance-matrix to be a diagonal matrix. This means,
            that the components of the configuration are uncorrelated.
            The result value is not checked against any boundaries of the configuration space.
        */
        void getNormalRandomConfig(Eigen::VectorXf& stroreConfig, const Eigen::VectorXf& mean, const Eigen::MatrixXf& variance);

        /*! Returns a normal distributed random configuration with nDimension components.
            This is a convenience function in case you want to apply the same variance in
            every dimension.
        */
        void getNormalRandomConfig(Eigen::VectorXf& stroreConfig, const Eigen::VectorXf& mean, float variance);

        unsigned int dimension;
        Eigen::VectorXf metricWeights;
    };

}

#endif // _Saba_sampler_h
