#include "Sampler.h"
#include "CSpace.h"

namespace Saba
{

    Sampler::Sampler(unsigned int dimension)
    {
        dimension = dimension;
        metricWeights.Ones(dimension);
    }

    Sampler::~Sampler()
    {
    }

    void Sampler::getUniformlyRandomConfig(Eigen::VectorXf& stroreConfig, CSpacePtr space)
    {
        for (unsigned int i = 0; i < dimension; i++)
        {
            stroreConfig[i] = space->getRandomConfig_UniformSampling(i);
        }
    }

    void Sampler::getNormalRandomConfig(Eigen::VectorXf& stroreConfig, const Eigen::VectorXf& mean, const Eigen::MatrixXf& variance)
    {
        //for(unsigned int i = 0; i < dimension; i++)
        //  pStoreConfig[i] = MathHelpers::getNormalRandom(mean[i], variance[i]);
        cout << "todo" << endl;

    }

    void Sampler::getNormalRandomConfig(Eigen::VectorXf& stroreConfig, const Eigen::VectorXf& mean, float variance)
    {
        //for(unsigned int i = 0; i < dimension; i++)
        //  pStoreConfig[i] = MathHelpers::getNormalRandom(mean[i], variance);
        cout << "todo" << endl;

    }

    void Sampler::enableMetricWeights(const Eigen::VectorXf& weights)
    {
        /*delete [] m_pMetricWeights;
        m_pMetricWeights = NULL;
        if (pWeights && dimension>0)
        {
            m_pMetricWeights = new float[dimension];
            memcpy(m_pMetricWeights,pWeights,sizeof(float)*dimension);
        }*/
        cout << "todo" << endl;
    }

    void Sampler::disableMetricWeights()
    {
        cout << "todo" << endl;
    }

}
