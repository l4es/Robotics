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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010, 2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

#ifndef _VirtualRobot_DHParameters_h_
#define _VirtualRobot_DHParameters_h_

#include "../VirtualRobotImportExport.h"
#include <Eigen/Core>

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT DHParameter
    {
    public:
        DHParameter()
        {
            _a = _d = _alpha = _theta = 0;
            _thetaRotation = Eigen::Matrix4f::Identity();
            _dTranslation = Eigen::Matrix4f::Identity();
            _aTranslation = Eigen::Matrix4f::Identity();
            _alphaRotation = Eigen::Matrix4f::Identity();
            isSet = false;
        };

        DHParameter(float theta, float d, float a, float alpha, bool isRadian)
        {
            setThetaRadian(theta, isRadian);
            setDInMM(d);
            setAInMM(a);
            setAlphaRadian(alpha, isRadian);
            isSet = true;
        };

        void setThetaRadian(float theta, bool isRadian)
        {
            _theta = theta;

            if (!isRadian)
            {
                _theta *= ((float)M_PI / 180.0f);
            }

            updateThetaRotation();
        }
        void setDInMM(float d)
        {
            _d = d;
            updateDTranslation();
        }
        void setAInMM(float a)
        {
            _a = a;
            updateATranslation();
        }
        void setAlphaRadian(float alpha, bool isRadian)
        {
            _alpha = alpha;

            if (!isRadian)
            {
                _alpha *= ((float)M_PI / 180.0f);
            }

            updateAlphaRotation();
        }

        float thetaRadian() const
        {
            return _theta;
        }
        float dMM() const
        {
            return _d;
        }
        float aMM() const
        {
            return _a;
        }
        float alphaRadian() const
        {
            return _alpha;
        }

        const Eigen::Matrix4f& thetaRotationRadian() const
        {
            return _thetaRotation;
        }

        const Eigen::Matrix4f& dTranslation() const
        {
            return _dTranslation;
        }

        const Eigen::Matrix4f& aTranslation() const
        {
            return _aTranslation;
        }

        const Eigen::Matrix4f& alphaRotationRadian() const
        {
            return _alphaRotation;
        }

        //! The complete transformation
        Eigen::Matrix4f transformation() const
        {
            return _thetaRotation * _dTranslation * _aTranslation * _alphaRotation;
        }



        bool isSet;
    protected:
        void updateTransformations()
        {
            updateThetaRotation();
            updateDTranslation();
            updateATranslation();
            updateAlphaRotation();
        }

        void updateThetaRotation()
        {
            _thetaRotation = Eigen::Matrix4f::Identity();
            _thetaRotation(0, 0) = cos(_theta);
            _thetaRotation(0, 1) = -sin(_theta);
            _thetaRotation(1, 0) = sin(_theta);
            _thetaRotation(1, 1) = cos(_theta);
        }

        void updateDTranslation()
        {
            _dTranslation = Eigen::Matrix4f::Identity();
            _dTranslation(2, 3) = _d;
        }

        void updateATranslation()
        {
            _aTranslation = Eigen::Matrix4f::Identity();
            _aTranslation(0, 3) = _a;
        }

        void updateAlphaRotation()
        {
            _alphaRotation = Eigen::Matrix4f::Identity();
            _alphaRotation(1, 1) = cos(_alpha);
            _alphaRotation(1, 2) = -sin(_alpha);
            _alphaRotation(2, 1) = sin(_alpha);
            _alphaRotation(2, 2) = cos(_alpha);
        }

        float _theta;
        float _d;
        float _a;
        float _alpha;

        Eigen::Matrix4f _thetaRotation;
        Eigen::Matrix4f _dTranslation;
        Eigen::Matrix4f _aTranslation;
        Eigen::Matrix4f _alphaRotation;
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_DHParameters_h_
