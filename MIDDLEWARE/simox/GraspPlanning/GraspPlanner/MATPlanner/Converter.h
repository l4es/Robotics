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
* @package    GraspStudio
* @author     Markus Przybylski
* @copyright  2013 H2T,KIT
*             GNU Lesser General Public License
*
*/
#ifndef CONVERTER_H
#define CONVERTER_H

#include "../../GraspStudio.h"
#include <vector>
#include "MedialSphere.h"

#include "../../ExternalDependencies/powercrust/powercrust.h"

namespace GraspStudio
{

    class GRASPSTUDIO_IMPORT_EXPORT Converter
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Converter();

        static std::vector<MedialSpherePtr> convertPowerCrustPolarBallsToMedialSpheres(std::vector<GraspStudio::PowerCrust::PolarBall>& polarBalls);

        static MedialSpherePtr convertPowerCrustPolarBallToMedialSphere(GraspStudio::PowerCrust::PolarBall& polarBall);


    };
}

#endif // CONVERTER_H
