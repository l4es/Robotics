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
#ifndef _Saba_CSpaceNode_h
#define _Saba_CSpaceNode_h

#include "../Saba.h"
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Saba
{

    /*!
     *
     * \class CSpaceNode
     *
     * A CSpaceNode is used to store a configuration in cspace.
     *
     * @see CSpaceTree
     *
     */
    class SABA_IMPORT_EXPORT CSpaceNode
    {
    public:

        CSpaceNode();
        virtual ~CSpaceNode();

        Eigen::VectorXf configuration;          //!< the configuration vector
        int parentID;                           //!< id of parent (root node if < 0)
        unsigned int ID;                        //!< id of CSpaceNode

        bool allocated;

        // optional
        int status;
        float obstacleDistance;                 //!< work space distance to obstacles (-1 if the dist was not calculated)
        float dynDomRadius;                     //!< radius for this node (used by dynamic domain RRTs)
        std::vector<CSpaceNodePtr> children;    //!< children of this node
    };

} // nameaspace

#endif // _Saba_CSpaceNode_h
