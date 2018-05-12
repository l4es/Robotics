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
* @author     Peter Kaiser, Nikolaus Vahrenkamp
* @copyright  2011 Peter Kaiser, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_WorkspaceDataArray_h_
#define _VirtualRobot_WorkspaceDataArray_h_

#include "WorkspaceData.h"
#include "../VirtualRobotImportExport.h"
#include "../Compression/CompressionBZip2.h"





#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace VirtualRobot
{
    /*!
        Stores a 6-dimensional array for the vertex data of a workspace representation.
        Internally unsigned char data types are used (0...255)
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT WorkspaceDataArray : public WorkspaceData, public boost::enable_shared_from_this<WorkspaceDataArray>
    {
    public:
        /*!
            Constructor, fills the data with 0
        */
        WorkspaceDataArray(unsigned int size1, unsigned int size2, unsigned int size3,
                           unsigned int size4, unsigned int size5, unsigned int size6, bool adjustOnOverflow);

        //! Clone other data structure
        WorkspaceDataArray(WorkspaceDataArray* other);

        //! create Workspace out of file
        WorkspaceDataArray(std::ofstream& file);

        ~WorkspaceDataArray();

        //! Return the amount of data in bytes
        unsigned int getSizeTr() const;
        unsigned int getSizeRot() const;

        void setDatum(float x[], unsigned char value, const WorkspaceRepresentation* workspace);

        inline void setDatum(unsigned int x0, unsigned int x1, unsigned int x2,
                             unsigned int x3, unsigned int x4, unsigned int x5, unsigned char value);

        inline void setDatum(unsigned int x[6], unsigned char value);

        void setDatumCheckNeighbors(unsigned int x[6], unsigned char value, unsigned int neighborVoxels);

        void increaseDatum(float x[], const WorkspaceRepresentation* workspace);

        inline void increaseDatum(unsigned int x0, unsigned int x1, unsigned int x2,
                                  unsigned int x3, unsigned int x4, unsigned int x5);

        inline void increaseDatum(unsigned int x[6]);
        /*!
            Set rotation data for given x,y,z position.
        */
        void setDataRot(unsigned char* data, unsigned int x, unsigned int y, unsigned int z);
        /*!
            Get rotation data for given x,y,z position.
        */
        const unsigned char* getDataRot(unsigned int x, unsigned int y, unsigned int z);

        unsigned char get(float x[], const WorkspaceRepresentation* workspace);

        int getMaxSummedAngleReachablity();

        //! Simulates a multi-dimensional array access
        inline unsigned char get(unsigned int x0, unsigned int x1, unsigned int x2,
                                 unsigned int x3, unsigned int x4, unsigned int x5);

        //! Simulates a multi-dimensional array access
        inline unsigned char get(unsigned int x[6]);

        bool hasEntry(unsigned int x, unsigned int y, unsigned int z);

        // Set all entries to 0
        void clear();
        void binarize();

        void bisectData();

        unsigned int getSize(int dim)
        {
            return sizes[dim];
        }

        unsigned char** getRawData()
        {
            return data;
        }

        WorkspaceData* clone();

        bool save(std::ofstream& file);
    protected:

        void ensureData(unsigned int x, unsigned int y, unsigned int z);
        int sumAngleReachabilities(int x0, int x1, int x2);

        inline void getPos(unsigned int x0, unsigned int x1, unsigned int x2,
                           unsigned int x3, unsigned int x4, unsigned int x5 ,
                           unsigned int& storePosTr, unsigned int& storePosRot) const
        {
            storePosTr  = x0 * sizeTr0  + x1 * sizeTr1  + x2;
            storePosRot = x3 * sizeRot0 + x4 * sizeRot1 + x5;
        }

        inline void getPos(unsigned int x[6], unsigned int& storePosTr, unsigned int& storePosRot) const
        {
            storePosTr  = x[0] * sizeTr0  + x[1] * sizeTr1  + x[2];
            storePosRot = x[3] * sizeRot0 + x[4] * sizeRot1 + x[5];
        }

        unsigned int sizes[6];
        unsigned int sizeTr0, sizeTr1;
        unsigned int sizeRot0, sizeRot1;

        unsigned char** data;
    };



} // namespace VirtualRobot

#endif // _WorkspaceData_h_
