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
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_VoxelTree6DElement_h_
#define _VirtualRobot_VoxelTree6DElement_h_

#include "../VirtualRobotImportExport.h"

#include <string>
#include <vector>


namespace VirtualRobot
{

    /*!
        A template definition for storing elements of a voxelized 6d grid.
        Internally the elements are copied!
    */
    template <typename T>
    class VoxelTree6DElement
    {
    public:
        /*!
            Construct en element at position p with given extends.
        */
        VoxelTree6DElement(float p[6], float extends[6], int level, int maxLevels)
        {
            for (int i = 0; i < 64; i++)
            {
                children[i] = NULL;
            }

            entry = NULL;
            leaf = false;
            memcpy(&(this->pos[0]), &(p[0]), sizeof(float) * 6);
            memcpy(&(this->extends[0]), &(extends[0]), sizeof(float) * 6);
            this->level = level;
            this->maxLevels = maxLevels;

            if (level >= maxLevels)
            {
                VR_ERROR << "Exceeding maxLevels?!" << endl;
            }
        };

        virtual ~VoxelTree6DElement()
        {
            for (int i = 0; i < 64; i++)
            {
                delete children[i];
            }

            delete entry;
        };

        /*!
            Automatically checks if a new child element has to be created.
            A copy of e is stored.
        */
        bool setEntry(float p[6], const T& e)
        {
            if (!covers(p))
            {
                return false;
            }

            if (leaf || level >= (maxLevels - 1))
            {
                leaf = true;
                delete entry;
                entry = new T(e);
                return true;
            }

            VoxelTree6DElement* c = createChild(p); // returns child if it is already existing

            if (!c->setEntry(p, e))
            {
                return false;
            }

            return true;
        };

        /*!
            Checks if there is an entry at the given position.
            True when this node is a leaf and the entry is set or the child at p exists and returns true on getChild(p)->hasEntry(p).
        */
        bool hasEntry(float p[6])
        {
            if (leaf)
            {
                if (!covers(p))
                {
                    return false;
                }

                if (entry)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

            int indx = getChildIndx(p);

            if (indx < 0 || !children[indx])
            {
                return false;
            }

            return children[indx]->hasEntry(p);
        };

        /*!
            Returns pointer to element when existing. NULL if not.
        */
        T* getEntry(float p[6])
        {
            if (leaf)
            {
                return entry;
            }

            int indx = getChildIndx(p);

            if (indx < 0 || !children[indx])
            {
                return NULL;
            }

            return children[indx]->getEntry(p);
        }

        bool isLeaf()
        {
            return leaf;
        }

        //! if isLeaf the corresponding entry is returned
        T* getEntry()
        {
            return entry;
        }
    protected:

        VoxelTree6DElement<T>* createChild(float p[6])
        {
            int indx = getChildIndx(p);

            if (indx < 0)
            {
                VR_ERROR << "Node do not cover this pos" << endl;
                return NULL;
            }

            if (children[indx])
            {
                // silently ignore an existing child
                return children[indx];
            }

            float newPos[6];
            float newExtends[6];

            for (int i = 0; i < 6; i++)
            {
                // check left / right
                if (p[i] > pos[i] + extends[i] * 0.5f)
                {
                    newPos[i] = pos[i] + extends[i] * 0.5f;
                }
                else
                {
                    newPos[i] = pos[i];
                }

                newExtends[i] = 0.5f * extends[i];
            }

            children[indx] = new VoxelTree6DElement(newPos, newExtends, level + 1, maxLevels);
            return children[indx];
        };

        int getChildIndx(float p[6])
        {
            if (!covers(p))
            {
                VR_ERROR << "Node do not cover this pos" << endl;
                return -1;
            }

            if (leaf)
            {
                VR_ERROR << "Node is already a leaf node?!" << endl;
                return -1;
            }

            int res = 0;

            for (int i = 0; i < 6; i++)
            {
                if (p[i] > pos[i] + extends[i] * 0.5f)
                {
                    // right side
                    int powI = 1;

                    for (int j = 0; j < i; j++)
                    {
                        powI *= 2;
                    }

                    res += powI;//pow(2,i) // no int version of pow
                }
            }

            // test, remove this
            if (res < 0 || res >= 64)
            {
                VR_ERROR << "INTERNAL ERROR?!" << endl;
                return -1;
            }

            return res;
        };

        bool covers(float p[6])
        {
            for (int i = 0; i < 6; i++)
            {
                if (p[i] < pos[i] || p[i] > pos[i] + extends[i])
                {
                    return false;
                }
            }

            return true;
        };

        //bool checkAllChildren();

        VoxelTree6DElement* children[64];
        T* entry;
        bool leaf;
        float extends[6];
        float pos[6]; // start == bottom left
        int level;
        int maxLevels;
    };


} // namespace

#endif // _VirtualRobot_VoxelTree6DElement_h_
