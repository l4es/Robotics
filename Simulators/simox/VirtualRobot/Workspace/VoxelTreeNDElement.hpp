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
#ifndef _VirtualRobot_VoxelTreeNDElement_h_
#define _VirtualRobot_VoxelTreeNDElement_h_

#include "../VirtualRobotImportExport.h"

#include <string>
#include <vector>
#include <map>
//#define VoxelTreeNDElement_DEBUG_OUTPUT

namespace VirtualRobot
{
    template <typename T, unsigned int N>
    class VoxelTreeND;
    /*!
        A template definition for storing elements of a voxelized n-d grid.
        Internally the elements are copied!
    */
    template <typename T, unsigned int N>
    class VoxelTreeNDElement
    {
        friend class VoxelTreeND<T, N>;
    public:
        /*!
            Construct an element at position p with given extends.
        */
        VoxelTreeNDElement(float p[N], /*float extends[N],*/ int level, /*int maxLevels,*/ VoxelTreeND<T, N>* master)
        {
            VR_ASSERT(master);
            this->tree = master;

            entry = NULL;

            if (level >= (tree->getMaxLevels() - 1))
            {
                leaf = true;
                children = NULL;
            } else
            {
                leaf = false;
                //num_children = pow_int(2,N);
                children = new VoxelTreeNDElement*[master->getNumChildren()];

                for (int i = 0; i < master->getNumChildren(); i++)
                {
                    children[i] = NULL;
                }
            }


            memcpy(&(this->pos[0]), &(p[0]), sizeof(float)*N);
            //memcpy (&(this->extends[0]),&(extends[0]),sizeof(float)*N);
            this->level = level;
            //this->maxLevels = maxLevels;
            VR_ASSERT_MESSAGE(level < master->getMaxLevels(), "Exceeding maxLevels?!");
            this->id = master->getNextID();
        };

        virtual ~VoxelTreeNDElement()
        {
            deleteData();
        };

        /*!
            Automatically checks if a new child element has to be created.
            A copy of e is stored.
        */
        bool setEntry(float p[N], const T& e)
        {
            if (!covers(p))
            {
                return false;
            }

#ifdef VoxelTreeNDElement_DEBUG_OUTPUT
            cout << "[" << level << "]";
#endif

            if (leaf || level >= (tree->getMaxLevels() - 1))
            {
                leaf = true;
                delete entry;
                entry = new T(e);
#ifdef VoxelTreeNDElement_DEBUG_OUTPUT
                cout << ":" << e;
#endif
                return true;
            }

            VoxelTreeNDElement* c = createChild(p); // returns child if it is already existing

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
        bool hasEntry(float p[N])
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
        T* getEntry(float p[N])
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


        VoxelTreeNDElement* getLeaf(float pos[N])
        {
            if (leaf)
            {
                return this;
            }

            int indx = getChildIndx(pos);

            if (indx < 0 || !children[indx])
            {
                return NULL;
            }

            return children[indx]->getLeaf(pos);
        }


        VoxelTreeNDElement* maxLeaf(const Eigen::VectorXf& pos)
        {
            if (leaf)
            {
                return this;
            }

            VR_ASSERT(pos.rows() > 0 && pos.rows() <= N);
            int numChildren = tree->getNumChildren();
            std::vector<int> c = getAllChildrenIndx(pos);
            std::vector<int>::iterator it = c.begin();
            T* maxEntry = NULL;
            VoxelTreeNDElement* maxElement = NULL;

            while (it != c.end())
            {
                VR_ASSERT((*it) >= 0 && (*it) < numChildren);

                if (children[*it])
                {
                    VoxelTreeNDElement* e = children[*it]->maxLeaf(pos);

                    if (e && e->isLeaf())
                    {
                        if (!maxEntry || *(e->entry) > *maxEntry)
                        {
                            maxEntry = e->entry;
                            maxElement = e;
                        }
                    }
                }

                it++;
            }

            return maxElement;
        }

        std::vector<VoxelTreeNDElement*> getAllLeafs(const Eigen::VectorXf& pos)
        {
            std::vector<VoxelTreeNDElement*> res;

            if (leaf)
            {
                res.push_back(this);
                return res;
            }

            VR_ASSERT(pos.rows() > 0 && pos.rows() <= N);
            int numChildren = tree->getNumChildren();
            std::vector<int> c = getAllChildrenIndx(pos);
            std::vector<int>::iterator it = c.begin();

            while (it != c.end())
            {
                VR_ASSERT((*it) >= 0 && (*it) < numChildren);

                if (children[*it])
                {
                    std::vector<VoxelTreeNDElement*> resSub;
                    resSub = children[*it]->getAllLeafs(pos);

                    if (!resSub.empty())
                    {
                        res.insert(res.end(), resSub.begin(), resSub.end());
                    }
                }

                it++;
            }

            return res;
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

        float getExtend(unsigned int d)
        {
            return tree->getExtends(level, d);
        }

        int getLevel()
        {
            return level;
        }

        Eigen::VectorXf getVoxelCenter()
        {
            Eigen::VectorXf c(N);

            for (int i = 0; i < N; i++)
            {
                c(i) = pos[i] + tree->getExtends(level, i) * 0.5f;
            }

            return c;
        }

        long countNodesRecursive() {
            long counter = 0;
            if (!leaf) {
                int index = 0;
                VoxelTreeNDElement<T, N> *nextChild = getNextChild(index, index);
                    while (nextChild) {
                        counter += nextChild->countNodesRecursive();
                        nextChild = getNextChild(index + 1, index);
                    }
            }
            return counter + 1;
        }

    protected:
        std::vector<int> getAllChildrenIndx(const Eigen::VectorXf& p)
        {
            std::vector<int> c;
            int depth = p.rows();
            int res = 0;

            for (int i = 0; i < depth; i++)
            {
                if (p[i] > pos[i] + tree->getExtends(level, i) * 0.5f)
                {
                    // right side
                    res += VirtualRobot::MathTools::pow_int(2, i);
                }
            }

            c.push_back(res); // all left (depth->N)
            int res2 = res;

            for (int i = depth; i < N; i++)
            {
                //c2.push_back(res2);
                // all entries of c with right at current level i
                std::vector<int> c2;

                for (size_t j = 0; j < c.size(); j++)
                {
                    int res2 = c[j] + VirtualRobot::MathTools::pow_int(2, i);
                    c2.push_back(res2);
                }

                c.insert(c.end(), c2.begin(), c2.end());
            }

            return c;
        }

        struct datablock
        {
            std::vector<unsigned int> children;
            T entry;
            //float extends[N];
            //float pos[N]; // start == bottom left
            //int level;
            //int maxLevels;
            unsigned int id;
        };

        bool write(datablock& storeData)
        {
            storeData.children.clear();

            if (!leaf)
            {
                for (int i = 0; i < tree->getNumChildren(); i++)
                {
                    if (children[i])
                    {
                        storeData.children.push_back(children[i]->id);
                    }
                    else
                    {
                        storeData.children.push_back(0);
                    }
                }
            }
            else
            {
                if (entry)
                {
                    storeData.entry = *entry;
                }
                else
                {
                    VR_ERROR << "Undefined entry ?!" << endl;
                    return false;
                }
            }

            //memcpy(&(storeData.extends[0]),&(extends[0]),sizeof(float)*N);
            //memcpy(&(storeData.pos[0]),&(pos[0]),sizeof(float)*N);
            //storeData.level = level;
            storeData.id = id;
            //storeData.maxLevels = maxLevels;
            return true;
        }

        bool read(const datablock& data, const std::map< unsigned int, VoxelTreeNDElement* >& idElementMapping)
        {
            deleteData();
            

            entry = NULL;
            leaf = false;
            //this->level = data.level;
            //this->maxLevels = data.maxLevels;
            this->id = data.id;

            int num_children = data.children.size();
            leaf = num_children == 0;

            if (!leaf)
            {
                children = new VoxelTreeNDElement*[tree->getNumChildren()];

                for (int i = 0; i < tree->getNumChildren(); i++)
                {
                    children[i] = NULL;
                }
                for (int i = 0; i < num_children; i++)
                {
                    if (data.children[i] > 0)
                    {
                        typename std::map< unsigned int, VoxelTreeNDElement<T, N>* >::const_iterator it = idElementMapping.find(data.children[i]);

                        if (it == idElementMapping.end())
                        {
                            VR_ERROR << "Could not find Element with id " << data.children[i] << endl;
                            return false;
                        }
                        else
                        {
                            this->children[i] = it->second;
                        }
                    }
                }
            }
            else
            {
                children = NULL;
                entry = new T(data.entry);
            }

            //memcpy(&(extends[0]), &(data.extends[0]), sizeof(float)*N);
            //memcpy(&(pos[0]), &(data.pos[0]), sizeof(float)*N);
            return true;
        }

        VoxelTreeNDElement<T, N>* createChild(float p[N])
        {
            VR_ASSERT (!leaf);
            int indx = getChildIndx(p);

            if (indx < 0)
            {
                VR_ERROR << "Node do not cover this pos:" << endl;

                for (int i = 0; i < N; i++)
                {
                    cout << p[i] << ",";
                }

                cout << endl;
                return NULL;
            }

#ifdef VoxelTreeNDElement_DEBUG_OUTPUT
            cout << "->" << indx << "->";
#endif

            if (children[indx])
            {
                // silently ignore an existing child
                return children[indx];
            }

            float newPos[N];

            //float newExtends[N];
            for (int i = 0; i < N; i++)
            {
                // check left / right
                if (p[i] > pos[i] + tree->getExtends(level, i) * 0.5f)
                {
                    newPos[i] = pos[i] + tree->getExtends(level, i) * 0.5f;
                }
                else
                {
                    newPos[i] = pos[i];
                }

                //newExtends[i] = 0.5f * extends[i];
            }

            children[indx] = new VoxelTreeNDElement(newPos,/*newExtends,*/level + 1,/*maxLevels,*/tree);
            return children[indx];
        };

        int getChildIndx(float p[N])
        {
            if (!covers(p))
            {
                VR_ERROR << "Node do not cover this pos" << endl;

                for (int i = 0; i < N; i++)
                {
                    cout << p[i] << ",";
                }

                cout << endl;
                return -1;
            }

            if (leaf)
            {
                VR_ERROR << "Node is already a leaf node?!" << endl;
                return -1;
            }

            int res = 0;

            for (int i = 0; i < N; i++)
            {
                if (p[i] > pos[i] + tree->getExtends(level, i) * 0.5f)
                {
                    // right side
                    res += VirtualRobot::MathTools::pow_int(2, i);
                }
            }

            VR_ASSERT(res >= 0 && res < tree->getNumChildren());
            return res;
        };

        bool covers(float p[N])
        {
            for (int i = 0; i < N; i++)
            {
                if (p[i] < pos[i] || p[i] > pos[i] + tree->getExtends(level, i))
                {
                    return false;
                }
            }

            return true;
        };

        void accumulateMemoryConsumtion(long &storeMemStructure, long &storeMemData)
        {
            storeMemStructure += sizeof(VoxelTreeNDElement<T,N>);

            if (entry)
            {
                VR_ASSERT(leaf);
                storeMemData += sizeof(T);
            }
            else
            {
                VR_ASSERT(!leaf);
                
                // size of children array 
                storeMemStructure += tree->getNumChildren() * sizeof (VoxelTreeNDElement*);

                for (int i = 0; i < tree->getNumChildren(); i++)
                {
                    if (children[i])
                    {
                        children[i]->accumulateMemoryConsumtion(storeMemStructure,storeMemData);
                    }
                }
            }
        }

        void collectElements(std::vector<VoxelTreeNDElement*>& elements)
        {
            elements.push_back(this);

            if (!leaf)
            {
                for (int i = 0; i < tree->getNumChildren(); i++)
                {
                    if (children[i])
                    {
                        children[i]->collectElements(elements);
                    }
                }
            }
        }

        void propagateData(float p[N], /*float extends[N],*/ int level, /*int maxLevels,*/ VoxelTreeND<T, N>* master)
        {
            VR_ASSERT(master);
            this->tree = master;
            memcpy(&(this->pos[0]), &(p[0]), sizeof(float)*N);
            //memcpy (&(this->extends[0]),&(extends[0]),sizeof(float)*N);
            this->level = level;

            //this->maxLevels = maxLevels;
            if (level >= tree->getMaxLevels())
            {
                VR_ERROR << "Exceeding maxLevels?!" << endl;
            }

            if (!leaf)
            {
                //float newExtends[N];
                float newP[N];
                //for (int i=0;i<N;i++)
                //newExtends[i] = 0.5f * extends[i];

                for (int i = 0; i < tree->getNumChildren(); i++)
                {
                    if (children[i])
                    {
                        // create newP
                        int res = i;

                        for (int j = N - 1; j >= 0; j--)
                        {
                            int k = VirtualRobot::MathTools::pow_int(2, j);

                            if (res >= k)
                            {
                                // right
                                newP[j] = p[j] + 0.5f * tree->getExtends(level, j);
                                res -= k;
                            }
                            else
                            {
                                // left
                                newP[j] = p[j];
                            }
                        }

                        children[i]->propagateData(newP,/*newExtends,*/level + 1/*,maxLevels*/, master);
                    }
                }
            }
        }

        void deleteData()
        {
            if (children)
            {
                for (int i = 0; i < tree->getNumChildren(); i++)
                {
                    delete children[i];
                }
                delete[] children;
            }
            delete entry;
            children = NULL;
            entry = NULL;
        }

        VoxelTreeNDElement<T, N>* getNextChild(int startIndex, int& storeElementNr)
        {
            VR_ASSERT(!leaf);
            for (int i = startIndex; i < tree->getNumChildren(); i++)
            {
                if (children[i])
                {
                    storeElementNr = i;
                    return children[i];
                }
            }

            return NULL;
        }

        VoxelTreeNDElement** children;
        T* entry;
        bool leaf;
        //float extends[N];
        float pos[N]; // start == bottom left
        int level;
        //int maxLevels;
        //int num_children;
        unsigned int id;
        VoxelTreeND<T, N>* tree;
    };


} // namespace

#endif // _VirtualRobot_VoxelTree6DElement_h_
