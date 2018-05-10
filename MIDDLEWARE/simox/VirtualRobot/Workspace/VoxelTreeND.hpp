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
* @copyright  2013 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_VoxelTreeND_h_
#define _VirtualRobot_VoxelTreeND_h_

#include "../VirtualRobotImportExport.h"
#include "../VirtualRobotException.h"
#include "../MathTools.h"
#include "../XML/FileIO.h"
#include "../Compression/CompressionBZip2.h"
#include "VoxelTreeNDElement.hpp"

#include <string>
#include <vector>
#include <iomanip>

//#define VoxelTreeND_DEBUG_OUTPUT

namespace VirtualRobot
{

    /*!
        A binary tree covering an N-dimensional space with voxels.
        On each tree level, 2^N children exist, each of which is either a leaf (and stores an entry) or it is further divided.
        The elements are created on demand, so it can hold sparse information.

        The space extends are given by minExtend / maxExtend and the maximum voxel sizes are determined by the discretization parameters.
        Note that the worst minExtend/maxExtend and discretization set is chosen to determine the number of levels of the tree.

        The following example shows how a 6D workspace consisting of position(-100mm -> +100mm) and orientation(-PI -> +PI, radian) can be coverd by this class.
        Each voxel covers a 20cm sized cube and rotation intervals of PI/10 radian for each RPY dimension. The voxel can hold an unsigned char value.
        float minExtend[6] = { -100.0f, -100.0f, -100.0f, -M_PI, -M_PI_2, -M_PI};
        float maxExtend[6] = {  100.0f,  100.0f,  100.0f,  M_PI,  M_PI_2,  M_PI};
        float discr[6] = { 20.0f, 20.0f, 20.0f, M_PI/10.0f, M_PI/10.0f, M_PI/10.0f };
        VoxelTreeND<unsigned char, 6> v(minExtend,maxExtend,discr);

        float pos[6] = {10.0f,20.0f,0.0f, 0.1f,0.3f,-0.4f};
        v.setEntry(pos,255);
        unsigned char *e = v.getEntry(pos);
        // check if entry is set
        if (e)
            cout << "Entry is set:" << *e << endl;

    */
    template <typename T, unsigned int N>
    class VoxelTreeND
    {
        friend class VoxelTreeNDElement<T, N>;
    public:


        VoxelTreeND(float minExtend[N], float maxExtend[N], float discretization[N], bool verbose = false):
            verbose(verbose),
            currentElementID(0)
        {
            memcpy(this->minExtend, minExtend, sizeof(float)*N);
            memcpy(this->maxExtend, maxExtend, sizeof(float)*N);
            memcpy(this->discretization, discretization, sizeof(float)*N);

            for (int i = 0; i < N; i++)
            {
                size[i] = maxExtend[i] - minExtend[i];
                THROW_VR_EXCEPTION_IF(size[i] <= 0.0f, "Invalid extend parameters?!");
                THROW_VR_EXCEPTION_IF(discretization[i] <= 0.0f, "Invalid discretization parameters?!");
            }

            int maxSteps = 0;

            for (int i = 0; i < N; i++)
            {
                int steps = (int)(size[i] / discretization[i] + 0.5f);

                if (steps > maxSteps)
                {
                    maxSteps = steps;
                }
            }

            // nrLeafes = 2^(depth-1)
            // -> depth = log_2 nrLeafes + 1
            //logn(x) = log10(x) / log10(n)
            maxLevels = int(ceil(log10((double)maxSteps) / log10((double)2))) + 1;

            // precompute extends for all sub elements
            elementExtends.resize(maxLevels, N);
            float newExtend[N];
            memcpy(newExtend, size, sizeof(float)*N);

            for (int a = 0; a < maxLevels; a++)
            {
                for (int b = 0; b < N; b++)
                {
                    elementExtends(a, b) = newExtend[b];
                    newExtend[b] *= 0.5f;
                }
            }

            //int tmp = int(ceil(sqrt(double(maxSteps))));
            //cout << "MaxLevels:" << maxLevels;
            //cout << "was (sqrt): " << tmp;
            if (verbose)
            {
                VR_INFO << "Creating Voxelized tree data structure. " << endl;
                VR_INFO << "Extends (min/max/size):" << endl;
                std::streamsize pr = std::cout.precision(2);

                for (int i = 0; i < N; i++)
                {
                    cout << std::fixed << minExtend[i] << "," << maxExtend[i] << " -> " << size[i] << endl;
                    cout << std::fixed << "\tdiscretization:" << discretization[i] << ". Max leafs:" << (int)(size[i] / discretization[i] + 0.5f) << endl;
                }

                std::cout << std::resetiosflags(std::ios::fixed);
                std::cout.precision(pr);
                VR_INFO << "--> Max Levels:" << maxLevels << endl;
            }

            THROW_VR_EXCEPTION_IF(maxLevels <= 0, "Invalid parameters...");
            num_children = VirtualRobot::MathTools::pow_int(2, N);
            root = new VoxelTreeNDElement<T, N>(minExtend,/*size,*/0,/*maxLevels,*/this);

        }

        virtual ~VoxelTreeND()
        {
            delete root;
        }

        /*!
            Store entry to this voxel grid.
            Creates a leaf if necessary. Existing entries are silently overwritten.
            A copy of e is created.
        */
        bool setEntry(float pos[N], const T& e)
        {
            return root->setEntry(pos, e);
        }

        /*!
            Returns entry at pos. If pos is outside the space representation or no data stored at pos, NULL is returned.
        */
        T* getEntry(float pos[N])
        {
            return root->getEntry(pos);
        }

        /*!
            Returns size of voxelized data structure (in local coordinate system)
        */
        void getSize(float storeSize[N])
        {
            memcpy(storeSize, size, N * sizeof(float));
        }

        /*!
            Returns min position of voxelized data structure (in local coordinate system)
        */
        void getMinExtend(float storeMin[N])
        {
            memcpy(storeMin, minExtend, N * sizeof(float));
        }

        /*!
            Returns max position of voxelized data structure (in local coordinate system)
        */
        void getMaxExtend(float storeMin[N])
        {
            memcpy(storeMin, maxExtend, N * sizeof(float));
        }

        /*!
            Returns discretization vector as defined on construction. (The actual discretization may differ, depending on max tree depth)
        */
        void getDiscretization(float storeDiscretization[N])
        {
            memcpy(storeDiscretization, discretization, N * sizeof(float));
        }

        /*!
            Get real min element sizes.
        */
        void getRealDiscretization(float storeDiscretization[N])
        {
            for (int i = 0; i < N; i++)
            {
                storeDiscretization[i] = getExtends(maxLevels - 1, i);
            }
        }

        /*!
            Returns leaf at position pos. NULL if no data stored.
        */
        VoxelTreeNDElement<T, N>* getLeafElement(float pos[N])
        {
            return root->getLeaf(pos);
        }

        /*!
            Gets leaf element with max entry T of all sub-tree elements at position p (size of Vector p defines which level of the tree is considered).
            If no entry is stored below p NULL is returned
        */
        VoxelTreeNDElement<T, N>* getMaxEntry(const Eigen::VectorXf& p)
        {
            VR_ASSERT(p.rows() > 0 && p.rows() <= N);
            return root->maxLeaf(p);
        }

        /*!
            Gets all leaf elements of all sub-tree elements at position p (size of Vector p defines which level of the tree is considered).
        */
        std::vector< VoxelTreeNDElement<T, N>* > getAllLeafs(const Eigen::VectorXf& p)
        {
            VR_ASSERT(p.rows() > 0 && p.rows() <= N);
            return root->getAllLeafs(p);
        }

        static VoxelTreeND<T, N>* load(std::ifstream& file)
        {
            THROW_VR_EXCEPTION_IF(!file, "File could not be read.");

            VoxelTreeND<T, N>* tree = NULL;

            try
            {
                std::string tmpString;

                // Check file type
                FileIO::readString(tmpString, file);
                bool fileTypeOK = false;

                if (tmpString == "VoxelTreeND Binary File")
                {
                    fileTypeOK = true;
                }

                THROW_VR_EXCEPTION_IF(!fileTypeOK, "Wrong file format.");

                // Check version
                int64_t version[2];
                FileIO::readArray<int64_t>(version , 2 , file);

                if (version[0] != 1 || version[1] != 1)
                {
                    THROW_VR_EXCEPTION("File version not supported");
                }

                // check sizeof type T
                int64_t expectedSize = int64_t(sizeof(T));
                int64_t storedSize = FileIO::read<int64_t>(file);
                THROW_VR_EXCEPTION_IF(storedSize != expectedSize, "Wrong type information in file...");

                // check N
                int64_t storedN;
                storedN = FileIO::read<int64_t>(file);

                THROW_VR_EXCEPTION_IF(storedN != int64_t(N), "Wrong N information in file...");

                // get maxLevels
                int maxLevels = int(FileIO::read<int64_t>(file));

                // get extends
                float minExtend[N];
                float maxExtend[N];
                float discretization[N];
                FileIO::readArray<float>(minExtend, N, file);
                FileIO::readArray<float>(maxExtend, N, file);
                FileIO::readArray<float>(discretization, N, file);

                for (int i = 0; i < N; i++)
                {
                    float s = maxExtend[i] - minExtend[i];
                    THROW_VR_EXCEPTION_IF(s <= 0.0f, "Invalid extend parameters?!");
                    THROW_VR_EXCEPTION_IF(discretization[i] <= 0.0f, "Invalid discretization parameters?!");
                }

                tree = new VoxelTreeND<T, N>(minExtend, maxExtend, discretization);
                std::map< unsigned int, VoxelTreeNDElement<T, N>* > idElementMapping;
                idElementMapping[1] = tree->root;

                // get nr of entries
                int64_t nrElements = FileIO::read<int64_t>(file);

                // id
                int64_t expectedID = FileIO::read<int64_t>(file);

                // create dummy elements to be filled afterwards
                float p[N];

                //float ex[N];
                for (int64_t i = 1; i < nrElements; i++)
                {
                    VoxelTreeNDElement<T, N>* e = new VoxelTreeNDElement<T, N>(p,/*ex,*/0,/*10,*/tree);
                    idElementMapping[(unsigned int)(i + 1)] = e; // we start with 1, but root is already created!
                }

                VR_ASSERT(tree->currentElementID == (unsigned int)expectedID);

                FileIO::readString(tmpString, file);
                THROW_VR_EXCEPTION_IF(tmpString != "DATA_START", "Bad file format, expecting DATA_START");


                // fill element data
                typename VoxelTreeNDElement<T, N>::datablock d;
                //d.maxLevels = maxLevels;
                CompressionBZip2Ptr bzip2(new CompressionBZip2(&file));
                int numChildren = VirtualRobot::MathTools::pow_int(2, N);
                unsigned int* childIDs = new unsigned int[numChildren];
                int n;

                for (int64_t i = 0; i < nrElements; i++)
                {
                    // read id
                    size_t dataSize = sizeof(unsigned int);
                    bzip2->read((void*)(&(d.id)), dataSize, n);

                    if (n != dataSize)
                    {
                        VR_ERROR << "Invalid number of bytes?!" << endl;
                        bzip2->close();
                        file.close();
                        delete tree;
                        return NULL;
                    }

                    // read levels
                    /*dataSize = sizeof(int);
                    bzip2->read((void*)(&(d.level)),dataSize,n);
                    if (n!=dataSize)
                    {
                        VR_ERROR << "Invalid number of bytes?!" << endl;
                        bzip2->close();
                        file.close();
                        delete tree;
                        return NULL;
                    }
                    // read pos
                    dataSize = sizeof(float)*N;
                    bzip2->read((void*)(&(d.pos[0])),dataSize,n);
                    if (n!=dataSize)
                    {
                        VR_ERROR << "Invalid number of bytes?!" << endl;
                        bzip2->close();
                        file.close();
                        delete tree;
                        return NULL;
                    }
                    // read extends
                    dataSize = sizeof(float)*N;
                    bzip2->read((void*)(&(d.extends[0])),dataSize,n);
                    if (n!=dataSize)
                    {
                        VR_ERROR << "Invalid number of bytes?!" << endl;
                        bzip2->close();
                        file.close();
                        delete tree;
                        return NULL;
                    }*/
                    // read leaf
                    bool leaf = false;
                    dataSize = sizeof(bool);
                    bzip2->read((void*)(&(leaf)), dataSize, n);

                    if (n != dataSize)
                    {
                        VR_ERROR << "Invalid number of bytes?!" << endl;
                        bzip2->close();
                        file.close();
                        delete tree;
                        return NULL;
                    }

                    d.children.clear();

                    if (leaf)
                    {
                        // read entry
                        dataSize = sizeof(T);
                        bzip2->read((void*)(&(d.entry)), dataSize, n);

                        if (n != dataSize)
                        {
                            VR_ERROR << "Invalid number of bytes?!" << endl;
                            bzip2->close();
                            file.close();
                            delete tree;
                            return NULL;
                        }
                    }
                    else
                    {
                        // read children ids
                        dataSize = sizeof(unsigned int) * numChildren;

                        bzip2->read((void*)(&(childIDs[0])), dataSize, n);

                        if (n != dataSize)
                        {
                            VR_ERROR << "Invalid number of bytes?!" << endl;
                            bzip2->close();
                            file.close();
                            delete tree;
                            return NULL;
                        }

                        for (int i = 0; i < numChildren; i++)
                        {
                            d.children.push_back(childIDs[i]);
                        }
                    }

                    VoxelTreeNDElement<T, N>* e = idElementMapping[d.id];

                    //VoxelTreeNDElement<T,N> *e = idElementMapping[unsigned int(i+1)];
                    if (!e->read(d, idElementMapping))
                    {
                        VR_ERROR << "Could not create element" << endl;
                        bzip2->close();
                        file.close();
                        delete tree;
                        return NULL;
                    }

                }

                bzip2->close();
                FileIO::readString(tmpString, file);
                THROW_VR_EXCEPTION_IF(tmpString != "DATA_END", "Bad file format, expecting DATA_END");
                // set root and update extend / pos information
                tree->setRoot(idElementMapping[1]);
                delete []childIDs;
            }
            catch (VirtualRobotException& e)
            {
                VR_ERROR << e.what() << endl;
                file.close();
                throw;
            }

            return tree;
        }
        static VoxelTreeND<T, N>* load(const std::string& filename)
        {
            std::ifstream file(filename.c_str(), std::ios::in | std::ios::binary);
            THROW_VR_EXCEPTION_IF(!file, "File could not be read.");
            VoxelTreeND<T, N>* tree = load(file);
            file.close();
            return tree;
        }
        bool save(std::ofstream& file)
        {
            THROW_VR_EXCEPTION_IF(!file.is_open(), "File is not open...");

            try
            {
                std::string tmpString = "VoxelTreeND Binary File";

                // file type
                FileIO::writeString(file, tmpString);

                // version
                int64_t version[2];
                version[0] = 1;
                version[1] = 1;
                FileIO::writeArray<int64_t>(file, version, 2);

                // sizeof type T
                FileIO::write<int64_t>(file, int64_t(sizeof(T)));

                // N
                FileIO::write<int64_t>(file, int64_t(N));

                // maxLevels
                FileIO::write<int64_t>(file, int64_t(maxLevels));

                // extends
                FileIO::writeArray<float>(file, minExtend, N);
                FileIO::writeArray<float>(file, maxExtend, N);
                FileIO::writeArray<float>(file, discretization, N);

                // nr of entries
                std::vector< VoxelTreeNDElement<T, N>* > elements;
                root->collectElements(elements);
                FileIO::write<int64_t>(file, int64_t(elements.size()));

                // id
                FileIO::write<int64_t>(file, int64_t(currentElementID));

                tmpString = "DATA_START";
                FileIO::writeString(file, tmpString);

                // element data
                typename VoxelTreeNDElement<T, N>::datablock d;
                //d.maxLevels = maxLevels;
                CompressionBZip2Ptr bzip2(new CompressionBZip2(&file));
                int numChildren = VirtualRobot::MathTools::pow_int(2, N);
                unsigned int* childIDs = new unsigned int[numChildren];

                for (size_t i = 0; i < elements.size(); i++)
                {
                    VoxelTreeNDElement<T, N>* e = elements[i];

                    if (!e->write(d))
                    {
                        VR_ERROR << "Could not convert data..." << endl;
                        bzip2->close();
                        file.close();
                        return false;
                    }

                    // write id
                    size_t dataSize = sizeof(unsigned int);
                    bzip2->write((void*)(&d.id), dataSize);

                    // write levels
                    /*dataSize = sizeof(int);
                    bzip2->write((void*)(&(d.level)),dataSize);

                    // write pos
                    dataSize = sizeof(float)*N;
                    bzip2->write((void*)(&(d.pos[0])),dataSize);

                    // write extends
                    dataSize = sizeof(float)*N;
                    bzip2->write((void*)(&(d.extends[0])),dataSize);*/

                    // write leaf
                    bool leaf = (d.children.size() == 0);
                    dataSize = sizeof(bool);
                    bzip2->write((void*)(&leaf), dataSize);

                    if (leaf)
                    {
                        // write entry
                        dataSize = sizeof(T);
                        bzip2->write((void*)(&(d.entry)), dataSize);
                    }
                    else
                    {
                        // write children ids
                        dataSize = sizeof(unsigned int) * numChildren;
                        THROW_VR_EXCEPTION_IF(numChildren != d.children.size(), "Internal error, numChildren wrong...");

                        for (int j = 0; j < numChildren; j++)
                        {
                            childIDs[j] = d.children[j];
                        }

                        bzip2->write((void*)(&(childIDs[0])), dataSize);
                    }
                }

                bzip2->close();

                tmpString = "DATA_END";
                FileIO::writeString(file, tmpString);
                delete[] childIDs;
            }
            catch (VirtualRobotException& e)
            {
                VR_ERROR << e.what() << endl;
                file.close();
                throw;
            }

            return true;
        }

        bool save(const std::string& filename)
        {
            std::ofstream file;
            file.open(filename.c_str(), std::ios::out | std::ios::binary);
            THROW_VR_EXCEPTION_IF(!file.is_open(), "Could not open file");
            bool res = save(file);
            file.close();
            return res;
        }

        bool isCovered(float p[N])
        {
            return root->covers(p);
        }


        void print()
        {
            cout << " **** VoxelTreeND ****" << endl;
            cout << "N=" << N << endl;
            cout << "max levels:" << maxLevels << endl;
            cout << "Element Count:" << currentElementID - 1 << endl;
            cout << "MinExtend:";

            for (int i = 0; i < N; i++)
            {
                cout << minExtend[i] << ",";
            }

            cout << endl;
            cout << "MaxExtend:";

            for (int i = 0; i < N; i++)
            {
                cout << maxExtend[i] << ",";
            }

            cout << endl;
            cout << "Size:";

            for (int i = 0; i < N; i++)
            {
                cout << size[i] << ",";
            }

            cout << endl;
            cout << "Discretization:";

            for (int i = 0; i < N; i++)
            {
                cout << discretization[i] << ",";
            }

            cout << endl;
            cout << " *********************" << endl;
        }

        int getMaxLevels()
        {
            return maxLevels;
        }

        struct ElementIterator
        {
            /*!
                Initialize iterator and return first element (NULL if no elements are present)
            */
            VoxelTreeNDElement<T, N>* init(VoxelTreeND<T, N>* tree)
            {
                this->tree = tree;
                elementStack.clear();
                idStack.clear();
                currentElement = tree->getRoot();

                // go down
                elementStack.push_back(currentElement);
                currentElement = currentElement->getNextChild(0, currentElementNr);

                while (currentElement && !currentElement->isLeaf())
                {
                    elementStack.push_back(currentElement);
                    idStack.push_back(currentElementNr);
                    currentElement = currentElement->getNextChild(0, currentElementNr);
                }

                if (!currentElement || !currentElement->isLeaf())
                {
                    VR_ERROR << "Could not determine first leaf element" << endl;
                }
                else
                {
                    elementStack.push_back(currentElement);
                    idStack.push_back(currentElementNr);
                }

                return currentElement;
            }

            /*!
                Iterate through tree and return next element (NULL when no more elements are present)
            */
            VoxelTreeNDElement<T, N>* getNextElement()
            {
#ifdef VoxelTreeND_DEBUG_OUTPUT
                cout << "current stack:" << endl;
                printStack();
#endif

                if (!currentElement)
                {
                    return NULL;
                }

                if (!currentElement->isLeaf())
                {
                    VR_ERROR << "not at leaf element..." << endl;
                    return NULL;
                }

                currentElement = elementStack.back();
                elementStack.pop_back(); // pop current element from stack

                do
                {
                    currentElement = elementStack.back();
                    currentElementNr = idStack.back();
                    idStack.pop_back();
                    currentElement = currentElement->getNextChild(currentElementNr + 1, currentElementNr);

                    if (!currentElement)
                    {
                        elementStack.pop_back();
                    }
                }
                while (!currentElement && elementStack.size() > 0 && idStack.size() > 0);

                if (!currentElement)
                {
                    return NULL;    // no more elements
                }

                // go down
                //elementStack.push_back(currentElement);
                //idStack.push_back(currentElementNr);
                //if (!currentElement->isLeaf())
                //  currentElement = currentElement->getNextChild(0,currentElementNr);
                while (currentElement && !currentElement->isLeaf())
                {
                    elementStack.push_back(currentElement);
                    idStack.push_back(currentElementNr);
                    currentElement = currentElement->getNextChild(0, currentElementNr);
                }

                if (!currentElement || !currentElement->isLeaf())
                {
                    VR_ERROR << "Could not determine next leaf element" << endl;
                    return NULL;
                }
                else
                {
                    elementStack.push_back(currentElement);
                    idStack.push_back(currentElementNr);
                }

#ifdef VoxelTreeND_DEBUG_OUTPUT
                cout << "new stack:" << endl;
                printStack();
#endif
                return currentElement;
            }

        protected:
            void printStack()
            {
                cout << "Stack: [" << elementStack[0]->getLevel() << "]";

                for (size_t i = 0; i < idStack.size(); i++)
                {
                    cout << "->" << idStack[i] << " ->" << "[" << elementStack[i + 1]->getLevel() << "]";
                }

                cout << endl;
            }

            std::vector<VoxelTreeNDElement<T, N>*> elementStack;
            std::vector<int> idStack;
            VoxelTreeNDElement<T, N>* currentElement;
            int currentElementNr;
            VoxelTreeND<T, N>* tree;
        };

        VoxelTreeNDElement<T, N>* getRoot()
        {
            return root;
        }

        void getMemoryConsumtion(long &storeMemStructure, long &storeMemData)
        {
            storeMemStructure = 0;
            storeMemData = 0;
            storeMemStructure += sizeof(VoxelTreeND<T,N>);
            if (root)
            {
                root->accumulateMemoryConsumtion(storeMemStructure,storeMemData);
            }
        }

        /*!
         *  Retruns number of all Nodes, including inner and leaf nodes.
         */
        long getNumNodes() {
            return root->countNodesRecursive();
        }

    protected:
        /*!
            Returns voxel extends of element in given level (0<=level<maxLevels) and dimension dim (0<=dim<N)
        */
        float getExtends(int level, int dim)
        {
            THROW_VR_EXCEPTION_IF(dim >= N || dim < 0, "Index out of bounds");
            return elementExtends(level, dim);
        }
        int getNumChildren()
        {
            return num_children;
        }

        void setRoot(VoxelTreeNDElement<T, N>* e)
        {
            if (e != root)
            {
                delete root;
                root = e;
            }

            if (root)
            {
                root->propagateData(minExtend,/*size,*/0,/*maxLevels,*/this);
            }
        }
        unsigned int getNextID()
        {
            currentElementID++;
            return currentElementID;
        }

        float minExtend[N];
        float maxExtend[N];
        float size[N];
        float discretization[N];
        int maxLevels;
        bool verbose;
        unsigned int currentElementID;
        int num_children;

        Eigen::MatrixXf elementExtends;

        VoxelTreeNDElement<T, N>* root;

    };



} // namespace

#endif // _VirtualRobot_VoxelTreeND_h_
