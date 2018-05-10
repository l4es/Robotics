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
#ifndef _saba_cspacetree_h
#define _saba_cspacetree_h

#include "../Saba.h"
#include <iostream>
#include <vector>
#include <map>

namespace Saba
{

    /*!
     * \brief   Contains CSpaceTree which uses a CSpace for managing configurations
     *
     *
     * A connected tree in c-space which represents a RRT.
     *
     */
    class SABA_IMPORT_EXPORT CSpaceTree
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //! constructor
        CSpaceTree(CSpacePtr cspace);

        //! destructor
        virtual ~CSpaceTree();


        /*!
            Return a cspace node.
          \param id the id of the node
          \return pointer to the cspace node
        */
        CSpaceNodePtr getNode(unsigned int id);

        CSpaceNodePtr getLastAddedNode();

        CSpacePtr getCSpace();

        //! get number of nodes
        virtual unsigned int getNrOfNodes() const;

        unsigned int getDimension() const;

        /*!
            creates path from startNode to goalNode by traversing the parents of goalNode
            \param startNode The start.
            \param goalNode The goal.
            \param fillPath Stores all configs.
            \return true on success.
        */
        virtual bool createPath(CSpaceNodePtr startNode, CSpaceNodePtr goalNode, CSpacePathPtr fillPath);

        //! set the number of nodes to zero
        virtual void reset();

        //! saves all nodes with ID, configuration and parentID to a file
        virtual bool saveAllNodes(char const* filename);

        /*!
            consider child vector list when adding nodes to the tree
            (standard: false)
            @see CSpaceNode
        */
        void setUpdateChildren(bool enable)
        {
            updateChildren = enable;
        };
        bool getUpdateChildren()
        {
            return updateChildren;
        }


        //! creates new CSpaceNode with configuration config and parentID
        /*!
          \param config the configuration
          \param parentID ID of the parent
          \param calcDistance if set, the distance to obstacles is calculated and stored in the node (be careful: expensive), otherwise the dist is set to -1.0
          \return pointer to the new node on success, otherwise NULL
        */
        virtual CSpaceNodePtr appendNode(const Eigen::VectorXf& config, int parentID, bool calcDistance = false);

        //! remove tree node
        virtual void removeNode(CSpaceNodePtr n);

        /*!
            get ID of nearest neighbor
          \param config configuration
          \param storeDist pointer to float for storing the distance (if not NULL)
          \return ID of nearest neighbor
        */
        virtual unsigned int getNearestNeighborID(const Eigen::VectorXf& config, float* storeDist = NULL);
        virtual CSpaceNodePtr getNearestNeighbor(const Eigen::VectorXf& config, float* storeDist = NULL);

        /*!
            append a path to the tree (without any checks)
            Intermediate configs are generated according to the current cspace
          \param startNode start from this node
          \param config
          \param storeLastAddedID if given, the id of the last added node is stored here
        */
        virtual bool appendPath(CSpaceNodePtr startNode, const Eigen::VectorXf& config, int* storeLastAddedID = NULL);

        /*!
        Append the given path to this tree. No checks are performed.
        */
        virtual bool appendPath(CSpaceNodePtr startNode, CSpacePathPtr path, int* storeLastAddedID = NULL);

        /*!
            Append a path to the tree until a collision is detected
            Intermediate configs are generated according to the current cspace
          \param startNode start from this node
          \param config
          \param storeLastAddedID if given, the id of the last added node is stored here
        */
        virtual bool appendPathUntilCollision(CSpaceNodePtr startNode, const Eigen::VectorXf& config, int* storeLastAddedID);

        virtual std::vector<CSpaceNodePtr> getNodes();


        /*!
          Create a new cspace node (does !not! set the config values to zero).
          \return pointer to new created node in cspace
        */
        //virtual CSpaceNodePtr createNewNode();

        void lock();
        void unlock();


        /*!
            Computes the complete length of all edges.
            \param useMetricWeights When set, the metric weights of the corresponding cspace are used for length calculation. When the cspace is not initialized with any metric weights, this option has no effect.
        */
        float getTreeLength(bool useMetricWeights = true);

        bool hasNode(CSpaceNodePtr n);

    protected:

        //! get distance of path start to end
        virtual float getPathDist(unsigned int idStart, unsigned int idEnd, bool useMetricWeights);

        //! shutdown tree, reset all data
        //virtual void shutdown();
        unsigned int dimension;             //!< dimension of the cspace

        CSpacePtr cspace;


        Eigen::VectorXf tmpConfig;

        std::vector< CSpaceNodePtr > nodes;             //! vector with pointers to all used nodes


        bool updateChildren;                    // CSpaceNode child management
        float randMult;

        std::map<unsigned int, CSpaceNodePtr > idNodeMapping; // mapping id<->node

        boost::mutex mutex;
    };

} // namespace Saba

#endif // _saba_cspacetree_h
