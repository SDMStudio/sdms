#pragma once

#include <iostream>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <boost/serialization/set.hpp>

#include <sdm/types.hpp>
#include <sdm/tools.hpp>
#include <sdm/public/boost_serializable.hpp>

namespace sdm
{

    /**
     * @class GraphNode
     * 
     * @brief Generic GraphNode class 
     * 
     * @tparam T the type of the data contains in each node
     * 
     * 
     * Usage
     *         GraphNode<int> tree();
     * 
     */
    template <typename TNode, typename TEdge>
    class GraphNode :   public std::enable_shared_from_this<GraphNode<TNode, TEdge>>,
                        public BoostSerializable<GraphNode<TNode, TEdge>>
    {
    public:
        /**
         * @brief Default constructor object
         * 
         */
        GraphNode();

        /**
         * @brief Construct a graph with an initial node*
         * 
         * @param data 
         */
        GraphNode(const TNode &data);

        /**
         * @brief Construct a new GraphNode object
         * 
         * @param parent the parent
         * @param data the value of the node
         * @param backup if true, save the new tree as a child for its parent
         */
        // GraphNode(std::shared_ptr<GraphNode> predecessor, const TNode &data);

        /**
         *  @fn     ~GraphNode()
         *  @brief  Destructor of GraphNode (that's bad). 
         *
         *  This destructor recursively all, children and the item from the tree, bottom up.
         */
        virtual ~GraphNode();

        /**
         * @brief Get the value of the current node
         * 
         * @return the address of the value
         */
        TNode getData() const;

        TNode &&data() const;

        void setData(const TNode &data);

        /**
         * @brief Get the number of successors.
         */
        number getNumSuccessors() const;

        /**
         * @brief Get the number of predecessors
         */
        number getNumPredecessors() const;

        /**
         * @brief Get the successor following a given edge 
         * 
         * @param edge a specific edge
         * @return the address of the successor's node
         */
        std::shared_ptr<GraphNode> getSuccessor(const TEdge &edge) const;

        /**
         * @brief Get the set of all predecessors
         * 
         * @return the set of predecessors 
         */
        std::shared_ptr<GraphNode> getPredecessor(const TEdge &edge) const;

        /**
         * @brief Add a successor node.
         * 
         * @param edge the edge
         * @param node the successor node value
         */
        void addSuccessor(const TEdge &edge_value, const std::shared_ptr<GraphNode> &node_value);

        /**
         * @brief Add a predecessor to the current node.
         * 
         * @param node_value the predecessor node value
         */
        void addPredecessor(const TEdge &edge_value, const std::shared_ptr<GraphNode> &node_value);

        std::string str() const;

        std::shared_ptr<GraphNode> getptr();

        template <class Archive>
        void serialize(Archive &archive, const unsigned int);

        friend std::ostream &operator<<(std::ostream &os, GraphNode &graph)
        {
            os << graph.str();
            return os;
        }

    public:
        /** @brief data of the current node */
        TNode data_;

        /**
         * @brief The map from edge value to successor
         */
        std::unordered_map<TEdge, std::weak_ptr<GraphNode>> successors;

        /**
         * @brief List of predecessors 
         */
        std::unordered_map<TEdge, std::weak_ptr<GraphNode>> predecessors;
    };

} // namespace sdm

#include <sdm/utils/struct/graph_node.tpp>