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
     * @brief Node of graphs.
     * 
     * GraphNode class is provide to give the user the possibility to transit directly on them. 
     * In fact, the class keep all the successors of a node in its attribute. 
     * 
     * @tparam TNode the type of the data contains in each node
     * @tparam TEdge the type of the edges between two nodes
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
         *  @fn     ~GraphNode()
         *  @brief  Destructor of GraphNode.
         *  
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
         * @brief Get the successor following a given edge 
         * 
         * @param edge a specific edge
         * @return the address of the successor's node
         */
        std::shared_ptr<GraphNode> getSuccessor(const TEdge &edge) const;

        /**
         * @brief Add a successor node.
         * 
         * @param edge the edge
         * @param node the successor node value
         */
        void addSuccessor(const TEdge &edge_value, const std::shared_ptr<GraphNode> &node_value);

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

        /** @brief The map from edge value to successor */
        std::unordered_map<TEdge, std::weak_ptr<GraphNode>> successors;
    };

} // namespace sdm

#include <sdm/utils/struct/graph_node.tpp>