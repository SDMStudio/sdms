#pragma once

#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <iostream>
#include <boost/serialization/set.hpp>

// #include <sdm/utils/struct/node.hpp>
#include <sdm/types.hpp>
#include <sdm/tools.hpp>
#include <sdm/utils/struct/graph_node.hpp>
#include <sdm/public/boost_serializable.hpp>

namespace sdm
{

    /**
     * @class Graph
     * 
     * @brief Generic Graph class 
     * 
     * @tparam T the type of the data contains in each node
     * 
     * 
     * Usage
     *         Graph<int> tree();
     * 
     */
    template <typename TNode, typename TEdge>
    class Graph : public std::enable_shared_from_this<Graph<TNode, TEdge>>,
                   public BoostSerializable<Graph<TNode, TEdge>>
    {
    public:
        /**
         * @brief Default constructor object
         * 
         */
        Graph();

        /**
         *  @fn     ~Graph()
         *  @brief  Destructor of Graph (that's bad). 
         *
         *  This destructor recursively all, children and the item from the tree, bottom up.
         */
        virtual ~Graph();

        /**
         * @brief Get the node associated to a given node's value .
         * 
         * @param node_value a specific node value
         * @return the address of the node 
         */
        std::shared_ptr<GraphNode<TNode, TEdge>> getNode(const TNode &belief) const;

        /**
         * @brief Add a node in the graph.
         * 
         * @param node_value the value of the node
         */
        void addNode(const TNode &node_value);

        /**
         * @brief Get the number of node.
         */
        number getNumNodes() const;

        std::shared_ptr<GraphNode<TNode, TEdge>> getSuccessor(const TNode &node, const TEdge &edge) const;
        std::shared_ptr<GraphNode<TNode, TEdge>> getPredecessor(const TNode &node, const TEdge &edge) const;

        void addSuccessor(const TNode &node_value, const TEdge &edge_value, const TNode &succ_node_value);

        std::string str() const;
        std::string node_str() const;

        std::shared_ptr<Graph> getptr();

        template <class Archive>
        void serialize(Archive &archive, const unsigned int);

        friend std::ostream &operator<<(std::ostream &os, Graph &graph)
        {
            os << graph.str();
            return os;
        }

        bool contains(const TNode &node_value) const;

    public:
        /**
         * @brief Space of nodes
         */
        std::unordered_map<TNode, std::shared_ptr<GraphNode<TNode, TEdge>>> node_space_;
    };

} // namespace sdm

#include <sdm/utils/struct/graph.tpp>