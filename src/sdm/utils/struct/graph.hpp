#pragma once

#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <iostream>
#include <boost/serialization/set.hpp>

// #include <sdm/utils/struct/node.hpp>
#include <sdm/types.hpp>
#include <sdm/tools.hpp>
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
         * @brief Construct a graph with an initial node*
         * 
         * @param data 
         */
        Graph(const TNode &data, const std::shared_ptr<std::unordered_map<TNode, std::shared_ptr<Graph>>> &node_space);

        /**
         * @brief Construct a new Graph object
         * 
         * @param parent the parent
         * @param data the value of the node
         * @param backup if true, save the new tree as a child for its parent
         */
        Graph(std::shared_ptr<Graph> predecessor, const TNode &data);

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
        std::shared_ptr<Graph> getNode(const TNode &belief) const;

        /**
         * @brief Add a node in the graph.
         * 
         * @param node_value the value of the node
         */
        void addNode(const TNode &node_value);

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
        std::shared_ptr<Graph> getSuccessor(const TEdge &edge) const;

        /**
         * @brief Get the set of all predecessors
         * 
         * @return the set of predecessors 
         */
        std::set<std::shared_ptr<Graph>> getPredecessors() const;

        /**
         * @brief Add a successor node.
         * 
         * @param edge the edge
         * @param node the successor node value
         */
        void addSuccessor(const TEdge &edge_value, const TNode &node_value);

        /**
         * @brief Add a predecessor to the current node.
         * 
         * @param node_value the predecessor node value
         */
        void addPredecessor(const TNode &node_value);

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

    protected:
        /** @brief data of the current node */
        TNode data_;

        /**
         * @brief The map from edge value to successor
         */
        std::unordered_map<TEdge, std::shared_ptr<Graph>> successors;

        /**
         * @brief List of predecessors 
         */
        std::set<std::shared_ptr<Graph>> predecessors;

        /**
         * @brief Space of nodes
         */
        std::shared_ptr<std::unordered_map<TNode, std::shared_ptr<Graph>>> node_space_;
    };

} // namespace sdm

#include <sdm/utils/struct/graph.tpp>