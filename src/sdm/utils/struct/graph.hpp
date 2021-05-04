#pragma once

#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <iostream>

// #include <sdm/utils/struct/node.hpp>
#include <sdm/types.hpp>
#include <sdm/tools.hpp>

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
    class Graph : public std::enable_shared_from_this<Graph<TNode, TEdge>>
    {
    public:
        /**
         * @brief Default constructor object
         * 
         */
        Graph();

        Graph(const std::shared_ptr<TNode> &data);

        /**
         * @brief Construct a new Graph object
         * 
         * @param parent the parent
         * @param data the value of the node
         * @param backup if true, save the new tree as a child for its parent
         */
        Graph(std::shared_ptr<Graph> predecessor, const std::shared_ptr<TNode> &data);

        /*!
         *  @fn     ~Tree()
         *  @brief  Destructor of Tree (that's bad). 
         *
         *  This destructor recursively all, children and the item from the tree, bottom up.
         */
        virtual ~Graph();

        /**
         * @brief Get the value of the current node
         * 
         * @return the address of the value
         */
        std::shared_ptr<TNode> getData() const;

        void setData(const std::shared_ptr<TNode> &data);

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
        std::shared_ptr<Graph<TNode, TEdge>> getSuccessor(const TEdge &edge) const;
        std::vector<std::shared_ptr<Graph<TNode, TEdge>>> getPredecessors() const;

        void addSuccessor(const TEdge &edge, const std::shared_ptr<TNode> &node);

        std::string str();

        std::shared_ptr<Graph<TNode, TEdge>> getptr();

        friend std::ostream &operator<<(std::ostream &os, Graph<TNode, TEdge> &graph)
        {
            os << graph.str();
            return os;
        }

    protected:
        //! @brief data of the current node
        std::shared_ptr<TNode> data_;

        /**
         * @brief The map from edge value to successor
         */
        std::unordered_map<TEdge, std::shared_ptr<Graph>> successors;

        /**
         * @brief List of predecessors 
         */
        std::vector<std::shared_ptr<Graph>> predecessors;
    };

} // namespace sdm

#include <sdm/utils/struct/graph.tpp>