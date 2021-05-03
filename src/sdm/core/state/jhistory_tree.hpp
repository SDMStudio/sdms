/**
 * @file jhistory_tree.hpp
 * @author Jilles S. Dibangoye
 * @author David Albert
 * @brief History Tree data structure
 * @version 0.1
 * @date 14/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <sdm/core/state/history_tree.hpp>
#include <sdm/core/joint.hpp>

namespace sdm
{

    /**
     * @class JointHistoryTree
     * 
     * @brief 
     * 
     * @tparam T 
     */
    template <typename T>
    class JointHistoryTree : public HistoryTree<Joint<T>>, public Joint<std::shared_ptr<HistoryTree<T>>>
    {
    protected:
        void addIndividualHistory(std::shared_ptr<HistoryTree<T>> ihist);

    public:
        using ihistory_type = std::shared_ptr<HistoryTree<T>>;

        /*!
         *  @fn     JointHistoryTree()
         *  @brief  Default constructor.
         *  This constructor builds a default and empty tree.
         */
        JointHistoryTree();

        /**
         * @brief Construct a new joint history tree object (the origin)
         * 
         * @param n_agents the number of agents
         */
        JointHistoryTree(number n_agents);

        /**
         * @brief Construct a new truncated joint history tree object (the origin)
         * 
         * @param n_agents the number of agents
         * @param max_depth the maximal depth of the truncated history  
         */
        JointHistoryTree(number n_agents, number max_depth);

        /**
         *  @brief  This constructor build a child node given its parent's node and a new joint item.
         *  @param  parent   the parent tree
         *  @param  item     the item
         */
        JointHistoryTree(std::shared_ptr<JointHistoryTree<T>> parent, const Joint<T> &item);

        /**
         * @brief Construct a new joint history based on individual histories
         * @warning This will build a well defined Joint<std::shared_ptr<HistoryTree<T>>> structure but wrong HistoryTree<Joint<T>> !!
         * 
         * @param ihistories the list of individual histories
         */
        JointHistoryTree(const Joint<std::shared_ptr<HistoryTree<T>>> &ihistories);

        /*!
         *  @brief  Expands the tree
         *  @param  data the data of the expanded node
         *  @return the expanded tree
         *
         *  If child leading from the item previously exists, the method return
         *  that child. Otherwise, it expands the tree by adding an item at the
         *  current leaf of the tree and creating if necessary a corresponding
         *  child. The constructed child is returned.
         */
        std::shared_ptr<JointHistoryTree<T>> expand(const Joint<T> &data, bool backup = true);

        /**
         * @brief Get the address of the individual history of agent 'agent_id' 
         * 
         * @param agent_id the agent id  
         * @return the address of the individual history of agent 'agent_id' 
         */
        std::shared_ptr<HistoryTree<T>> getIndividualHistory(number agent_id) const;

        /**
         * @brief Get the address of the individual histories of all agents
         * 
         * @return a vector that contains all individual histories
         */
        Joint<std::shared_ptr<HistoryTree<T>>> getIndividualHistories() const;

        std::string str();

        friend std::ostream &operator<<(std::ostream &os, JointHistoryTree &j_hist)
        {
            os << j_hist.str();
            return os;
        }
    };

} // namespace sdm
#include <sdm/core/state/jhistory_tree.tpp>