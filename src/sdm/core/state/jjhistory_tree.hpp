/**
 * @file history_tree.hpp
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

#include <sdm/core/state/jhistory_tree.hpp>

namespace sdm
{

    /**
     * @class JointJointHistoryTree
     * 
     * @brief 
     * 
     * @tparam T 
     */
    template <typename T>
    class JointJointHistoryTree : public JointHistoryTree<Joint<T>>
    {
    protected:

        void addIndivJHist(std::shared_ptr<JointHistoryTree<T>> jhist);
    public:
        using ihistory_type = std::shared_ptr<HistoryTree<T>>;
        using jhistory_type = std::shared_ptr<JointHistoryTree<T>>;
        Joint<std::shared_ptr<JointHistoryTree<T>>> indiv_jhists;

        /*!
         *  @fn     JointJointHistoryTree()
         *  @brief  Default constructor.
         *  This constructor builds a default and empty tree.
         */
        JointJointHistoryTree();

        /**
         * @brief Construct a new joint history tree object (the origin)
         * 
         * @param n_agents the number of agent
         */
        JointJointHistoryTree(number n_agents);

        JointJointHistoryTree(number n_agents, number max_depth);

        JointJointHistoryTree(std::vector<jhistory_type> trees);

        /*!
         *  @fn     JointJointHistoryTree(std::shared_ptr<JointJointHistoryTree>, const T&, bool = true)
         *  @brief  constructor
         *  @param  parent   the parent tree
         *  @param  item     the item
         *  @param  backup wheter the node is marked or not
         *  This constructor builds a tree with a given parent and item.
         */
        JointJointHistoryTree(std::shared_ptr<JointJointHistoryTree<T>> parent, const Joint<T> &item);

        /*!
         *  @fn     HistoryTree<T> *expand(const T &data);
         *  @brief  Expands the tree
         *  @param  data the data of the expanded node
         *  @return the expanded tree
         *
         *  If child leading from the item previously exists, the method return
         *  that child. Otherwise, it expands the tree by adding an item at the
         *  current leaf of the tree and creating if necessary a corresponding
         *  child. The constructed child is returned.
         */
        std::shared_ptr<JointJointHistoryTree<T>> expand(const Joint<Joint<T>> &data, bool backup = true);

        std::shared_ptr<JointHistoryTree<T>> getIndividualJointHistory(number ag_id) const;
        std::vector<std::shared_ptr<JointHistoryTree<T>>> getIndividualJointHistories() const;

        friend std::ostream &operator<<(std::ostream &os, const JointJointHistoryTree &j_hist)
        {
            os << static_cast<HistoryTree<Joint<T>>>(j_hist);
            return os;
        }
    };

} // namespace sdm
#include <sdm/core/state/jjhistory_tree.tpp>