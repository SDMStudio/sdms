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
    class JointHistoryTree : public HistoryTree<Joint<T>>
    {
    protected:

        void addIndivHist(std::shared_ptr<HistoryTree<T>> ihist);
    public:
        using ihistory_type = std::shared_ptr<HistoryTree<T>>;
        Joint<std::shared_ptr<HistoryTree<T>>> indiv_hist;

        /*!
         *  @fn     JointHistoryTree()
         *  @brief  Default constructor.
         *  This constructor builds a default and empty tree.
         */
        JointHistoryTree();

        /**
         * @brief Construct a new joint history tree object (the origin)
         * 
         * @param n_agents the number of agent
         */
        JointHistoryTree(number n_agents);

        JointHistoryTree(number n_agents, number max_depth);

        JointHistoryTree(std::vector<ihistory_type> trees);

        /*!
         *  @fn     JointHistoryTree(std::shared_ptr<JointHistoryTree>, const T&, bool = true)
         *  @brief  constructor
         *  @param  parent   the parent tree
         *  @param  item     the item
         *  @param  backup wheter the node is marked or not
         *  This constructor builds a tree with a given parent and item.
         */
        JointHistoryTree(std::shared_ptr<JointHistoryTree<T>> parent, const Joint<T> &item);

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
        std::shared_ptr<JointHistoryTree<T>> expand(const Joint<T> &data, bool backup = true);

        std::shared_ptr<HistoryTree<T>> getIndividualHistory(number ag_id) const;
        std::vector<std::shared_ptr<HistoryTree<T>>> getIndividualHistories() const;

        friend std::ostream &operator<<(std::ostream &os, const JointHistoryTree &j_hist)
        {
            os << static_cast<HistoryTree<Joint<T>>>(j_hist);
            return os;
        }
    };

} // namespace sdm
#include <sdm/core/state/jhistory_tree.tpp>