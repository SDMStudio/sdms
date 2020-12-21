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
    class JointHistoryTree : public HistoryTree<Joint<T>>, public Joint<HistoryTree<T>*>
    {
    protected:
       

    public:
        /*!
         *  @fn     JointHistoryTree()
         *  @brief  Default constructor.
         *  This constructor builds a default and empty tree.
         */
        JointHistoryTree();

        /*!
         *  @fn     JointHistoryTree(std::shared_ptr<JointHistoryTree>, const T&, bool = true)
         *  @brief  constructor
         *  @param  parent   the parent tree
         *  @param  item     the item
         *  @param  is_marked wheter the node is marked or not
         *  This constructor builds a tree with a given parent and item.
         */
        JointHistoryTree(std::shared_ptr<JointHistoryTree> parent, const T &item, bool is_marked = true);

        /*!
         *  @fn     JointHistoryTree(const std::vector<T>&, const std::vector<action>&)
         *  @brief  constructor
         *  @param  const std::vector<T>&  parameters of a function
         *  @param  const std::vector<action>& values of that same function
         *  This constructor builds a parametric JointHistoryTree using parameters and corresponding values.
         */
        JointHistoryTree(const std::vector<T> &, const std::vector<action> &);

        /*!
         *  @fn     ~JointHistoryTree()
         *  @brief  destructor
         *
         *  This destructor recursively destroy the entire tree (i.e. node item and its children). Bottom up.
         */
        ~JointHistoryTree();

        HistoryTree<T> getHistory(number ag_id);
    };

} // namespace sdm