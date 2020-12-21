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

#include <sdm/utils/struct/tree.hpp>

namespace sdm
{

    /**
     * @class HistoryTree
     * 
     * @brief 
     * 
     * @tparam T 
     */
    template <typename T>
    class HistoryTree : public Tree<T>
    {
    protected:
        /*!
         *  @fn     std::shared_ptr<Tree> const truncated_expand(const T &data);
         *  @brief  Expands the tree using truncated expand method
         *  @param  data the data of the expanded node
         *  @return the truncated expanded tree
         */
        // HistoryTree * truncated_expand(const T &data);

    public:
        /*!
         *  @fn     HistoryTree()
         *  @brief  Default constructor.
         *  This constructor builds a default and empty tree.
         */
        HistoryTree();

        /*!
         *  @fn     HistoryTree(std::shared_ptr<HistoryTree>, const T&, bool = true)
         *  @brief  constructor
         *  @param  parent   the parent tree
         *  @param  item     the item
         *  @param  backup wheter the node is marked or not
         *  This constructor builds a tree with a given parent and item.
         */
        HistoryTree(HistoryTree<T> *parent, const T &item, bool backup = true);

        /*!
         *  @fn     ~HistoryTree()
         *  @brief  destructor
         *
         *  This destructor recursively destroy the entire tree (i.e. node item and its children). Bottom up.
         */
        ~HistoryTree();

        /*!
         *  @fn     std::shared_ptr<Tree> const expand(const T &data);
         *  @brief  Expands the tree
         *  @param  data the data of the expanded node
         *  @return the expanded tree
         *
         *  If child leading from the item previously exists, the method return
         *  that child. Otherwise, it expands the tree by adding an item at the
         *  current leaf of the tree and creating if necessary a corresponding
         *  child. The constructed child is returned.
         */
        HistoryTree<T> *expand(const T &data);

        /*!
         *  @fn     void initTree(const std::vector<std::shared_ptr<HistoryTree>>&, const number&);
         *  @param  subtrees
         *  @param  horizon the planning horizon
         *  @brief  Initializes the parameters of a history tree
         */
        void initTree(HistoryTree<T> *subtrees, const number &horizon);
    };

} // namespace sdm