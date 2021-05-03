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
         *  @fn     HistoryTree<T> *truncatedExpand(const T &data, bool backup);
         *  @brief  Expands the tree using truncated expand method
         *  @param  data the data of the expanded node
         *  @param  backup wheter the node is marked or not
         *  @return the truncated expanded tree
         */
        template <typename output = HistoryTree<T>>
        std::shared_ptr<output> truncatedExpand(const T &data, bool backup);

    public:
        /*!
         *  @fn     HistoryTree()
         *  @brief  Default constructor.
         *  This constructor builds a default and empty tree.
         */
        HistoryTree();

        /**
         * @brief Construct a new Tree object (the origin)
         * 
         * @param data the value of the origin 
         */
        HistoryTree(number max_depth);

        /*!
         *  @fn     HistoryTree(std::shared_ptr<HistoryTree>, const T&, bool = true)
         *  @brief  constructor
         *  @param  parent   the parent tree
         *  @param  item     the item
         *  @param  backup wheter the node is marked or not
         *  This constructor builds a tree with a given parent and item.
         */
        HistoryTree(std::shared_ptr<HistoryTree<T>> parent, const T &item);

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
        template <typename output = HistoryTree<T>>
        std::shared_ptr<output> expand(const T &data, bool backup = true);

        number getHorizon() const;

        std::string str();

        std::string short_str();

        std::shared_ptr<HistoryTree<T>> getptr();


        friend std::ostream &operator<<(std::ostream &os, HistoryTree &i_hist)
        {
            os << i_hist.str();
            return os;
        }
    };

} // namespace sdm
#include <sdm/core/state/history_tree.tpp>
