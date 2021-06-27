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

#include <sdm/core/state/state.hpp>
#include <sdm/utils/struct/tree.hpp>
#include <sdm/core/state/interface/history_interface.hpp>

namespace sdm
{

    /**
     * @class HistoryTree
     * 
     * @brief 
     * 
     */
    class HistoryTree : virtual public HistoryInterface, public Tree<std::shared_ptr<Observation>>
    {
    protected:
        /*!
         *  @brief  Expands the tree using truncated expand method
         *  @param  data the data of the expanded node
         *  @param  backup wheter the node is marked or not
         *  @return the truncated expanded tree
         */
        template <typename output>
        std::shared_ptr<output> truncatedExpand(const std::shared_ptr<Observation> &observation, bool backup);

    public:
        using value_type = typename Tree<std::shared_ptr<Observation>>::value_type;
        /*!
         *  @brief  Default constructor.
         *  This constructor builds a default and empty tree.
         */
        HistoryTree();

        /**
         * @brief Construct a new truncated tree object
         * 
         * @param data the value of the origin 
         */
        HistoryTree(number max_depth);

        /*!
         *  @brief  constructor
         *  @param  parent   the parent tree
         *  @param  item     the item
         *  @param  backup wheter the node is marked or not
         *  This constructor builds a tree with a given parent and item.
         */
        HistoryTree(std::shared_ptr<HistoryTree> parent, const std::shared_ptr<Observation> &item);

        std::shared_ptr<HistoryInterface> getPreviousHistory();

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
        template <typename output = HistoryTree>
        std::shared_ptr<output> expand(const std::shared_ptr<Observation> &observation, bool backup = true);
        std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Observation> &observation, bool backup = true);

        /**
         * @brief Get the horizon
         * 
         * @return number 
         */
        number getHorizon() const;

        std::string str() const;
        std::string short_str() const;

        std::shared_ptr<Item> getPointer();
        std::shared_ptr<HistoryTree> getptr();

        std::shared_ptr<HistoryTree> getParent() const;
        std::shared_ptr<HistoryTree> getOrigin();
        std::vector<std::shared_ptr<HistoryTree>> getChildren() const;
        std::shared_ptr<HistoryTree> getChild(const std::shared_ptr<Observation> &child_item) const;

        friend std::ostream &operator<<(std::ostream &os, HistoryTree &i_hist)
        {
            os << i_hist.str();
            return os;
        }

        template <class Archive>
        void serialize(Archive &archive, const unsigned int);
    };

} // namespace sdm
#include <sdm/core/state/history_tree.tpp>
