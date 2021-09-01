/**
 * @file tree.hpp
 * @author Jilles S. Dibangoye
 * @author David Albert
 * @brief Tree data structure
 * @version 0.1
 * @date 12/04/2016
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <memory>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/weak_ptr.hpp>

#include <sdm/types.hpp>
#include <sdm/tools.hpp>
#include <sdm/public/boost_serializable.hpp>

namespace sdm
{

    /**
     * @class Tree
     * 
     * @brief Generic Tree class 
     * 
     * @tparam T the type of the data contains in each node
     * 
     * 
     * Usage
     *         Tree<int> tree();
     *         tree.addChildren({3, 4, 5});
     *         tree.getChild(3).addChildren({9, 8, 7, 6});
     *         tree.getChild(5).addChildren({1, 3});
     * 
     */
    template <typename T>
    class Tree : public std::inheritable_enable_shared_from_this<Tree<T>>
    //public BoostSerializable<Tree<T>>
    {
    public:
        using value_type = T;
        /**
         * @brief Default constructor object
         * 
         */
        Tree();

        /**
         * @brief Construct a new Tree object (the origin)
         * 
         * @param data the value of the origin 
         */
        Tree(number max_depth);

        /**
         * @brief Construct a new Tree object
         * 
         * @param parent the parent
         * @param data the value of the node
         * @param backup if true, save the new tree as a child for its parent
         */
        Tree(std::shared_ptr<Tree<T>> parent, const T &data);

        /*!
         *  @fn     ~Tree()
         *  @brief  Destructor of Tree (that's bad). 
         *
         *  This destructor recursively all, children and the item from the tree, bottom up.
         */
        virtual ~Tree();

        bool isOrigin() const;

        const T &getData() const;

        number getNumChildren() const;

        std::shared_ptr<Tree<T>> getChild(const T &child_item) const;

        std::vector<std::shared_ptr<Tree<T>>> getChildren() const;

        void addChild(const T &child_item);

        void addChildren(const std::vector<T> &child_items);

        std::shared_ptr<Tree<T>> getParent() const;

        std::shared_ptr<Tree<T>> getOrigin();

        number getDepth() const;

        number getMaxDepth() const;

        void setMaxDepth(number) const;

        std::string str() const;

        std::shared_ptr<Tree<T>> getptr();

        template <class Archive>
        void serialize(Archive &archive, const unsigned int);

        friend std::ostream &operator<<(std::ostream &os, Tree<T> &tree)
        {
            os << tree.str();
            return os;
        }

    protected:
        //! @brief depth of the tree
        number depth_ = 0;

        //! @brief maximum length of the tree
        number max_depth_ = std::numeric_limits<number>::max();

        //! @brief data of the current node
        T data_;

        //! @brief the root of the tree
        std::weak_ptr<Tree<T>> origin_;

        //! @brief the parent node
        std::weak_ptr<Tree<T>> parent_;

        //! @brief mapping of items to successor trees
        std::map<T, std::shared_ptr<Tree<T>>> children_;

        bool is_origin = false;
    };

} // namespace sdm

#include <sdm/utils/struct/tree.tpp>