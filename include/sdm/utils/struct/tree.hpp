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

#include <unordered_set>
#include <unordered_map>
#include <memory>

#include <sdm/utils/struct/node.hpp>
#include <sdm/types.hpp>
#include <sdm/tools.hpp>

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
    class Tree
    {
    protected:
        //! @brief maximum length of the tree
        number length_limit_;

        //! @brief depth of the tree
        number depth_ = 0;

        //! @brief data of the current node
        T data_;

        //! @brief root of the tree and parent node
        Tree<T> *origin_ = nullptr;
        Tree<T> *parent_ = nullptr;

        //! @brief mapping of items to successor trees
        std::unordered_map<T, Tree<T> *> children_;

    public:
        /**
         * @brief Construct a new Base Tree object
         * 
         */
        Tree();

        /**
         * @brief Construct a new Base Tree object
         * 
         */
        Tree(const T &data);

        /**
         * @brief Construct a new Base Tree object
         * 
         * @param parent 
         * @param data 
         * @param is_marked 
         */
        Tree(Tree *parent, const T &data, bool backup = true);

        /*!
         *  @fn     ~Tree()
         *  @brief  Destructor of Tree (that's bad). 
         *
         *  This destructor recursively all, children and the item from the tree, bottom up.
         */
        ~Tree();

        const T &getData() const;

        number getNumChildren() const;

        Tree<T> *getChild(const T &child_item) const;

        std::vector<Tree<T> *> getChildren() const;

        void addChild(const T &child_item);

        void addChildren(const std::vector<T> &child_items);

        Tree<T> *getParent() const;

        Tree<T> *getOrigin() const;

        number getDepth() const;

        number getLengthLimit() const;

        void setLengthLimit(number) const;

        friend std::ostream &operator<<(std::ostream &os, Tree<T> tree)
        {
            os << sdm::tools::addIndent("", tree.getDepth());
            os << "<tree address=\"" << &tree << "\" size=\"" << tree.getNumChildren() << "\"  horizon=\"" << tree.getDepth() << "\">" << std::endl;
            os << sdm::tools::addIndent("<data>", tree.getDepth() + 1) << std::endl;
            os << sdm::tools::addIndent("", tree.getDepth() + 2) << tree.getData() << std::endl;
            os << sdm::tools::addIndent("</data>", tree.getDepth() + 1) << std::endl;
            for (auto child : tree.getChildren())
            {
                os << *child << std::endl;
            }
            os << sdm::tools::addIndent("", tree.getDepth());
            os << "</tree>" << std::endl;
            // }
            return os;
        }
    };

} // namespace sdm

#include <sdm/utils/struct/tree.tpp>