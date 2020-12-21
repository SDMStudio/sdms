#include <sdm/utils/struct/tree.hpp>

namespace sdm
{
    template <typename T>
    Tree<T>::Tree() : depth_(0), parent_(nullptr)
    {
        origin_ = this;
    }

    template <typename T>
    Tree<T>::Tree(const T &data) : depth_(0), parent_(nullptr), data_(data)
    {
        origin_ = this;
    }

    template <typename T>
    Tree<T>::Tree(Tree *parent, const T &data, bool backup) : parent_(parent), data_(data), origin_(parent->getOrigin()), depth_(parent->getDepth() + 1)
    {
        if (backup)
        {
            parent->children_.emplace(this->data_, this);
        }
    }

    template <typename T>
    Tree<T>::~Tree()
    {
        // std::pair<T, Tree<T> *> me; // what a map<int, int> is made of
        // for (auto me : this->children_)
        // {
        //     if (me.second != NULL)
        //     {
        //         delete me.second;
        //     }
        // }
    }

    template <typename T>
    const T &Tree<T>::getData() const
    {
        return this->data_;
    }

    template <typename T>
    number Tree<T>::getNumChildren() const
    {
        return this->children_.empty() ? 0 : this->children_.size();
    }

    template <typename T>
    Tree<T> *Tree<T>::getChild(const T &child) const
    {
        return this->children_.at(child);
    }

    template <typename T>
    std::vector<Tree<T> *> Tree<T>::getChildren() const
    {
        std::pair<T, Tree<T> *> me; // what a map<int, int> is made of
        std::vector<Tree<T> *> value;
        for (auto me : this->children_)
        {
            value.push_back(me.second);
        }
        return value;
    }

    template <typename T>
    void Tree<T>::addChild(const T &child_item)
    {
        this->children_.emplace(child_item, new Tree<T>(this, child_item, false));
    }

    template <typename T>
    void Tree<T>::addChildren(const std::vector<T> &child_items)
    {
        for (int i = 0; i < child_items.size(); i++)
        {
            this->children_.emplace(child_items[i], new Tree<T>(this, child_items[i], false));
        }
    }

    template <typename T>
    Tree<T> *Tree<T>::getOrigin() const
    {
        return this->origin_;
    }

    template <typename T>
    Tree<T> *Tree<T>::getParent() const
    {
        return this->parent_;
    }

    template <typename T>
    number Tree<T>::getDepth() const
    {
        return this->depth_;
    }

    template <typename T>
    number Tree<T>::getLengthLimit() const
    {
        return this->length_limit_;
    }
    template <typename T>
    void Tree<T>::setLengthLimit(number length_limit) const
    {
        this->length_limit_ = length_limit;
    }
} // namespace sdm