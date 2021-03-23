#include <sdm/utils/struct/tree.hpp>

namespace sdm
{
    template <typename T>
    Tree<T>::Tree() : depth_(0)
    {
        this->is_origin = true;
    }

    template <typename T>
    Tree<T>::Tree(number max_depth) : depth_(0), max_depth_(max_depth)
    {
        this->is_origin = true;
    }

    template <typename T>
    Tree<T>::Tree(std::shared_ptr<Tree<T>> parent, const T &data) : data_(data), max_depth_(parent->getMaxDepth()), depth_(parent->getDepth() + 1)
    {
        this->parent_ = parent;
        if (parent->isOrigin())
        {
            this->origin_ = parent;
        }
        else {
            this->origin_ = parent->getOrigin();
        }
    }

    template <typename T>
    Tree<T>::~Tree()
    {
    }

    template <typename T>
    bool Tree<T>::isOrigin() const
    {
        return this->is_origin;
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
    std::shared_ptr<Tree<T>> Tree<T>::getChild(const T &child) const
    {
        return this->children_.at(child);
    }

    template <typename T>
    std::vector<std::shared_ptr<Tree<T>>> Tree<T>::getChildren() const
    {
        std::vector<std::shared_ptr<Tree<T>>> value;
        for (auto me : this->children_)
        {
            value.push_back(me.second);
        }
        return value;
    }

    template <typename T>
    void Tree<T>::addChild(const T &child_item)
    {
        assert(this->getDepth() < this->getMaxDepth());
        if (this->children_.find(child_item) == this->children_.end())
        {
            this->children_.emplace(child_item, std::make_shared<Tree<T>>(this->shared_from_this(), child_item));
        }
    }

    template <typename T>
    void Tree<T>::addChildren(const std::vector<T> &child_items)
    {
        for (int i = 0; i < child_items.size(); i++)
        {
            this->addChild(child_items[i]);
        }
    }

    template <typename T>
    std::shared_ptr<Tree<T>> Tree<T>::getOrigin()
    {
        if (this->isOrigin())
        {
            return this->shared_from_this();
        }
        else
        {
            return this->origin_;
        }
    }

    template <typename T>
    std::shared_ptr<Tree<T>> Tree<T>::getParent() const
    {
        return this->parent_.lock();
    }

    template <typename T>
    number Tree<T>::getDepth() const
    {
        return this->depth_;
    }

    template <typename T>
    number Tree<T>::getMaxDepth() const
    {
        return this->max_depth_;
    }
} // namespace sdm