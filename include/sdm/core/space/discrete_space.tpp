#include <sdm/core/space/discrete_space.hpp>
#include <sdm/common.hpp>

namespace sdm
{
    template <typename TItem>
    DiscreteSpaceBase<TItem>::DiscreteSpaceBase() : num_items_(0) {}

    template <typename TItem>
    DiscreteSpaceBase<TItem>::DiscreteSpaceBase(const std::vector<TItem> &items) : num_items_(items.size())
    {
        for (number i = 0; i < this->num_items_; i++)
        {
            this->all_items_.insert(items_bimap_value(i, items[i]));
        }
    }

    template <typename TItem>
    DiscreteSpaceBase<TItem>::DiscreteSpaceBase(const DiscreteSpaceBase<TItem> &copy)
    {
        *this = copy;
    }

    template <typename TItem>
    DiscreteSpaceBase<TItem>::DiscreteSpaceBase(std::initializer_list<TItem> vals) : DiscreteSpaceBase(std::vector<TItem>(vals))
    {
    }

    template <typename TItem>
    bool DiscreteSpaceBase<TItem>::isDiscrete() const
    {
        return true;
    }

    template <typename TItem>
    TItem DiscreteSpaceBase<TItem>::sample() const
    {
        assert(!this->all_items_.empty());
        std::uniform_int_distribution<int> distr(0, this->all_items_.size() - 1);
        return this->all_items_.left.at(distr(common::global_urng()));
    }

    template <typename TItem>
    std::vector<TItem> DiscreteSpaceBase<TItem>::getAll()
    {
        std::vector<TItem> v;
        for (auto &it : this->all_items_.left)
        {
            v.push_back(it.second);
        }
        return v;
    }

    template <typename TItem>
    number DiscreteSpaceBase<TItem>::getNumItems() const
    {
        return this->num_items_;
    }

    template <typename TItem>
    TItem DiscreteSpaceBase<TItem>::getItem(number index) const
    {
        return this->all_items_.left.at(index);
    }

    template <typename TItem>
    number DiscreteSpaceBase<TItem>::getItemIndex(const TItem &item) const
    {
        return this->all_items_.right.at(item);
    }

    template <typename TItem>
    std::vector<number> DiscreteSpaceBase<TItem>::getDim() const
    {
        return {1};
    }

    template <typename TItem>
    std::string DiscreteSpaceBase<TItem>::str() const
    {
        std::ostringstream res;
        res << "DiscreteSpace(" << this->getNumItems() << ")";
        res << "[";
        for (number i = 0; i < this->getNumItems(); i++)
        {
            res << "" << this->getItem(i) << " ";
        }
        res << "]";
        return res.str();
    }

    template <typename TItem>
    DiscreteSpaceBase<TItem> &DiscreteSpaceBase<TItem>::operator=(const DiscreteSpaceBase<TItem> &sp)
    {
        this->all_items_.clear();
        this->num_items_ = sp.getNumItems();
        for (number idx = 0; idx < this->num_items_; ++idx)
        {
            this->all_items_.insert(items_bimap_value(idx, sp.getItem(idx)));
        }
        return *this;
    }

    template <typename TItem>
    bool DiscreteSpaceBase<TItem>::operator==(const DiscreteSpaceBase &sp) const
    {
        if (this->getNumItems() != sp.getNumItems())
        {
            return false;
        }
        else
        {
            for (number id = 0; id < this->getNumItems(); id++)
            {
                if (this->getItem(id) != sp.getItem(id))
                {
                    return false;
                }
            }
            return true;
        }
    }

    template <typename TItem>
    bool DiscreteSpaceBase<TItem>::operator!=(const DiscreteSpaceBase &sp) const
    {
        return !(operator==(sp));
    }

    DiscreteSpace::DiscreteSpace() : DiscreteSpaceBase<number>()
    {
    }

    DiscreteSpace::DiscreteSpace(int number_item)
    {
        std::vector<number> l(number_item);
        std::iota(l.begin(), l.end(), 0);
        *this = DiscreteSpace(l);
    }

    DiscreteSpace::DiscreteSpace(const std::vector<number> &items) : DiscreteSpaceBase<number>(items)
    {
    }

    DiscreteSpace::DiscreteSpace(const DiscreteSpace &copy)
    {
        *this = copy;
    }

    DiscreteSpace::DiscreteSpace(std::initializer_list<number> vals) : DiscreteSpace(std::vector<number>(vals))
    {
    }

} // namespace sdm
