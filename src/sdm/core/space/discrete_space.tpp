#include <sdm/core/space/discrete_space.hpp>
#include <sdm/common.hpp>

namespace sdm
{
    template <typename TItem>
    DiscreteSpace<TItem>::DiscreteSpace() : num_items_(0) {}

    template <typename TItem>
    DiscreteSpace<TItem>::DiscreteSpace(const std::vector<TItem> &items) : num_items_(items.size()), list_items_(items)
    {
        for (number i = 0; i < this->num_items_; i++)
        {
            this->all_items_.insert(items_bimap_value(i, items[i]));
        }
    }

    template <typename TItem>
    DiscreteSpace<TItem>::DiscreteSpace(const DiscreteSpace<TItem> &copy)
    {
        *this = copy;
    }

    template <typename TItem>
    DiscreteSpace<TItem>::DiscreteSpace(std::initializer_list<TItem> vals) : DiscreteSpace(std::vector<TItem>(vals))
    {
    }

    template <typename TItem>
    template <bool TBool>
    DiscreteSpace<TItem>::DiscreteSpace(std::enable_if_t<TBool, int> num_items)
    {
        std::vector<number> l(num_items);
        std::iota(l.begin(), l.end(), 0);
        *this = DiscreteSpace<TItem>(l);
    }

    template <typename TItem>
    bool DiscreteSpace<TItem>::isDiscrete() const
    {
        return true;
    }

    template <typename TItem>
    TItem DiscreteSpace<TItem>::sample() const
    {
        assert(!this->all_items_.empty());
        std::uniform_int_distribution<int> distr(0, this->all_items_.size() - 1);
        return this->all_items_.left.at(distr(common::global_urng()));
    }

    template <typename TItem>
    std::vector<TItem> DiscreteSpace<TItem>::getAll()
    {
        return this->list_items_;
    }

    template <typename TItem>
    number DiscreteSpace<TItem>::getNumItems() const
    {
        return this->num_items_;
    }

    template <typename TItem>
    TItem DiscreteSpace<TItem>::getItem(number index) const
    {
        return this->all_items_.left.at(index);
    }

    template <typename TItem>
    number DiscreteSpace<TItem>::getItemIndex(const TItem &item) const
    {
        return this->all_items_.right.at(item);
    }

    template <typename TItem>
    std::vector<number> DiscreteSpace<TItem>::getDim() const
    {
        return {1};
    }

    template <typename TItem>
    std::string DiscreteSpace<TItem>::str() const
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
    DiscreteSpace<TItem> &DiscreteSpace<TItem>::operator=(const DiscreteSpace<TItem> &sp)
    {
        this->all_items_.clear();
        this->num_items_ = sp.getNumItems();
        for (number idx = 0; idx < this->num_items_; ++idx)
        {
            this->all_items_.insert(items_bimap_value(idx, sp.getItem(idx)));
            this->list_items_.push_back(sp.getItem(idx));
        }
        return *this;
    }

    template <typename TItem>
    bool DiscreteSpace<TItem>::operator==(const DiscreteSpace &sp) const
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
    bool DiscreteSpace<TItem>::operator!=(const DiscreteSpace &sp) const
    {
        return !(operator==(sp));
    }

    template <typename TItem>
    bool DiscreteSpace<TItem>::contains(const TItem &item) const
    {
        return std::find(this->list_items_.begin(), this->list_items_.end(),item) !=this->list_items_.end() ? true : false;
    }

} // namespace sdm
