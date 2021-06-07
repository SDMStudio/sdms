#include <sdm/core/space/discrete_space.hpp>
#include <sdm/common.hpp>

namespace sdm
{
    DiscreteSpace::DiscreteSpace() : num_items_(0) {}

    DiscreteSpace::DiscreteSpace(const std::vector<std::shared_ptr<Item>> &items) : num_items_(items.size()), list_items_(items)
    {
        for (number i = 0; i < this->num_items_; i++)
        {
            this->all_items_.insert(items_bimap_value(i, items[i]));
        }
    }

    template <typename T>
    DiscreteSpace::DiscreteSpace(const std::vector<T> &items)
    {
        std::vector<std::shared_ptr<Item>> titems(items.begin(), items.end());
        *this = DiscreteSpace(titems);
    }

    DiscreteSpace::DiscreteSpace(std::initializer_list<std::shared_ptr<Item>> vals) : DiscreteSpace(std::vector<std::shared_ptr<Item>>(vals))
    {
    }

    DiscreteSpace::DiscreteSpace(const DiscreteSpace &copy)
        : num_items_(copy.num_items_),
          all_items_(copy.all_items_),
          list_items_(copy.list_items_)
    {
    }

    template <bool TBool>
    DiscreteSpace::DiscreteSpace(std::enable_if_t<TBool, int> num_items)
    {
        std::vector<number> l(num_items);
        std::iota(l.begin(), l.end(), 0);
        *this = DiscreteSpace(l);
    }

    bool DiscreteSpace::isDiscrete() const
    {
        return true;
    }

    std::shared_ptr<Item> DiscreteSpace::sample() const
    {
        assert(!this->all_items_.empty());
        std::uniform_int_distribution<int> distr(0, this->all_items_.size() - 1);
        return this->all_items_.left.at(distr(common::global_urng()));
    }

    std::vector<std::shared_ptr<Item>> DiscreteSpace::getAll() const
    {
        return this->list_items_;
    }

    number DiscreteSpace::getNumItems() const
    {
        return this->num_items_;
    }

    std::shared_ptr<Item> DiscreteSpace::getItem(number index) const
    {
        return this->all_items_.left.at(index);
    }

    number DiscreteSpace::getItemIndex(const std::shared_ptr<Item> &item) const
    {
        return this->all_items_.right.at(item);
    }

    std::vector<number> DiscreteSpace::getDim() const
    {
        return {1};
    }

    std::string DiscreteSpace::str() const
    {
        std::ostringstream res;
        res << "DiscreteSpace(" << this->getNumItems() << ")";
        res << this->list_items_;
        return res.str();
    }

    bool DiscreteSpace::operator==(const DiscreteSpace &sp) const
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

    bool DiscreteSpace::operator!=(const DiscreteSpace &sp) const
    {
        return !(operator==(sp));
    }

    bool DiscreteSpace::contains(const std::shared_ptr<Item> &item) const
    {
        return std::find(this->list_items_.begin(), this->list_items_.end(), item) != this->list_items_.end() ? true : false;
    }

} // namespace sdm
