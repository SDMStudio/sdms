#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/variations.hpp>
#include <sdm/exception.hpp>
#include <sdm/utils/struct/iterator/combination_iterator.hpp>

namespace sdm
{
    template <typename TItem>
    MultiDiscreteSpace<TItem>::MultiDiscreteSpace()
    {
    }

    template <typename TItem>
    MultiDiscreteSpace<TItem>::MultiDiscreteSpace(const std::vector<std::shared_ptr<BaseSpace<TItem>>> &spaces, bool store_items)
    {
        this->storeItems(store_items);
        this->setSpaces(spaces);
    }

    template <typename TItem>
    MultiDiscreteSpace<TItem>::MultiDiscreteSpace(const std::vector<std::vector<TItem>> &values, bool store_items)
    {
        this->storeItems(store_items);
        this->setSpaces(values);
    }

    template <typename TItem>
    MultiDiscreteSpace<TItem>::MultiDiscreteSpace(const MultiDiscreteSpace &copy)
        : MultiDiscreteSpace(static_cast<std::vector<std::shared_ptr<BaseSpace<TItem>>>>(copy))
    {
        this->storeItems(copy.store_items_);
    }

    template <bool TBool>
    template <typename TItem>
    MultiDiscreteSpace<TItem>::MultiDiscreteSpace(const std::enable_if_t<TBool, std::vector<TItem>> &num_items)
    {
        this->setSpaces(num_items);
    }

    template <typename TItem>
    number MultiDiscreteSpace<TItem>::getNumSpaces() const
    {
        return this->size();
    }

    template <typename TItem>
    std::shared_ptr<Space> MultiDiscreteSpace<TItem>::getSpace(number index) const
    {
        return this->get(index);
    }

    template <typename TItem>
    number MultiDiscreteSpace<TItem>::getItemIndex(number ag_id, const TItem &item) const
    {
        return this->cast(this->getSpace(ag_id))->getItemIndex(item);
    }

    template <typename TItem>
    TItem MultiDiscreteSpace<TItem>::getJointItem(number idx) const
    {
        return DiscreteSpace::getItem(idx);
    }

    template <typename TItem>
    TItem MultiDiscreteSpace<TItem>::getItem(number idx) const
    {
        return DiscreteSpace::getItem(idx);
    }

    template <typename TItem>
    TItem MultiDiscreteSpace<TItem>::getItem(number ag_id, number el_id) const
    {
        return this->cast(this->getSpace(ag_id))->getItem(el_id);
    }

    template <typename TItem>
    template <bool TBool>
    void MultiDiscreteSpace<TItem>::setSpaces(const std::enable_if_t<TBool, std::vector<TItem>> &num_items)
    {
        this->num_items_ = 1;
        this->clear();

        for (number ag = 0; ag < num_items.size(); ++ag)
        {
            this->push_back(std::shared_ptr<DiscreteSpace>(new DiscreteSpace<TItem>(num_items[ag])));
            this->num_items_ *= num_items[ag];
        }
        this->generateItems();
    }

    template <typename TItem>
    void MultiDiscreteSpace<TItem>::setSpaces(const std::vector<std::vector<TItem>> &e_names)
    {
        this->num_items_ = 1;
        this->clear();

        for (number ag = 0; ag < e_names.size(); ++ag)
        {
            this->push_back(std::shared_ptr<DiscreteSpace>(new DiscreteSpace(e_names[ag])));
            this->num_items_ *= e_names[ag].size();
        }
        this->generateItems();
    }

    template <typename TItem>
    void MultiDiscreteSpace<TItem>::setSpaces(const std::vector<std::shared_ptr<Space>> &spaces)
    {
        this->num_items_ = 1;
        this->clear();

        for (number ag = 0; ag < spaces.size(); ++ag)
        {
            this->push_back(spaces[ag]);
            this->num_items_ *= this->cast(spaces[ag])->getNumItems();
        }
        this->generateItems();
    }

    template <typename TItem>
    number MultiDiscreteSpace<TItem>::getJointItemIndex(std::shared_ptr<Joint<TItem>> &jitem) const
    {
        return DiscreteSpace::getItemIndex(jitem);
    }

    template <typename TItem>
    // number MultiDiscreteSpace<TItem>::getJointItemIndex(const std::vector<TItem> &jitem) const
    // {
    //     return DiscreteSpace::getItemIndex(jitem);
    // }

    template <typename TItem>
    MultiDiscreteSpace<TItem>::iterator_type MultiDiscreteSpace<TItem>::begin()
    {
        if (this->isStoringItems())
        {
            if (!this->isGenerated())
            {
                this->generateItems();
            }
            return DiscreteSpace::begin();
        }
        else
        {
            std::vector<iterator_type> begin_iterators, current_iterators, end_iterators;
            for (number space_id = 0; space_id < this->getNumSpaces(); space_id++)
            {
                begin_iterators.push_back(this->getSpace(space_id)->begin());
                end_iterators.push_back(this->getSpace(space_id)->end());
            }
            return std::make_shared<sdm::iterator::CombinationIterator>(begin_iterators, end_iterators);
        }
    }

    template <typename TItem>
    MultiDiscreteSpace<TItem>::iterator_type MultiDiscreteSpace<TItem>::end()
    {
        if (this->isStoringItems())
        {
            if (!this->isGenerated())
            {
                this->generateItems();
            }
            return DiscreteSpace::end();
        }
        else
        {
            return std::make_shared<sdm::iterator::CombinationIterator>();
        }
    }

    template <typename TItem>
    MultiDiscreteSpace<TItem> &MultiDiscreteSpace<TItem>::operator=(const MultiDiscreteSpace &sp)
    {
        this->setSpaces(static_cast<std::vector<std::shared_ptr<BaseSpace<TItem>>>>(sp));
        return *this;
    }

    template <typename TItem>
    bool MultiDiscreteSpace<TItem>::operator==(const MultiDiscreteSpace &other) const
    {
        return DiscreteSpace::operator==(other);
    }

    template <typename TItem>
    bool MultiDiscreteSpace<TItem>::operator!=(const MultiDiscreteSpace &other) const
    {
        return (!this->operator==(other));
    }

    template <typename TItem>
    std::string MultiDiscreteSpace<TItem>::str() const
    {
        std::ostringstream res;
        res << "MultiDiscreteSpace(" << this->getNumSpaces() << ")";
        res << "[";
        for (number i = 0; i < this->getNumSpaces(); i++)
        {
            // res << "\n\t" << i << " : " << *this->getSpace(i);
            res << "\n\t" << i << " : "
                << "\t";
            res << *this->getSpace(i);
        }
        res << "\n]";
        return res.str();
    }

    template <typename TItem>
    bool MultiDiscreteSpace<TItem>::contains(const TItem &item) const
    {
        return std::find(this->list_items_.begin(), this->list_items_.end(), item) != this->list_items_.end() ? true : false;
    }

    template <typename TItem>
    inline std::shared_ptr<DiscreteSpace<TItem>> MultiDiscreteSpace<TItem>::cast(const std::shared_ptr<BaseSpace<TItem>> &space) const
    {
        return std::static_pointer_cast<DiscreteSpace<TItem>>(space);
    }

} // namespace sdm
