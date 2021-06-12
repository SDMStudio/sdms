#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/variations.hpp>
#include <sdm/exception.hpp>
#include <sdm/utils/struct/iterator/combination_iterator.hpp>

namespace sdm
{
    MultiDiscreteSpace::MultiDiscreteSpace()
    {
    }

    MultiDiscreteSpace::MultiDiscreteSpace(const std::vector<std::shared_ptr<Space>> &spaces, bool store_items)
    {
        this->storeItems(store_items);
        this->setSpaces(spaces);
    }

    MultiDiscreteSpace::MultiDiscreteSpace(const std::vector<std::vector<std::shared_ptr<Item>>> &values, bool store_items)
    {
        this->storeItems(store_items);
        this->setSpaces(values);
    }

    MultiDiscreteSpace::MultiDiscreteSpace(const MultiDiscreteSpace &copy)
        : MultiDiscreteSpace(static_cast<std::vector<std::shared_ptr<Space>>>(copy))
    {
        this->storeItems(copy.store_items_);
    }

    template <bool TBool>
    MultiDiscreteSpace::MultiDiscreteSpace(const std::enable_if_t<TBool, std::vector<std::shared_ptr<Item>>> &num_items)
    {
        this->setSpaces(num_items);
    }

    number MultiDiscreteSpace::getNumSpaces() const
    {
        return this->size();
    }

    std::shared_ptr<Space> MultiDiscreteSpace::getSpace(number index) const
    {
        return this->get(index);
    }

    number MultiDiscreteSpace::getItemIndex(number ag_id, const std::shared_ptr<Item> &item) const
    {
        return this->cast(this->getSpace(ag_id))->getItemIndex(item);
    }

    std::shared_ptr<Item> MultiDiscreteSpace::getJointItem(number idx) const
    {
        return DiscreteSpace::getItem(idx);
    }

    std::shared_ptr<Item> MultiDiscreteSpace::getItem(number idx) const
    {
        return DiscreteSpace::getItem(idx);
    }

    std::shared_ptr<Item> MultiDiscreteSpace::getItem(number ag_id, number el_id) const
    {
        return this->cast(this->getSpace(ag_id))->getItem(el_id);
    }

    template <bool TBool>
    void MultiDiscreteSpace::setSpaces(const std::enable_if_t<TBool, std::vector<std::shared_ptr<Item>>> &num_items)
    {
        this->num_items_ = 1;
        this->clear();

        for (number ag = 0; ag < num_items.size(); ++ag)
        {
            this->push_back(std::shared_ptr<DiscreteSpace>(new DiscreteSpace(num_items[ag])));
            this->num_items_ *= num_items[ag];
        }
        this->generateItems();
    }

    void MultiDiscreteSpace::setSpaces(const std::vector<std::vector<std::shared_ptr<Item>>> &e_names)
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

    void MultiDiscreteSpace::setSpaces(const std::vector<std::shared_ptr<Space>> &spaces)
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

    number MultiDiscreteSpace::getJointItemIndex(std::shared_ptr<Joint<std::shared_ptr<Item>>> &jitem) const
    {
        return DiscreteSpace::getItemIndex(jitem);
    }

    // number MultiDiscreteSpace::getJointItemIndex(const std::vector<std::shared_ptr<Item>> &jitem) const
    // {
    //     return DiscreteSpace::getItemIndex(jitem);
    // }

    MultiDiscreteSpace::iterator_type MultiDiscreteSpace::begin()
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
            std::cout << "begin iterators=" << begin_iterators << std::endl;
            std::cout << "end iterators=" << end_iterators << std::endl;
            return std::make_shared<sdm::iterator::CombinationIterator>(begin_iterators, end_iterators);
        }
    }

    MultiDiscreteSpace::iterator_type MultiDiscreteSpace::end()
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

    MultiDiscreteSpace &MultiDiscreteSpace::operator=(const MultiDiscreteSpace &sp)
    {
        this->setSpaces(static_cast<std::vector<std::shared_ptr<Space>>>(sp));
        return *this;
    }

    bool MultiDiscreteSpace::operator==(const MultiDiscreteSpace &other) const
    {
        return DiscreteSpace::operator==(other);
    }

    bool MultiDiscreteSpace::operator!=(const MultiDiscreteSpace &other) const
    {
        return (!this->operator==(other));
    }

    std::string MultiDiscreteSpace::str() const
    {
        std::ostringstream res;
        res << "MultiDiscreteSpace(" << this->getNumSpaces() << ")";
        res << "[";
        for (number i = 0; i < this->getNumSpaces(); i++)
        {
            res << "\n\t" << i << " : " << *this->getSpace(i);
        }
        res << "\n]";
        return res.str();
    }

    bool MultiDiscreteSpace::contains(const std::shared_ptr<Item> &item) const
    {
        return std::find(this->list_items_.begin(), this->list_items_.end(), item) != this->list_items_.end() ? true : false;
    }

    inline std::shared_ptr<DiscreteSpace> MultiDiscreteSpace::cast(const std::shared_ptr<Space> &space) const
    {
        return std::static_pointer_cast<DiscreteSpace>(space);
    }

} // namespace sdm
