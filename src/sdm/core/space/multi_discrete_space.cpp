#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/variations.hpp>
#include <sdm/exception.hpp>
#include <sdm/utils/struct/iterator/multi_iterator.hpp>

namespace sdm
{
    MultiDiscreteSpace::MultiDiscreteSpace()
    {
    }

    MultiDiscreteSpace::MultiDiscreteSpace(const std::vector<std::shared_ptr<Space>> &spaces, bool store_items) : store_items_(store_items)
    {
        this->setSpaces(spaces);
    }

    MultiDiscreteSpace::MultiDiscreteSpace(const std::vector<std::vector<std::shared_ptr<Item>>> &values, bool store_items) : store_items_(store_items)
    {
        this->setSpaces(values);
    }

    MultiDiscreteSpace::MultiDiscreteSpace(const MultiDiscreteSpace &copy) : MultiDiscreteSpace(static_cast<std::vector<std::shared_ptr<Space>>>(copy)) {}

    template <bool TBool>
    MultiDiscreteSpace::MultiDiscreteSpace(const std::enable_if_t<TBool, std::vector<std::shared_ptr<Item>>> &num_items)
    {
        this->setSpaces(num_items);
    }

    bool MultiDiscreteSpace::isStoringItems() const
    {
        return this->store_items_;
    }

    void MultiDiscreteSpace::storeItems(bool store_items)
    {
        this->store_items_ = store_items;
    }

    bool MultiDiscreteSpace::isGenerated()
    {
        return !this->list_items_.empty();
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
    }

    void MultiDiscreteSpace::generateJointItems()
    {
        if (this->isStoringItems())
        {
            this->all_items_.clear();
            this->list_items_.clear();
            // Build a vector of vector of items to fit with Variation construct
            std::vector<std::vector<std::shared_ptr<Item>>> v_possible_items;
            for (number space_id = 0; space_id < this->getNumSpaces(); space_id++)
            {
                v_possible_items.push_back(this->cast(this->getSpace(space_id))->getAll());
            }

            // Generate joint items and store in containers
            number counter = 0;
            Variations<Joint<std::shared_ptr<Item>>> vars(v_possible_items);
            for (std::shared_ptr<Item> v = vars.begin(); v != vars.end(); v = vars.next())
            {
                this->all_items_.insert(jitems_bimap_value(counter, v));
                this->list_items_.push_back(v);
                counter++;
            }
        }
    }

    std::vector<std::shared_ptr<Item>> MultiDiscreteSpace::getAll()
    {
        if (!this->store_items_)
        {
            throw sdm::exception::Exception("You are trying to generate all items of space with parameter 'store_items=false'\n#> Use for loop on the space or set parameter store items to 'true'\n");
        }
        else
        {
            if (this->list_items_.empty())
            {
                this->generateJointItems();
            }
            return this->list_items_;
        }
    }

    number MultiDiscreteSpace::getJointItemIndex(std::shared_ptr<Joint<std::shared_ptr<Item>>> &jitem) const
    {
        return DiscreteSpace::getItemIndex(jitem);
    }

    // number MultiDiscreteSpace::getJointItemIndex(const std::vector<std::shared_ptr<Item>> &jitem) const
    // {
    //     return DiscreteSpace::getItemIndex(jitem);
    // }

    std::shared_ptr<MultiDiscreteSpace::iterator_type> MultiDiscreteSpace::begin()
    {
        if (this->isStoringItems())
        {
            if (!this->isGenerated())
            {
                this->generateJointItems();
            }
            return DiscreteSpace::begin();
        }
        else
        {
            std::vector<std::shared_ptr<iterator_type>> begin_iterators, end_iterators;
            for (number space_id = 0; space_id < this->getNumSpaces(); space_id++)
            {
                begin_iterators.push_back(this->getSpace(space_id)->begin());
                end_iterators.push_back(this->getSpace(space_id)->end());
            }
            return std::make_shared<sdm::iterator::MultiIterator>(begin_iterators, end_iterators, begin_iterators);
        }
    }

    std::shared_ptr<MultiDiscreteSpace::iterator_type> MultiDiscreteSpace::end()
    {
        if (this->isStoringItems())
        {
            if (!this->isGenerated())
            {
                this->generateJointItems();
            }
            return DiscreteSpace::end();
        }
        else
        {
            std::vector<std::shared_ptr<iterator_type>> begin_iterators, end_iterators;
            for (number space_id = 0; space_id < this->getNumSpaces(); space_id++)
            {
                begin_iterators.push_back(this->getSpace(space_id)->begin());
                end_iterators.push_back(this->getSpace(space_id)->end());
            }
            return std::make_shared<sdm::iterator::MultiIterator>(begin_iterators, end_iterators, end_iterators);
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
