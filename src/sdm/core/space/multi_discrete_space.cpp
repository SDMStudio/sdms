#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/variations.hpp>

namespace sdm
{
    MultiDiscreteSpace::MultiDiscreteSpace()
    {
    }

    MultiDiscreteSpace::MultiDiscreteSpace(const std::vector<DiscreteSpace> &spaces)
    {
        this->setSpaces(spaces);
    }

    MultiDiscreteSpace::MultiDiscreteSpace(const std::vector<std::shared_ptr<DiscreteSpace>> &spaces)
    {
        this->setSpaces(spaces);
    }

    MultiDiscreteSpace::MultiDiscreteSpace(const std::vector<std::vector<std::shared_ptr<Item>>> &values)
    {
        this->setSpaces(values);
    }

    MultiDiscreteSpace::MultiDiscreteSpace(const MultiDiscreteSpace &copy) : MultiDiscreteSpace(copy.getSpaces()) {}

    template <bool TBool>
    MultiDiscreteSpace::MultiDiscreteSpace(const std::enable_if_t<TBool, std::vector<std::shared_ptr<Item>>> &num_items)
    {
        this->setSpaces(num_items);
    }

    // number MultiDiscreteSpace::joint2single(const std::vector<std::shared_ptr<Item>> &jel) const
    // {
    //     return this->getJointItemIndex(jel);
    // }

    // std::vector<std::shared_ptr<Item>> MultiDiscreteSpace::single2joint(number jel) const
    // {
    //     return this->getJointItem(jel);
    // }

    // number MultiDiscreteSpace::getNumJointItems() const
    // {
    //     return this->getNumItems();
    // }

    number MultiDiscreteSpace::getNumSpaces() const
    {
        return this->spaces_.size();
    }

    std::shared_ptr<DiscreteSpace> MultiDiscreteSpace::getSpace(number index) const
    {
        return this->spaces_[index];
    }

    number MultiDiscreteSpace::getItemIndex(number ag_id, const std::shared_ptr<Item> &item) const
    {
        return this->getSpace(ag_id)->getItemIndex(item);
    }

    std::shared_ptr<Item> MultiDiscreteSpace::getJointItem(number idx) const
    {
        return DiscreteSpace::getItem(idx);
    }

    std::shared_ptr<Item> MultiDiscreteSpace::getItem(number ag_id, number el_id) const
    {
        return this->getSpace(ag_id)->getItem(el_id);
    }

    template <bool TBool>
    void MultiDiscreteSpace::setSpaces(const std::enable_if_t<TBool, std::vector<std::shared_ptr<Item>>> &num_items)
    {
        this->num_items_ = 1;
        this->spaces_.clear();

        for (number ag = 0; ag < num_items.size(); ++ag)
        {
            this->spaces_.push_back(std::shared_ptr<DiscreteSpace>(new DiscreteSpace(num_items[ag])));
            this->num_items_ *= num_items[ag];
        }
        this->generateJointItems();
    }

    void MultiDiscreteSpace::setSpaces(const std::vector<std::vector<std::shared_ptr<Item>>> &e_names)
    {

        this->num_items_ = 1;
        this->spaces_.clear();

        for (number ag = 0; ag < e_names.size(); ++ag)
        {
            this->spaces_.push_back(std::shared_ptr<DiscreteSpace>(new DiscreteSpace(e_names[ag])));
            this->num_items_ *= e_names[ag].size();
        }
        this->generateJointItems();
    }

    void MultiDiscreteSpace::setSpaces(const std::vector<DiscreteSpace> &spaces)
    {
        this->num_items_ = 1;
        this->spaces_.clear();

        for (number ag = 0; ag < spaces.size(); ++ag)
        {
            this->spaces_.push_back(std::shared_ptr<DiscreteSpace>(new DiscreteSpace(spaces[ag])));
            this->num_items_ *= spaces[ag].getNumItems();
        }
        this->generateJointItems();
    }

    void MultiDiscreteSpace::setSpaces(const std::vector<std::shared_ptr<DiscreteSpace>> &spaces)
    {
        this->num_items_ = 1;
        this->spaces_.clear();

        for (number ag = 0; ag < spaces.size(); ++ag)
        {
            this->spaces_.push_back(spaces[ag]);
            this->num_items_ *= spaces[ag]->getNumItems();
        }
        this->generateJointItems();
    }

    void MultiDiscreteSpace::generateJointItems()
    {
        this->all_items_.clear();
        this->list_items_.clear();
        // Build a vector of vector of items to fit with Variation construct
        std::vector<std::vector<std::shared_ptr<Item>>> v_possible_items;
        for (auto sp : this->getSpaces())
        {
            v_possible_items.push_back(sp->getAll());
        }

        // Generate joint items and store in containers
        number counter = 0;
        Variations<std::shared_ptr<Joint<std::shared_ptr<Item>>>> vars(v_possible_items);
        for (auto v = vars.begin(); v != vars.end(); v = vars.next())
        {
            this->all_items_.insert(jitems_bimap_value(counter, *v));
            this->list_items_.push_back(*v);
            counter++;
        }
    }

    std::vector<std::shared_ptr<Joint<std::shared_ptr<Item>>>> MultiDiscreteSpace::getAll() const
    {
        return this->list_items_;
    }

    number MultiDiscreteSpace::getJointItemIndex(std::shared_ptr<Joint<std::shared_ptr<Item>>> &jitem) const
    {
        return DiscreteSpace::getItemIndex(jitem);
    }

    number MultiDiscreteSpace::getJointItemIndex(const std::vector<std::shared_ptr<Item>> &jitem) const
    {
        return DiscreteSpace::getItemIndex(jitem);
    }

    MultiDiscreteSpace &MultiDiscreteSpace::operator=(const MultiDiscreteSpace &sp)
    {
        this->setSpaces(sp.getSpaces());
        return *this;
    }

    bool MultiDiscreteSpace::operator==(const MultiDiscreteSpace &other) const
    {
        return DiscreteSpace::operator==(other);
    }

    bool MultiDiscreteSpace::operator!=(const MultiDiscreteSpace &other) const
    {
        return this->operator!=(other);
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

    bool MultiDiscreteSpace::contains(const std::vector<std::shared_ptr<Item>> &item) const
    {
        return std::find(this->list_items_.begin(), this->list_items_.end(), item) != this->list_items_.end() ? true : false;
    }

} // namespace sdm
