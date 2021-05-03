#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/variations.hpp>

namespace sdm
{
    template <typename TItem>
    MultiDiscreteSpace<TItem>::MultiDiscreteSpace()
    {
    }

    template <typename TItem>
    MultiDiscreteSpace<TItem>::MultiDiscreteSpace(const std::vector<DiscreteSpace<TItem>> &spaces)
    {
        this->setSpaces(spaces);
        //this->generateJointItems();
    }

    template <typename TItem>
    MultiDiscreteSpace<TItem>::MultiDiscreteSpace(const std::vector<std::shared_ptr<DiscreteSpace<TItem>>> &spaces)
    {
        this->setSpaces(spaces);
        //this->generateJointItems();
    }

    template <typename TItem>
    MultiDiscreteSpace<TItem>::MultiDiscreteSpace(const std::vector<std::vector<TItem>> &values)
    {
        this->setSpaces(values);
        //this->generateJointItems();
    }

    template <typename TItem>
    MultiDiscreteSpace<TItem>::MultiDiscreteSpace(const MultiDiscreteSpace<TItem> &copy) : MultiDiscreteSpace<TItem>(copy.getSpaces()) {}

    template <typename TItem>
    template <bool TBool>
    MultiDiscreteSpace<TItem>::MultiDiscreteSpace(const std::enable_if_t<TBool, std::vector<TItem>> &num_items)
    {
        this->setSpaces(num_items);
        //this->generateJointItems();
    }

    template <typename TItem>
    number MultiDiscreteSpace<TItem>::joint2single(const std::vector<TItem> &jel) const
    {
        return this->getJointItemIndex(jel);
    }

    template <typename TItem>
    std::vector<TItem> MultiDiscreteSpace<TItem>::single2joint(number jel) const
    {
        return this->getJointItem(jel);
    }

    template <typename TItem>
    number MultiDiscreteSpace<TItem>::getNumJointItems() const
    {
        return this->getNumItems();
    }

    template <typename TItem>
    number MultiDiscreteSpace<TItem>::getItemIndex(number ag_id, const TItem &item) const
    {
        return this->getSpace(ag_id)->getItemIndex(item);
    }

    template <typename TItem>
    Joint<TItem> MultiDiscreteSpace<TItem>::getJointItem(number idx) const
    {
        return DiscreteSpace<Joint<TItem>>::getItem(idx);
    }

    template <typename TItem>
    TItem MultiDiscreteSpace<TItem>::getItem(number ag_id, number el_id) const
    {
        return this->getSpace(ag_id)->getItem(el_id);
    }

    template <typename TItem>
    template <bool TBool>
    void MultiDiscreteSpace<TItem>::setSpaces(const std::enable_if_t<TBool, std::vector<TItem>> &num_items)
    {
        this->num_items_ = 1;
        this->spaces_.clear();

        for (number ag = 0; ag < num_items.size(); ++ag)
        {
            this->spaces_.push_back(std::shared_ptr<DiscreteSpace<TItem>>(new DiscreteSpace<TItem>(num_items[ag])));
            this->num_items_ *= num_items[ag];
        }
        this->generateJointItems();
    }

    template <typename TItem>
    void MultiDiscreteSpace<TItem>::setSpaces(const std::vector<std::vector<TItem>> &e_names)
    {

        this->num_items_ = 1;
        this->spaces_.clear();

        for (number ag = 0; ag < e_names.size(); ++ag)
        {
            this->spaces_.push_back(std::shared_ptr<DiscreteSpace<TItem>>(new DiscreteSpace<TItem>(e_names[ag])));
            this->num_items_ *= e_names[ag].size();
        }
        this->generateJointItems();
    }

    template <typename TItem>
    void MultiDiscreteSpace<TItem>::setSpaces(const std::vector<DiscreteSpace<TItem>> &spaces)
    {
        this->num_items_ = 1;
        this->spaces_.clear();

        for (number ag = 0; ag < spaces.size(); ++ag)
        {
            this->spaces_.push_back(std::shared_ptr<DiscreteSpace<TItem>>(new DiscreteSpace<TItem>(spaces[ag])));
            this->num_items_ *= spaces[ag].getNumItems();
        }
        this->generateJointItems();
    }

    template <typename TItem>
    void MultiDiscreteSpace<TItem>::setSpaces(const std::vector<std::shared_ptr<DiscreteSpace<TItem>>> &spaces)
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

    template <typename TItem>
    void MultiDiscreteSpace<TItem>::generateJointItems()
    {
        this->all_items_.clear();
        // Build a vector of vector of items to fit with Variation construct
        std::vector<std::vector<TItem>> v_possible_items;
        for (auto sp : this->getSpaces())
        {
            v_possible_items.push_back(sp->getAll());
        }

        // Generate joint items and store in containers
        number counter = 0;
        Variations<Joint<TItem>> vars(v_possible_items);
        for (auto v = vars.begin(); v != vars.end(); v = vars.next())
        {
            this->all_items_.insert(jitems_bimap_value(counter, *v));
            this->list_items_.push_back(*v);
            counter++;
        }
    }

    template <typename TItem>
    std::vector<Joint<TItem>> MultiDiscreteSpace<TItem>::getAll()
    {
        return this->list_items_;
    }

    template <typename TItem>
    number MultiDiscreteSpace<TItem>::getJointItemIndex(Joint<TItem> &jitem) const
    {
        return DiscreteSpace<Joint<TItem>>::getItemIndex(jitem);
    }

    template <typename TItem>
    number MultiDiscreteSpace<TItem>::getJointItemIndex(const std::vector<TItem> &jitem) const
    {
        return DiscreteSpace<Joint<TItem>>::getItemIndex(jitem);
    }

    template <typename TItem>
    MultiDiscreteSpace<TItem> &MultiDiscreteSpace<TItem>::operator=(const MultiDiscreteSpace<TItem> &sp)
    {
        this->all_items_.clear();
        this->setSpaces(sp.getSpaces());
        return *this;
    }

    template <typename TItem>
    std::string MultiDiscreteSpace<TItem>::str() const
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

    template <typename TItem>
    bool MultiDiscreteSpace<TItem>::contains(const std::vector<TItem> &item) const
    {
        return std::find(this->list_items_.begin(), this->list_items_.end(), item) != this->list_items_.end() ? true : false;
    }

} // namespace sdm
