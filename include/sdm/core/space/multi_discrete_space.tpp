#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/utils/decision_rules/variations2.hpp>

namespace sdm
{
    template <typename TItem>
    MDSpace<TItem>::MDSpace()
    {
    }

    template <typename TItem>
    MDSpace<TItem>::MDSpace(const std::vector<number> &num_elements)
    {
        this->setSpaces(num_elements);
    }

    template <typename TItem>
    MDSpace<TItem>::MDSpace(const std::vector<std::vector<TItem>> &e_names)
    {
        this->setSpaces(e_names);
    }

    template <typename TItem>
    MDSpace<TItem>::MDSpace(const std::vector<std::shared_ptr<DSpace<TItem>>> &spaces)
    {
        this->setSpaces(spaces);
    }

    template <typename TItem>
    number MDSpace<TItem>::joint2single(const std::vector<TItem> &jel) const
    {
        return this->getJointElementIndex(jel);
    }

    template <typename TItem>
    const std::vector<TItem> &MDSpace<TItem>::single2joint(number jel) const
    {
        return this->getJointElement(jel);
    }

    template <typename TItem>
    number MDSpace<TItem>::getNumJElements() const
    {
        return this->num_jelement;
    }

    template <typename TItem>
    number MDSpace<TItem>::getNumElements() const
    {
        return this->getNumSpaces();
    }

    template <typename TItem>
    number MDSpace<TItem>::getNumElements(number idx) const
    {
        return this->getSpace(idx)->getNumElements();
    }

    template <typename TItem>
    number MDSpace<TItem>::getElementIndex(number ag_id, const TItem &item) const
    {
        return this->getSpace(ag_id)->getElementIndex(item);
    }

    template <typename TItem>
    TItem MDSpace<TItem>::getElement(number ag_id, number el_id) const
    {
        return this->getSpace(ag_id)->getElement(el_id);
    }

    template <typename TItem>
    void MDSpace<TItem>::setSpaces(const std::vector<number> &num_elements)
    {
        this->spaces_.clear();
        this->num_jelement = 1;

        for (number ag = 0; ag < num_elements.size(); ++ag)
        {
            this->spaces_.push_back(std::shared_ptr<DSpace<TItem>>(new DSpace<TItem>(num_elements[ag])));
            this->num_jelement *= num_elements[ag];
        }
        this->generateJointElements();
    }

    template <typename TItem>
    void MDSpace<TItem>::setSpaces(const std::vector<int> &num_elements)
    {
        this->spaces_.clear();
        this->num_jelement = 1;

        for (number ag = 0; ag < num_elements.size(); ++ag)
        {
            this->spaces_.push_back(std::shared_ptr<DSpace<TItem>>(new DSpace<TItem>(num_elements[ag])));
            this->num_jelement *= num_elements[ag];
        }
        this->generateJointElements();
    }

    template <typename TItem>
    void MDSpace<TItem>::setSpaces(const std::vector<std::vector<TItem>> &e_names)
    {

        this->num_jelement = 1;
        this->spaces_.clear();

        for (number ag = 0; ag < e_names.size(); ++ag)
        {
            this->spaces_.push_back(std::shared_ptr<DSpace<TItem>>(new DSpace<TItem>(e_names[ag])));
            this->num_jelement *= e_names[ag].size();
        }
        this->generateJointElements();
    }

    template <typename TItem>
    void MDSpace<TItem>::setSpaces(const std::vector<std::shared_ptr<DSpace<TItem>>> &spaces)
    {

        this->num_jelement = 1;
        this->spaces_.clear();

        for (number ag = 0; ag < spaces.size(); ++ag)
        {
            this->spaces_.push_back(spaces[ag]);
            this->num_jelement *= spaces[ag]->getNumElements();
        }
        this->generateJointElements();
    }

    template <typename TItem>
    void MDSpace<TItem>::generateJointElements()
    {
        if (this->joint_items_bimap.size() == 0)
        {
            std::vector<std::vector<TItem>> v_possible_items;
            for (auto sp : this->getSpaces())
            {
                v_possible_items.push_back(sp->getAll());
            }

            number counter = 0;
            Variations<Joint<TItem>> vars(v_possible_items);
            for (auto v = vars.begin(); v != vars.end(); v = vars.next())
            {
                this->joint_items_bimap.insert(jitems2index(*v, counter));
                this->all_items_.push_back(*v);
                counter++;
            }
        }
        else
        {
            std::cerr << "#> Joint items cannot be generated twice.";
        }
    }

    template <typename TItem>
    number MDSpace<TItem>::getJointElementIndex(Joint<TItem> &jitem) const
    {
        typename jitems_bimap::const_iterator iter;

        for (iter = this->joint_items_bimap.begin(); iter != this->joint_items_bimap.end(); ++iter)
        {
            if (iter->left == jitem)
            {
                return iter->right;
            }
        }
        assert(false && "No such joint item !");
    }

    template <typename TItem>
    number MDSpace<TItem>::getJointElementIndex(const std::vector<number> &v) const
    {
        typename jitems_bimap::const_iterator iter;

        for (iter = this->joint_items_bimap.begin(); iter != this->joint_items_bimap.end(); ++iter)
        {
            if (iter->left == v)
            {
                return iter->right;
            }
        }

        assert(false && "No such joint item !");
    }

    template <typename TItem>
    const Joint<TItem> &MDSpace<TItem>::getJointElement(number idx) const
    {
        assert(joint_items_bimap.right.find(idx) != joint_items_bimap.right.end());
        return joint_items_bimap.right.at(idx);
    }

    template <typename TItem>
    std::vector<Joint<TItem>> MDSpace<TItem>::getAll()
    {
        if (this->joint_items_bimap.size() == 0)
        {
            this->generateJointElements();
        }
        return this->all_items_;
    }

    template <typename TItem>
    MDSpace<TItem> &MDSpace<TItem>::operator=(const MDSpace<TItem> &sp)
    {
        this->names_.clear();
        this->joint_items_bimap.clear();
        this->setSpaces(sp.getSpaces());
        return *this;
    }

    template <typename TItem>
    std::string MDSpace<TItem>::str() const
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

} // namespace sdm
