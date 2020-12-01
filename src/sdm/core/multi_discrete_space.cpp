#include <sdm/core/multi_discrete_space.hpp>
#include <sdm/core/space.hpp>
#include <sdm/utils/decision_rules/variations.hpp>

namespace sdm
{
    MultiDiscreteSpace::MultiDiscreteSpace()
    {
    }

    MultiDiscreteSpace::MultiDiscreteSpace(const std::vector<number> &num_elements)
    {
        this->setSpaces(num_elements);
    }

    MultiDiscreteSpace::MultiDiscreteSpace(const std::vector<int> &num_elements)
    {
        this->setSpaces(num_elements);
    }

    MultiDiscreteSpace::MultiDiscreteSpace(const std::vector<std::vector<std::string>> &e_names)
    {
        this->setSpaces(e_names);
    }

    MultiDiscreteSpace::MultiDiscreteSpace(const std::vector<DiscreteSpace> &spaces)
    {
        this->setSpaces(spaces);
    }

    std::vector<number> MultiDiscreteSpace::sample() const
    {
        std::vector<number> sample;
        for (const auto &sp : this->spaces_)
        {
            sample.push_back(sp.sample());
        }
        return sample;
    }

    number MultiDiscreteSpace::joint2single(const std::vector<number> &jel) const
    {
        return this->getJointElementIndex(jel);
    }

    const std::vector<number> &MultiDiscreteSpace::single2joint(number jel) const
    {
        return this->getJointElement(jel);
    }

    number MultiDiscreteSpace::getNumJElements() const
    {
        return this->num_jelement;
    }

    number MultiDiscreteSpace::getNumElements() const
    {
        return this->getNumSpaces();
    }

    number MultiDiscreteSpace::getNumSpaces() const
    {
        return this->spaces_.size();
    }

    std::vector<DiscreteSpace> MultiDiscreteSpace::getSpaces() const
    {
        return this->spaces_;
    }

    const DiscreteSpace &MultiDiscreteSpace::getSpace(number index) const
    {
        return this->spaces_[index];
    }

    number MultiDiscreteSpace::getNumElements(number idx) const
    {
        return this->getSpace(idx).getNumElements();
    }

    number MultiDiscreteSpace::getElementIndex(number ag_id, const std::string &name) const
    {
        return this->getSpace(ag_id).getElementIndex(name);
    }

    std::string MultiDiscreteSpace::getElementName(number ag_id) const
    {
        if (this->names_.size() > 0)
        {
            return this->names_[ag_id];
        }
        else
        {
            return std::to_string(ag_id);
        }
    }

    std::string MultiDiscreteSpace::getElementName(number ag_id, number el_id) const
    {
        return this->getSpace(ag_id).getElementName(el_id);
    }

    void MultiDiscreteSpace::setNames(const std::vector<std::string> &names)
    {
        if (this->getNumSpaces() > 0)
        {
            assert(this->getNumSpaces() == names.size());
        }
        this->names_ = names;
    }

    void MultiDiscreteSpace::setSpaces(const std::vector<DiscreteSpace> &spaces)
    {
        if (this->names_.size() > 0)
        {
            assert(this->names_.size() == spaces.size());
        }
        this->spaces_.clear();
        this->num_jelement = 1;
        this->spaces_ = spaces;

        for (number ag = 0; ag < spaces.size(); ++ag)
        {
            this->num_jelement *= spaces[ag].getNumElements();
        }
        this->generateJointElements();
    }

    void MultiDiscreteSpace::setSpaces(const std::vector<number> &num_elements)
    {
        if (this->names_.size() > 0)
        {
            assert(this->names_.size() == num_elements.size());
        }
        this->spaces_.clear();
        this->num_jelement = 1;

        for (number ag = 0; ag < num_elements.size(); ++ag)
        {
            this->spaces_.push_back(DiscreteSpace(num_elements[ag]));
            this->num_jelement *= num_elements[ag];
        }
        this->generateJointElements();
    }

    void MultiDiscreteSpace::setSpaces(const std::vector<int> &num_elements)
    {
        if (this->names_.size() > 0)
        {
            assert(this->names_.size() == num_elements.size());
        }
        this->spaces_.clear();
        this->num_jelement = 1;

        for (number ag = 0; ag < num_elements.size(); ++ag)
        {
            this->spaces_.push_back(DiscreteSpace(num_elements[ag]));
            this->num_jelement *= num_elements[ag];
        }
        this->generateJointElements();
    }

    void MultiDiscreteSpace::setSpaces(const std::vector<std::vector<std::string>> &e_names)
    {
        if (this->names_.size() > 0)
        {
            assert(this->names_.size() == e_names.size());
        }

        this->num_jelement = 1;
        this->spaces_.clear();

        for (number ag = 0; ag < e_names.size(); ++ag)
        {
            this->spaces_.push_back(DiscreteSpace(e_names[ag]));
            this->num_jelement *= e_names[ag].size();
        }
        this->generateJointElements();
    }

    void MultiDiscreteSpace::generateJointElements()
    {
        if (this->joint_items_bimap.size() == 0)
        {

            number ag;
            number counter = 0;
            std::vector<number> v_agents;
            std::vector<number> v_dspace;
            for (ag = 0; ag < this->getNumSpaces(); ++ag)
            {
                v_agents.push_back(ag);
                v_dspace.push_back(this->getNumElements(ag));
            }
            //! generator of variations for joint items
            variations<std::vector<number>, JointItem> jaction_generator(v_agents, v_dspace);
            std::vector<std::unique_ptr<JointItem>> ja;

            ja.push_back(std::unique_ptr<JointItem>(jaction_generator.begin()));
            this->joint_items_bimap.insert(jitems2index(*ja[counter], counter));

            do
            {
                counter++;
                ja.push_back(std::unique_ptr<JointItem>(jaction_generator.next()));
                if (ja[counter] != nullptr)
                    this->joint_items_bimap.insert(jitems2index(*ja[counter], counter));

            } while (ja[counter] != nullptr);
        }
        else
        {
            std::cerr << "#> Joint items cannot be generated twice.";
        }
    }

    number MultiDiscreteSpace::getJointElementIndex(const std::vector<number> &v) const
    {
        typename jitems_bimap::const_iterator iter;

        for (iter = this->joint_items_bimap.begin(); iter != this->joint_items_bimap.end(); ++iter)
        {
            if (iter->left == v)
            {
                return iter->right;
            }
        }

        return iter->right;
    }

    const JointItem &MultiDiscreteSpace::getJointElement(number idx) const
    {
        assert(joint_items_bimap.right.find(idx) != joint_items_bimap.right.end());
        return joint_items_bimap.right.at(idx);
    }

    MultiDiscreteSpace &MultiDiscreteSpace::operator=(const MultiDiscreteSpace &sp)
    {
        this->names_.clear();
        this->joint_items_bimap.clear();
        this->setSpaces(sp.getSpaces());
        return *this;
    }

    bool MultiDiscreteSpace::operator==(const MultiDiscreteSpace &sp) const
    {
        if (this->getNumSpaces() != sp.getNumSpaces())
        {
            return false;
        }
        else
        {
            for (number id = 0; id < this->getNumSpaces(); id++)
            {
                if (this->getSpace(id) != sp.getSpace(id) || this->getElementName(id) != sp.getElementName(id))
                {
                    return false;
                }
            }
            return true;
        }
    }

    bool MultiDiscreteSpace::operator!=(const MultiDiscreteSpace &sp) const
    {
        return !(operator==(sp));
    }

    std::ostream &operator<<(std::ostream &os, const MultiDiscreteSpace &sp)
    {
        os << "MultiDiscreteSpace(" << sp.getNumSpaces() << ")";
        os << "[";
        for (number i = 0; i < sp.getNumSpaces(); i++)
        {
            os << "\n\t" << sp.getElementName(i) << " : " << sp.getSpace(i);
        }
        os << "\n]";
        return os;
    }
} // namespace sdm
