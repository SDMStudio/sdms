#include <sdm/core/multi_discrete_space.hpp>
#include <sdm/core/space.hpp>

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

    number MultiDiscreteSpace::joint2single(const std::vector<number> &) const
    {
        std::cout << "(joint2single) Not implemented method" << std::endl;
    }

    const std::vector<number> &MultiDiscreteSpace::single2joint(number) const
    {
        std::cout << "(single2joint) Not implemented method" << std::endl;
    }

    void MultiDiscreteSpace::generateJointElements(number)
    {
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
    }

    MultiDiscreteSpace &MultiDiscreteSpace::operator=(const MultiDiscreteSpace &sp)
    {
        this->names_.clear();
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
        return os;
    }
} // namespace sdm
