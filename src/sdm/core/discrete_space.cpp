#include <sdm/core/discrete_space.hpp>
#include <sdm/common.hpp>

namespace sdm
{
    DiscreteSpace::DiscreteSpace() : num_elements_(0) {}

    DiscreteSpace::DiscreteSpace(number num_elements) : num_elements_(num_elements) {}

    DiscreteSpace::DiscreteSpace(const std::vector<std::string> &e_names) : num_elements_(e_names.size())
    {
        for (number idx = 0; idx < this->num_elements_; ++idx)
        {
            this->names_bimap_.insert(name2index(e_names[idx], idx));
        }
    }

    number DiscreteSpace::getLength() const
    {
        return this->getNumElements();
    }

    std::vector<number> DiscreteSpace::getDim() const
    {
        return {1};
    }

    number DiscreteSpace::getNumElements() const
    {
        return this->num_elements_;
    }

    number DiscreteSpace::getElementIndex(const std::string &e_name) const
    {
        return this->names_bimap_.empty() ? 0 : this->names_bimap_.left.at(e_name);
    }

    std::string DiscreteSpace::getElementName(number index) const
    {
        return this->names_bimap_.empty() ? std::to_string(index) : this->names_bimap_.right.at(index);
    }

    number DiscreteSpace::sample() const
    {
        std::uniform_int_distribution<> distrib(0, this->getLength() - 1);
        return distrib(sdm::common::global_urng());
    }

    void DiscreteSpace::setNumElements(number num_elements)
    {
        this->num_elements_ = num_elements;
    }

    void DiscreteSpace::setElementsNames(const std::vector<std::string> &e_names)
    {
        if (this->num_elements_ == 0)
        {
            this->setNumElements(e_names.size());
        }
        else
        {
            assert(this->num_elements_ == e_names.size());
        }

        for (number idx = 0; idx < this->num_elements_; ++idx)
        {
            this->names_bimap_.insert(name2index(e_names[idx], idx));
        }
    }

    DiscreteSpace &DiscreteSpace::operator=(const DiscreteSpace &sp)
    {
        this->setNumElements(sp.getNumElements());
        this->names_bimap_.clear();
        for (number idx = 0; idx < this->num_elements_; ++idx)
        {
            this->names_bimap_.insert(name2index(sp.getElementName(idx), idx));
        }
        return *this;
    }

    bool DiscreteSpace::operator==(const DiscreteSpace &sp) const
    {
        if (this->getNumElements() != sp.getNumElements())
        {
            return false;
        }
        else
        {
            for (number id = 0; id < this->getNumElements(); id++)
            {
                if (this->getElementName(id) != sp.getElementName(id))
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

    std::string DiscreteSpace::str() const
    {
        std::ostringstream res;
        res << "DiscreteSpace(" << this->getNumElements() << ")";
        res << "[";
        for (number i = 0; i < this->getNumElements(); i++)
        {
            res << "" << this->getElementName(i) << " ";
        }
        res << "]";
        return res.str();
    }

    std::ostream &operator<<(std::ostream &os, const DiscreteSpace &sp)
    {
        os << sp.str();
        return os;
    }
} // namespace sdm
