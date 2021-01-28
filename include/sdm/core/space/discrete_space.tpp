#include <sdm/core/space/discrete_space.hpp>
#include <sdm/common.hpp>

namespace sdm
{
    template <typename TItem>
    DSpace<TItem>::DSpace() : num_elements_(0) {}

    template <typename TItem>
    DSpace<TItem>::DSpace(number num_elements) : num_elements_(num_elements)
    {
        for (number i = 0; i < num_elements; i++)
        {
            all_items_.push_back(TItem(i));
        }
    }

    template <typename TItem>
    DSpace<TItem>::DSpace(const std::vector<TItem> &items) : num_elements_(items.size()), all_items_(items)
    {
    }

    template <typename TItem>
    DSpace<TItem>::DSpace(const DSpace<TItem> &copy)
    {
        *this = copy;
    }

    template <typename TItem>
    bool DSpace<TItem>::isDiscrete() const
    {
        return true;
    }

    template <typename TItem>
    TItem DSpace<TItem>::sample() const
    {
        assert(this->all_items_.size() > 0);
        if (this->all_items_.size() > 0)
        {
            std::uniform_int_distribution<int>  distr(0, this->all_items_.size()-1);
            return this->all_items_[distr(common::global_urng())];
        }
    }

    template <typename TItem>
    number DSpace<TItem>::getLength() const
    {
        return this->getNumElements();
    }

    template <typename TItem>
    number DSpace<TItem>::getNumElements() const
    {
        return this->num_elements_;
    }

    template <typename TItem>
    std::vector<TItem> DSpace<TItem>::getAll()
    {
        return this->all_items_;
    }

    template <typename TItem>
    std::vector<number> DSpace<TItem>::getDim() const
    {
        return {1};
    }

    template <typename TItem>
    std::string DSpace<TItem>::str() const
    {
        std::ostringstream res;
        res << "DSpace(" << this->getNumElements() << ")";
        res << "[";
        for (number i = 0; i < this->getNumElements(); i++)
        {
            res << "" << this->getElement(i) << " ";
        }
        res << "]";
        return res.str();
    }

    template <typename TItem>
    number DSpace<TItem>::getElementIndex(const TItem &item) const
    {
        for (number i = 0; i < this->all_items_.size(); i++)
        {
            if (this->all_items_[i] == item)
            {
                return i;
            }
        }
    }

    template <typename TItem>
    TItem DSpace<TItem>::getElement(number index) const
    {
        return this->all_items_[index];
    }

    template <typename TItem>
    void DSpace<TItem>::setNumElements(number num_elements)
    {
        this->num_elements_ = num_elements;
    }

    template <typename TItem>
    DSpace<TItem> &DSpace<TItem>::operator=(const DSpace<TItem> &sp)
    {
        this->setNumElements(sp.getNumElements());
        this->all_items_.clear();
        for (number idx = 0; idx < this->getNumElements(); ++idx)
        {
            this->all_items_.push_back(sp.getElement(idx));
        }
        return *this;
    }

    template <typename TItem>
    bool DSpace<TItem>::operator==(const DSpace &sp) const
    {
        if (this->getNumElements() != sp.getNumElements())
        {
            return false;
        }
        else
        {
            for (number id = 0; id < this->getNumElements(); id++)
            {
                if (this->getElement(id) != sp.getElement(id))
                {
                    return false;
                }
            }
            return true;
        }
    }

    template <typename TItem>
    bool DSpace<TItem>::operator!=(const DSpace &sp) const
    {
        return !(operator==(sp));
    }
} // namespace sdm
