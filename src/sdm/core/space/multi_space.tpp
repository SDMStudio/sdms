#include <sdm/core/space/multi_space.hpp>

namespace sdm
{
    template <typename TSpace>
    MultiSpace<TSpace>::MultiSpace()
    {
    }

    template <typename TSpace>
    MultiSpace<TSpace>::MultiSpace(const std::vector<std::shared_ptr<TSpace>> &spaces)
    {
        this->setSpaces(spaces);
    }

    template <typename TSpace>
    bool MultiSpace<TSpace>::isDiscrete() const
    {
        for (auto sp : this->spaces_)
        {
            if (sp->isContinuous())
            {
                return false;
            }
        }
        return true;
    }

    template <typename TSpace>
    number MultiSpace<TSpace>::getNumSpaces() const
    {
        return this->spaces_.size();
    }

    template <typename TSpace>
    std::vector<number> MultiSpace<TSpace>::getDim() const
    {
        std::vector<number> dim;
        for (auto sp : this->spaces_)
        {
            dim.push_back(sp->getDim()[0]);
        }
        return dim;
    }

    template <typename TSpace>
    std::vector<std::shared_ptr<TSpace>> MultiSpace<TSpace>::getSpaces() const
    {
        return this->spaces_;
    }

    template <typename TSpace>
    std::shared_ptr<TSpace> MultiSpace<TSpace>::getSpace(number index) const
    {
        return this->spaces_[index];
    }

    template <typename TSpace>
    void MultiSpace<TSpace>::setSpaces(const std::vector<std::shared_ptr<TSpace>> &spaces)
    {
        this->spaces_.clear();
        this->spaces_ = spaces;
    }

    template <typename TSpace>
    std::vector<typename TSpace::value_type> MultiSpace<TSpace>::getAll() const
    {
        std::vector<typename TSpace::value_type> all_states;
        for (const auto &space : this->getSpaces())
        {
            for (const auto &s : space->getAll())
            {
                all_states.push_back(s);
            }
        }
        return all_states;
    }

    template <typename TSpace>
    std::string MultiSpace<TSpace>::str() const
    {
        std::ostringstream res;
        res << "MultiSpace(" << this->getNumSpaces() << ")";
        res << "[";
        for (number i = 0; i < this->getNumSpaces(); i++)
        {
            res << "\n\t" << i << " : " << this->getSpace(i)->str();
        }
        res << "\n]";
        return res.str();
    }

    template <typename TSpace>
    MultiSpace<TSpace> &MultiSpace<TSpace>::operator=(const MultiSpace<TSpace> &sp)
    {
        this->names_.clear();
        this->setSpaces(sp.getSpaces());
        return *this;
    }

    template <typename TSpace>
    bool MultiSpace<TSpace>::operator==(const MultiSpace<TSpace> &sp) const
    {
        if (this->getNumSpaces() != sp.getNumSpaces())
        {
            return false;
        }
        else
        {
            for (number id = 0; id < this->getNumSpaces(); id++)
            {
                if (this->getSpace(id)->operator!=(*sp.getSpace(id)))
                {
                    return false;
                }
            }
            return true;
        }
    }

    template <typename TSpace>
    bool MultiSpace<TSpace>::operator!=(const MultiSpace<TSpace> &sp) const
    {
        return !(operator==(sp));
    }
} // namespace sdm
