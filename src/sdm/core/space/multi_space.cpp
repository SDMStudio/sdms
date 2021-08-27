#include <sdm/core/space/multi_space.hpp>

namespace sdm
{
    MultiSpace::MultiSpace()
    {
    }

    MultiSpace::MultiSpace(const std::vector<std::shared_ptr<Space>> &spaces)
    {
        this->setSpaces(spaces);
    }

    bool MultiSpace::isDiscrete() const
    {
        for (const auto &sp : this->spaces_)
        {
            if (sp->isContinuous())
            {
                return false;
            }
        }
        return true;
    }

    number MultiSpace::getNumSpaces() const
    {
        return this->spaces_.size();
    }

    std::vector<number> MultiSpace::getDim() const
    {
        std::vector<number> dim;
        for (auto sp : this->spaces_)
        {
            dim.push_back(sp->getDim()[0]);
        }
        return dim;
    }

    std::vector<std::shared_ptr<Space>> MultiSpace::getSpaces() const
    {
        return this->spaces_;
    }

    std::shared_ptr<Space> MultiSpace::getSpace(number index) const
    {
        return this->spaces_[index];
    }

    void MultiSpace::setSpaces(const std::vector<std::shared_ptr<Space>> &spaces)
    {
        this->spaces_.clear();
        this->spaces_ = spaces;
    }

    std::string MultiSpace::str() const
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

    MultiSpace &MultiSpace::operator=(const MultiSpace &sp)
    {
        this->setSpaces(sp.getSpaces());
        return *this;
    }

    bool MultiSpace::operator==(const MultiSpace &sp) const
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

    bool MultiSpace::operator!=(const MultiSpace &sp) const
    {
        return !(operator==(sp));
    }
} // namespace sdm
