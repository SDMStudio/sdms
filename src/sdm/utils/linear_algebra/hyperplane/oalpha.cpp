#include <sdm/utils/linear_algebra/hyperplane/oalpha.hpp>

namespace sdm
{
    oAlpha::oAlpha(double default_value) : AlphaVector(default_value)
    {
    }

    bool oAlpha::isDominated(const Hyperplane &other) const
    {
        for (const auto &o_balpha : this->repr)
        {
            for (const auto &x_value : o_balpha.second)
            {
                if (x_value.second > other.getValueAt(x_value.first, o_balpha.first))
                    return false;
            }
        }
        return true;
    }

    double oAlpha::getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o) const
    {
        auto iter = this->repr.find(o);
        if (iter == this->repr.end())
            return this->default_value;
        else
            return iter->second.getValueAt(x);
    }

    void oAlpha::setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, double value)
    {
        auto iter = this->repr.find(o);
        if (iter == this->repr.end())
            this->repr.emplace(o, MappedVector<std::shared_ptr<State>, double>(default_value));
        else
            this->repr.at(o).setValueAt(x, value);
    }

    size_t oAlpha::hash(double precision) const
    {
        return 0;
    }

    bool oAlpha::isEqual(const std::shared_ptr<Hyperplane> &other, double precision) const
    {
        for (const auto &o_balpha : this->repr)
        {
            for (const auto &x_value : o_balpha.second)
            {
                if (std::abs(x_value.second - other->getValueAt(x_value.first, o_balpha.first)) > precision)
                    return false;
            }
        }
        return true;
    }

} // namespace sdm
