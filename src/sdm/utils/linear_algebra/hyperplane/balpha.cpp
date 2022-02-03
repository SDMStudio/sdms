#include <sdm/utils/linear_algebra/hyperplane/balpha.hpp>

namespace sdm
{
    bAlpha::bAlpha(double default_value) : AlphaVector(default_value)
    {
    }

    bool bAlpha::isDominated(const Hyperplane &other) const
    {
        for (const auto &x_value : this->repr)
        {
            if (x_value.second > other.getValueAt(x_value.first, nullptr))
                return false;
        }
        return true;
    }

    double bAlpha::getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &) const
    {
        return this->repr.getValueAt(x);
    }

    void bAlpha::setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &, double value)
    {
        this->repr.setValueAt(x, value);
    }

    size_t bAlpha::hash(double precision) const
    {
        if (precision < 0)
            precision = bAlpha::PRECISION;
        return std::hash<MappedVector<std::shared_ptr<State>, double>>()(this->repr, precision);
    }

    bool bAlpha::isEqual(const std::shared_ptr<Hyperplane> &other, double precision) const
    {
        auto alpha_other = std::static_pointer_cast<bAlpha>(other);
        if (precision < 0)
            precision = bAlpha::PRECISION;
        return this->repr.isEqual(alpha_other->repr, precision);
    }

} // namespace sdm
