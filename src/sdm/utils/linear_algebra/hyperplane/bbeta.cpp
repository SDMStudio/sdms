#include <sdm/utils/linear_algebra/hyperplane/bbeta.hpp>

namespace sdm
{
    bBeta::bBeta(double default_value) : BetaVector(default_value)
    {
    }

    bool bBeta::isDominated(const Hyperplane &other) const
    {
        for (const auto &x_u_v : this->repr)
        {
            for (const auto &u_v : x_u_v.second)
            {
                if (u_v.second > other.getValueAt(x_u_v.first, nullptr, u_v.first))
                    return false;
            }
        }
        return true;
    }

    double bBeta::getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<Action> &u) const
    {
        return this->repr.getValueAt(x, u);
    }

    void bBeta::setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<Action> &u, double value)
    {
        this->repr.setValueAt(x, u, value);
    }

    size_t bBeta::hash(double precision) const
    {
        return 0;
    }

    bool bBeta::isEqual(const std::shared_ptr<Hyperplane> &other, double precision) const
    {
        if (precision < 0)
            precision = bBeta::PRECISION;

        for (const auto &state_vector : this->repr)
        {
            for (const auto &action_value : state_vector.second)
            {
                if (std::abs(action_value.second - other->getValueAt(state_vector.first, nullptr, action_value.first)) > precision)
                    return false;
            }
        }
        return true;
    }
} // namespace sdm
