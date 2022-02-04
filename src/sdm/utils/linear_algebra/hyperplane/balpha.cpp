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

    double bAlpha::getBetaValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<Action>& u, const std::shared_ptr<POMDPInterface>& pomdp, number t)
    {
        // Compute \beta_t(x,o,u) = R(x,u) + \gamma \sum_{y, z} p^{uz}_{xy} \alpha_{t+1}(y, (o,u,z))
        double next_expected_value = 0.0;

        // Go over all hidden state reachable next state
        for (const auto &y : pomdp->getReachableStates(x, u, t))
        {
            // Go over all observation reachable observation
            for (const auto &z : pomdp->getReachableObservations(x, u, y, t))
            {
                // Determine the best next hyperplan for the next belief and compute the dynamics and probability of this best next hyperplan
                next_expected_value += this->getValueAt(y, nullptr) * pomdp->getDynamics(x, u, y, z, t);
            }
        }
        return pomdp->getReward(x, u, t) + pomdp->getDiscount(t) * next_expected_value;
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
