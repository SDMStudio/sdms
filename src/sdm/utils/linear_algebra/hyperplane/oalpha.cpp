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
        {
            this->repr.emplace(o, MappedVector<std::shared_ptr<State>, double>(default_value));
        }
        this->repr.at(o).setValueAt(x, value);
    }

    double oAlpha::getBetaValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action>& u, const std::shared_ptr<POMDPInterface>& pomdp, number t)
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
                next_expected_value += this->getValueAt(y, o->expand(z)) * pomdp->getDynamics(x, u, y, z, t);
            }
        }
        return pomdp->getReward(x, u, t) + pomdp->getDiscount(t) * next_expected_value;
    }

    size_t oAlpha::hash(double precision) const
    {
        if (precision < 0)
            precision = oAlpha::PRECISION;

        size_t seed = 0;
        std::map<std::shared_ptr<HistoryInterface>, MappedVector<std::shared_ptr<State>, double>> ordered(this->repr.begin(), this->repr.end());
        for (const auto &v : ordered)
        {
            // Combine the hash of the current vector with the hashes of the previous ones
            sdm::hash_combine(seed, v.first);
            sdm::hash_combine(seed, std::hash<MappedVector<std::shared_ptr<State>, double>>()(v.second, precision));
        }
        return seed;
    }

    bool oAlpha::isEqual(const oAlpha &other, double precision) const
    {
        if (other.size() > this->size())
        {
            return other.isEqual(*this, precision);
        }

        if (precision < 0)
            precision = oAlpha::PRECISION;

        for (const auto &o_balpha : this->repr)
        {
            for (const auto &x_value : o_balpha.second)
            {
                if (std::abs(x_value.second - other.getValueAt(x_value.first, o_balpha.first)) > precision)
                {
                    return false;
                }
            }
        }
        return true;
    }

    bool oAlpha::isEqual(const std::shared_ptr<Hyperplane> &other, double precision) const
    {
        auto other_oalpha = std::static_pointer_cast<oAlpha>(other);
        if (other_oalpha == nullptr)
            return false;
        else
            return this->isEqual(*other_oalpha, precision);
    }

    size_t oAlpha::size() const
    {
        return this->repr.size();
    }

    std::string oAlpha::str() const
    {
        std::ostringstream res;
        res << "<plan>" << std::endl;

        for (const auto &hist_belief : this->repr)
        {
            res << "\t<" << hist_belief.first->short_str() << ">" << std::endl;
            for (const auto &b_value : hist_belief.second)
            {
                res << "\t\t" << b_value.first->str() << " : " << b_value.second << std::endl;
            }
            res << "\t</>" << std::endl;
        }

        res << "</plan>" << std::endl;
        return res.str();
    }

} // namespace sdm
