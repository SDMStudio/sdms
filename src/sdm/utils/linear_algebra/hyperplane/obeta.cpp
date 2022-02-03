#include <sdm/utils/linear_algebra/hyperplane/obeta.hpp>

namespace sdm
{
    oBeta::oBeta(double default_value) : BetaVector(default_value)
    {
    }

    double oBeta::getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u)
    {
        auto iter = this->repr.find(o);
        if (iter == this->repr.end())
            return this->default_value;
        else
            return iter->second.getValueAt(x, o, u);
    }

    void oBeta::setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, double value)
    {
        auto iter = this->repr.find(o);
        if (iter == this->repr.end())
            this->repr.emplace(o, bBeta(default_value));
        else
            this->repr.at(o).setValueAt(x, o, u, value);
    }
} // namespace sdm
