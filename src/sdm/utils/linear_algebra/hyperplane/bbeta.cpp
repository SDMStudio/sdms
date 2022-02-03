#include <sdm/utils/linear_algebra/hyperplane/bbeta.hpp>

namespace sdm
{
    bBeta::bBeta(double default_value) : BetaVector(default_value)
    {
    }

    double bBeta::getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<Action> &u)
    {
        return this->repr.getValueAt(x, u);
    }

    void bBeta::setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<Action> &u, double value)
    {
        this->repr.setValueAt(x, u, value);
    }
} // namespace sdm
