#include <sdm/utils/linear_algebra/hyperplane/balpha.hpp>

namespace sdm
{
    bAlpha::bAlpha(double default_value) : AlphaVector(default_value)
    {
    }

    double bAlpha::getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &)
    {
        return this->repr.getValueAt(x);
    }

    void bAlpha::setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &, double value)
    {
        this->repr.setValueAt(x, value);
    }
} // namespace sdm
