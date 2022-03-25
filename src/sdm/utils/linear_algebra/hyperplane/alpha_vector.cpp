#include <sdm/utils/linear_algebra/hyperplane/alpha_vector.hpp>

namespace sdm
{
    double AlphaVector::PRECISION = 0.1;

    double AlphaVector::getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u) const
    {
        throw sdm::exception::Exception("Bad call to getValueAt function in AlphaVector.");
    }
    void AlphaVector::setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, double value)
    {
        throw sdm::exception::Exception("Bad call to setValueAt function in AlphaVector.");
    }
} // namespace sdm
