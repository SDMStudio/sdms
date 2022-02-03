#include <sdm/utils/linear_algebra/hyperplane/beta_vector.hpp>

namespace sdm
{
    double BetaVector::getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o)
    {
        throw sdm::exception::Exception("Bad call to getValueAt function in BetaVector.");
    }
    void BetaVector::setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, double value)
    {
        throw sdm::exception::Exception("Bad call to setValueAt function in BetaVector.");
    }
} // namespace sdm
