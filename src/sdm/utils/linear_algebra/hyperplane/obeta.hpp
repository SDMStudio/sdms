#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/hyperplane/beta_vector.hpp>
#include <sdm/utils/linear_algebra/hyperplane/bbeta.hpp>

namespace sdm
{
    class oBeta : public BetaVector
    {
    public:
        oBeta(double default_value);

        double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u);
        void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, double value);

    protected:
        std::unordered_map<std::shared_ptr<HistoryInterface>, bBeta> repr;
    };
}
