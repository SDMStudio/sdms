#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/mapped_matrix.hpp>
#include <sdm/utils/linear_algebra/hyperplane/beta_vector.hpp>

namespace sdm
{
    class oBeta : public BetaVector
    {
    public:
        oBeta(double default_value);

        bool isDominated(const Hyperplane &other) const;

        double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u) const;
        void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, double value);

    protected:
        std::unordered_map<std::shared_ptr<HistoryInterface>, MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Action>, double>> repr;
    };
}
