#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/hyperplane/beta_vector.hpp>
#include <sdm/utils/linear_algebra/mapped_matrix.hpp>

namespace sdm
{
    class bBeta : public BetaVector
    {
    public:
        bBeta(double default_value);

        bool isDominated(const Hyperplane &other) const;

        double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u) const;
        void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, double value);

        size_t hash(double precision) const;

        bool isEqual(const bBeta &other, double precision) const;
        bool isEqual(const std::shared_ptr<Hyperplane> &other, double precision) const;

        size_t size() const;
        std::string str() const;

    protected:
        MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Action>, double> repr;
    };
}
