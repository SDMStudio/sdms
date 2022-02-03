#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/hyperplane/alpha_vector.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>

namespace sdm
{
    class bAlpha : public AlphaVector
    {
    public:
        bAlpha(double default_value);

        double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &);
        void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &, double value);

    protected:
        MappedVector<std::shared_ptr<State>, double> repr;
    };
}
