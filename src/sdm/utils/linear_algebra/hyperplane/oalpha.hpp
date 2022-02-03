#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/hyperplane/alpha_vector.hpp>
#include <sdm/utils/linear_algebra/hyperplane/balpha.hpp>

namespace sdm
{
    class oAlpha : public AlphaVector
    {
    public:
        oAlpha(double default_value);

        double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o);
        void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, double value);

    protected:
        std::unordered_map<std::shared_ptr<HistoryInterface>, bAlpha> repr;
    };
}
