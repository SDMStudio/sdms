#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/state/interface/history_interface.hpp>
#include <sdm/utils/struct/recursive_map.hpp>
#include <sdm/utils/linear_algebra/mapped_matrix.hpp>

namespace sdm
{
    class AlphaVector
    {
    public:
        AlphaVector(double default_value) : default_value(default_value) {}

        virtual double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o) = 0;
        virtual void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, double value) = 0;

    protected:
        double default_value;
    };
}