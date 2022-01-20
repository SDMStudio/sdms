#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/state/interface/history_interface.hpp>
#include <sdm/utils/struct/recursive_map.hpp>
#include <sdm/utils/linear_algebra/mapped_matrix.hpp>

namespace sdm
{
    class Hyperplane : public BaseState<RecursiveMap<std::shared_ptr<HistoryInterface>, MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Action>, double>>>
    {
    public:
        Hyperplane(double default_value) : default_value(default_value)
        {
        }

        double getValueAt(const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<State> &x, const std::shared_ptr<Action> &u)
        {
            auto iter = this->state.find(o);
            if (iter == this->state.end())
                return this->default_value;
            else
                return iter->second.getValueAt(x, u);
        }

        void setValueAt(const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<State> &x, const std::shared_ptr<Action> &u, double value)
        {
            auto iter = this->state.find(o);
            if (iter == this->state.end())
                this->state.emplace(o, MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Action>, double>(default_value));
            else
                this->state.at(o).setValueAt(x, u, value);
        }

    protected:
        double default_value;
    };
}
