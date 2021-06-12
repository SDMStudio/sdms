#include <sdm/utils/value_function/backup/tabular_backup.hpp>

namespace sdm
{
    TabularBackup::TabularBackup() {}

    TabularBackup::TabularBackup(std::shared_ptr<SolvableByHSVI> world)
    {
        this->world_ = world;
    }

    TabularBackup::~TabularBackup() {}

    std::pair<double, std::shared_ptr<State>> TabularBackup::getMaxAt(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &state, number t)
    {
        double value;
        if (std::find(value_function->getSupport(t).begin(), value_function->getSupport(t).end(), state) == value_function->getSupport(t).end())
        {
            value = value_function->getInitFunction()->operator()(state, t);
        }
        else
        {
            value = value_function->getValueAt(state, t);
        }
        return std::make_pair(value, state);
    }

    double TabularBackup::backup(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &state, number t)
    {
        return this->getQValueAt(value_function, state, t)->max();
    }

    std::shared_ptr<Action> TabularBackup::getBestAction(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &state, number t)
    {
        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;

        for (const auto &action : *this->world_->getActionSpaceAt(state, t))
        {
            auto casted_action = action->toAction();
            if (max < (tmp = this->getQValueAt(value_function, state, casted_action, t)))
            {
                best_action = casted_action;
                max = tmp;
            }
        }
        return best_action;
    }

}