#include <sdm/utils/value_function/backup/tabular_backup.hpp>

namespace sdm
{
    TabularBackup::TabularBackup() {}

    TabularBackup::TabularBackup(const std::shared_ptr<SolvableByHSVI> &world)
    {
        this->world_ = world;
    }

    TabularBackup::~TabularBackup() {}

    std::pair<double, std::shared_ptr<State>> TabularBackup::getMaxAt(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        double value;
        if (std::find(vf->getSupport(t).begin(), vf->getSupport(t).end(), state) == vf->getSupport(t).end() && vf->getInitFunction() != nullptr)
        {
            value = vf->getInitFunction()->operator()(state, t);
        }
        else
        {
            value = vf->getValueAt(state, t);
        }
        return std::make_pair(value, state);
    }

    double TabularBackup::backup(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        auto value_max = this->getQValueAt(vf, state, this->getBestAction(vf, state, t), t);
        std::cout << "VMAX " << value_max << std::endl;

        return value_max;
    }

    std::shared_ptr<Action> TabularBackup::getBestAction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        // return this->getQValueAt(vf,state, t)->argmax();

        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;
        auto action_space = this->world_->getActionSpaceAt(state, t);

        for (const auto &action : *action_space)
        {
            auto casted_action = action->toAction();
            if (max < (tmp = this->getQValueAt(vf, state, casted_action, t)))
            {
                best_action = casted_action;
                max = tmp;
            }
        }
        return best_action;
    }

}