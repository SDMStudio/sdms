#include <sdm/utils/value_function/backup/tabular_backup.hpp>

namespace sdm
{
    TabularBackup::TabularBackup(){}

    TabularBackup::TabularBackup(std::shared_ptr<SolvableByHSVI> world)
    {
        this->world_ = world;
    }

    TabularBackup::~TabularBackup(){}

    std::pair<double, std::shared_ptr<State>> TabularBackup::getMaxAt(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        double value;
        if (std::find(vf->getSupport(t).begin(),vf->getSupport(t).end(),state) == vf->getSupport(t).end() && vf->getInitFunction() != nullptr)
        {
            std::cout<<"Not found";
            value = vf->getInitFunction()->operator()(state,t);
        }
        else
        {
            value = vf->getValueAt(state,t);
        }
        return std::make_pair(value,state);
    }

    double TabularBackup::backup(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        return this->getQValueAt(vf,state, t)->max();
    }

    std::shared_ptr<Action> TabularBackup::getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;

        for (const auto &action : *this->world_->getActionSpaceAt(state, t))
        {
            auto casted_action = action->toAction();
            if (max < (tmp = this->getQValueAt(vf,state, casted_action, t)))
            {
                best_action = casted_action;
                max = tmp;
            }
        }
        return best_action;
    }

}