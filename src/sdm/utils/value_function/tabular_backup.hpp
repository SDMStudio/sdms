#pragma once

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/backup_base.hpp>

namespace sdm
{
    class TabularBackup : public BackupBase<double>
    {
    public:
        using TData = double;

        TabularBackup(){}
        TabularBackup(std::shared_ptr<SolvableByHSVI> world){this->world_ = world;}

        std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
        {
            double value;
            if (std::find(vf->getSupport(t).begin(),vf->getSupport(t).end(),state) == vf->getSupport(t).end())
            {
                value = vf->getInitFunction()->operator()(state,t);

            }else
            {
                value = vf->getValueAt(state,t);
            }
            return std::make_pair(value,state);
        }

        TData backup(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
        {
            return this->getQValueAt(vf,state, t)->max();
        }

        std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
        {
            std::shared_ptr<Action> best_action;
            double max = -std::numeric_limits<double>::max(), tmp;

            for (const auto &action : *this->world_->getActionSpaceAt(state, t))
            {
                auto casted_action = std::static_pointer_cast<Action>(action);
                if (max < (tmp = this->getQValueAt(vf,state, casted_action, t)))
                {
                    best_action = casted_action;
                    max = tmp;
                }
            }
            return best_action;
        }

        /**
         * @brief Get the q-value at a state
         * 
         * @param state the state
         * @return the action value vector 
         */
        std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValueAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t)
        {
            // Compute Q(s,*)
            std::shared_ptr<MappedVector<std::shared_ptr<Action>, double>> q_s = std::make_shared<MappedVector<std::shared_ptr<Action>, double>>();
            for (const auto &action : *world_->getActionSpaceAt(state, t))
            {
                auto casted_action = std::static_pointer_cast<Action>(action);
                (*q_s)[casted_action] = this->getQValueAt(vf,state, casted_action, t);
            }
            return q_s;
        }

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
        {
            return world_->getReward(state, action, t) + world_->getDiscount(t) * world_->getExpectedNextValue(vf->getptr(), state, action, t);
        }
    };
}